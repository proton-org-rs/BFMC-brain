#!/usr/bin/env python3
"""
Auto Drive Mode Script

This script puts the car into AUTO mode (autonomous driving) and returns it to STOP mode
when exiting with Ctrl+C (KeyboardInterrupt).

Usage:
    cd src/Brain
    python -m src.examples.run_auto_drive
"""

import logging
import sys
import os
import time
import multiprocessing as mp
from multiprocessing import Queue, Event

# Setup multiprocessing
try:
    mp.set_start_method("fork", force=True)
except RuntimeError:
    pass

# Setup path
if os.path.basename(os.getcwd()) != 'Brain':
    brain_root = os.path.join(os.getcwd(), 'src', 'Brain')
    if os.path.isdir(brain_root):
        os.chdir(brain_root)
sys.path.append('.')

from src.gateway.processGateway import processGateway
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.data.AutonomousDriving.processAutonomousDriving import processAutonomousDriving
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import Klem, SpeedMotor, Brake


def main():
    """
    Auto Drive Mode - starts autonomous driving and stops on Ctrl+C.
    
    Features:
    - Enables motor (KL30)
    - Starts autonomous driving process
    - Drives forward, stops at stop lines, waits 3s, continues
    - Ignores stop lines for 5s after resuming
    - Returns to STOP mode on Ctrl+C
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger("auto_drive")

    # Queues
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
        "Log": Queue(),
    }

    # Ready events
    gateway_ready = Event()
    serial_ready = Event()
    dashboard_ready = Event()
    autonomous_ready = Event()
    dashboard_ready.set()  # We don't have dashboard in this script

    # Processes
    gw = None
    sh = None
    auto_drive = None

    try:
        # Start Gateway
        logger.info("Starting Gateway...")
        gw = processGateway(queueList, logger, gateway_ready)
        gw.daemon = True
        gw.start()

        # Start Serial Handler
        logger.info("Starting Serial Handler...")
        sh = processSerialHandler(queueList, logger, serial_ready, dashboard_ready, debugging=False)
        sh.daemon = True
        sh.start()

        # Wait for serial handler
        logger.info("Waiting for Serial Handler to be ready...")
        ready = serial_ready.wait(timeout=15)
        if not ready:
            logger.error("Serial Handler not ready after timeout!")
            raise RuntimeError("Serial Handler initialization failed")
        
        logger.info("Serial Handler ready!")
        time.sleep(2.0)  # Wait longer for serial threads to fully initialize

        # Start Autonomous Driving Process
        # Note: The thread will enable KL30 automatically when it starts
        logger.info("Starting Autonomous Driving Process...")
        auto_drive = processAutonomousDriving(queueList, logger, autonomous_ready, debugging=True)
        auto_drive.daemon = True
        auto_drive.start()

        # Wait for autonomous driving to be ready
        logger.info("Waiting for Autonomous Driving to initialize...")
        auto_ready = autonomous_ready.wait(timeout=10)
        if not auto_ready:
            logger.warning("Autonomous driving ready event not set, but continuing...")

        print("\n" + "=" * 70)
        print("🚗 AUTO DRIVE MODE ACTIVE")
        print("=" * 70)
        print("\nThe car is now in autonomous driving mode:")
        print("  • Driving forward at constant speed")
        print("  • Monitoring IR sensor for stop lines")
        print("  • Stops for 3 seconds when line detected")
        print("  • Ignores lines for 5 seconds after resuming")
        print("\n⚠️  Press Ctrl+C to STOP the car and exit")
        print("=" * 70 + "\n")

        # Keep running until Ctrl+C
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n")
        logger.info("=" * 70)
        logger.info("🛑 STOPPING - KeyboardInterrupt received")
        logger.info("=" * 70)

    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)

    finally:
        logger.info("Shutting down and returning to STOP mode...")

        # Create senders to stop the car
        try:
            klem_sender = messageHandlerSender(queueList, Klem)
            speed_sender = messageHandlerSender(queueList, SpeedMotor)
            brake_sender = messageHandlerSender(queueList, Brake)

            # Emergency stop sequence
            logger.info("Sending BRAKE command...")
            brake_sender.send("0")
            time.sleep(0.1)

            logger.info("Setting speed to 0...")
            speed_sender.send("0")
            time.sleep(0.1)

            logger.info("Disabling engine (KL=0)...")
            klem_sender.send("0")
            time.sleep(0.2)

        except Exception as e:
            logger.error(f"Error during stop sequence: {e}")

        # Stop processes - order matters! Stop autonomous driving first
        logger.info("Stopping processes...")

        if auto_drive is not None:
            try:
                logger.info("Stopping Autonomous Driving process...")
                auto_drive.stop()
                time.sleep(0.5)  # Give threads time to cleanup
                auto_drive.join(timeout=5)
                if auto_drive.is_alive():
                    logger.warning("Autonomous Driving process didn't stop gracefully, terminating...")
                    auto_drive.terminate()
                    auto_drive.join(timeout=2)
                logger.info("Autonomous Driving process stopped.")
            except Exception as e:
                logger.error(f"Error stopping auto_drive: {e}")

        if sh is not None:
            try:
                logger.info("Stopping Serial Handler...")
                sh.stop()
                sh.join(timeout=3)
                if sh.is_alive():
                    sh.terminate()
                    sh.join(timeout=1)
                logger.info("Serial Handler stopped.")
            except Exception as e:
                logger.error(f"Error stopping serial handler: {e}")

        if gw is not None:
            try:
                logger.info("Stopping Gateway...")
                gw.stop()
                gw.join(timeout=3)
                if gw.is_alive():
                    gw.terminate()
                    gw.join(timeout=1)
                logger.info("Gateway stopped.")
            except Exception as e:
                logger.error(f"Error stopping gateway: {e}")

        logger.info("=" * 70)
        logger.info("✓ Car stopped. Goodbye!")
        logger.info("=" * 70)


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("BFMC AUTO DRIVE MODE")
    print("=" * 70)
    print("\nThis script will:")
    print("  1. Start the autonomous driving system")
    print("  2. Drive forward at constant speed")
    print("  3. Stop at stop lines (3 seconds)")
    print("  4. Continue driving (ignoring lines for 5s)")
    print("  5. Return to STOP mode on Ctrl+C")
    print("\n⚠️  SAFETY WARNING:")
    print("  - Make sure the car is in a safe, open area")
    print("  - Be ready to catch the car if needed")
    print("  - Keep hands away from wheels")
    print("=" * 70)
    
    response = input("\nPress ENTER to start AUTO DRIVE mode (or Ctrl+C to cancel)...")
    
    main()
