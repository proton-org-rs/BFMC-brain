import logging
import sys
import os
import time
import multiprocessing as mp
from multiprocessing import Queue, Event

# Local imports from project
# Ensure Brain package root is in sys.path when running from workspace root
# When executed from workspace root, change CWD to Brain root for relative paths
# Force fork start method on Linux/RPi to avoid resource_sharer/DupFd glitches
try:
    mp.set_start_method("fork", force=True)
except RuntimeError:
    # Already set; ignore
    pass

if os.path.basename(os.getcwd()) != 'Brain':
    brain_root = os.path.join(os.getcwd(), 'src', 'Brain')
    if os.path.isdir(brain_root):
        os.chdir(brain_root)
sys.path.append('.')
from src.gateway.processGateway import processGateway
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import Klem, Control


def main():
    """
    Minimal runner that starts gateway + serial and sends real commands
    to drive forward until a simulated stop condition.

    Steps:
    - Start Gateway and SerialHandler processes
    - Enable engine (KL=30)
    - Send speed=20 and steer=0 to go straight
    - After a timeout (simulated stop line), set speed=0 and KL=0
    """
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("forward_until_stop")

    # Queues
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
        "Log": Queue(),
    }

    # Ready events
    gateway_ready = Event()  # not used by gateway
    serial_ready = Event()
    dashboard_ready = Event()
    dashboard_ready.set()  # inform serial handler that dashboard is "ready"

    # Start processes
    gw = processGateway(queueList, logger, gateway_ready)
    sh = processSerialHandler(queueList, logger, serial_ready, dashboard_ready, debugging=False)

    gw.daemon = False
    sh.daemon = False

    gw.start()
    sh.start()

    # Wait until serial handler reports ready
    logger.info("Waiting for serial handler to be ready...")
    ready = serial_ready.wait(timeout=10)
    if not ready:
        logger.error("Serial handler not ready after timeout!")
        raise RuntimeError("Serial handler initialization failed")
    
    logger.info("Serial handler ready, waiting for threads to initialize...")
    time.sleep(1.5)  # Give threads time to fully initialize

    # Senders
    klem_sender = messageHandlerSender(queueList, Klem)
    control_sender = messageHandlerSender(queueList, Control)

    try:
        # Enable engine (KL=30)
        logger.info("Enabling engine (KL=30)...")
        klem_sender.send("30")
        
        # CRITICAL: Wait for engine to enable and sensors to load
        # threadWrite.load_config("sensors") is called after KL=30
        time.sleep(1.0)
        
        logger.info("Engine enabled, ready to drive")

        # Send control command with time, speed, steer (same format as dashboard)
        # All values are multiplied by 10 (like dashboard does)
        # time: seconds to run (multiplied by 10)
        # speed: cm/s [-50, 50] (multiplied by 10)
        # steer: degrees [-25, 25] (multiplied by 10)
        
        logger.info("Sending VCD control: time=5s, speed=20cm/s, steer=0°")
        control_sender.send({
            "Time": str(5 * 10),      # 5 seconds * 10
            "Speed": str(20 * 10),    # 20 cm/s * 10
            "Steer": str(0 * 10)      # 0 degrees * 10
        })
        
        # Wait while the vehicle is driving
        # (The VCD command handles timing internally, but we wait to observe)
        drive_time_s = 5.0 + 1.0  # 5s drive + 1s margin for command processing
        logger.info(f"Vehicle is executing VCD command for ~{drive_time_s}s...")
        time.sleep(drive_time_s)

        # Disable engine after motion complete
        logger.info("Disabling engine (KL=0)...")
        klem_sender.send("0")
        time.sleep(0.3)

        logger.info("Stop command sent. Shutting down processes...")
    finally:
        # Gracefully stop processes
        sh.stop()
        gw.stop()

        # Join processes with fallback terminate
        sh.join(timeout=3)
        gw.join(timeout=3)

        if sh.is_alive():
            logger.warning("Serial handler still alive, terminating...")
            sh.terminate()
            sh.join(timeout=1)
        if gw.is_alive():
            logger.warning("Gateway still alive, terminating...")
            gw.terminate()
            gw.join(timeout=1)


if __name__ == "__main__":
    main()
