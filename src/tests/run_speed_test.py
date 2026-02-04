import logging
import sys
import os
import time
import multiprocessing as mp
from multiprocessing import Queue, Event

# Local imports from project
try:
    mp.set_start_method("fork", force=True)
except RuntimeError:
    pass

if os.path.basename(os.getcwd()) != 'Brain':
    brain_root = os.path.join(os.getcwd(), 'src', 'Brain')
    if os.path.isdir(brain_root):
        os.chdir(brain_root)
sys.path.append('.')
from src.gateway.processGateway import processGateway
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import Klem, SpeedMotor, SteerMotor, stopLine


def main():
    """
    Speed test runner - tests multiple speed values to verify they're being sent correctly.
    
    This will test speeds: 5, 10, 15, 20, 30, 40 cm/s
    Each speed is maintained for 3 seconds so you can verify actual movement speed.
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger("speed_test")

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
    dashboard_ready.set()

    # Start processes
    logger.info("Starting processes...")
    gw = processGateway(queueList, logger, gateway_ready)
    sh = processSerialHandler(queueList, logger, serial_ready, dashboard_ready, debugging=False)

    gw.daemon = False
    sh.daemon = False

    gw.start()
    sh.start()

    # Wait for initialization
    logger.info("Waiting for serial handler...")
    ready = serial_ready.wait(timeout=10)
    if not ready:
        logger.error("Serial handler not ready!")
        raise RuntimeError("Initialization failed")
    
    logger.info("Serial handler ready, waiting for threads...")
    time.sleep(1.5)

    # Senders
    klem_sender = messageHandlerSender(queueList, Klem)
    speed_sender = messageHandlerSender(queueList, SpeedMotor)
    steer_sender = messageHandlerSender(queueList, SteerMotor)

    # Speed values to test
    test_speeds = [50, 100, 150, 200, 300, 400, 450, 500]
    test_duration = 3.0  # seconds per speed

    try:
        # Enable engine
        logger.info("=" * 70)
        logger.info("ENABLING ENGINE (KL=30)")
        logger.info("=" * 70)
        klem_sender.send("30")
        time.sleep(1.0)
        
        # Set steering to straight
        logger.info("Setting steering to 0 (straight)")
        steer_sender.send("0")
        time.sleep(0.2)

        # Test each speed
        logger.info("=" * 70)
        logger.info("SPEED TEST SEQUENCE")
        logger.info("=" * 70)
        
        for speed_value in test_speeds:
            logger.info(f"\n>>> Testing speed = {speed_value} cm/s for {test_duration}s")
            logger.info(f"    Sending speed={speed_value}...")
            speed_sender.send(str(speed_value))
            
            # Wait and show progress
            time.sleep(0.3)  # Give command time to be processed
            logger.info(f"    Running at {speed_value} cm/s...")
            
            for i in range(int(test_duration)):
                time.sleep(1.0)
                logger.info(f"      ... {i+1}s / {int(test_duration)}s")
            
            logger.info(f"    ✓ {speed_value} cm/s test complete")

        # Stop
        logger.info("\n" + "=" * 70)
        logger.info("STOPPING")
        logger.info("=" * 70)
        logger.info("Setting speed=0...")
        speed_sender.send("0")
        time.sleep(0.3)
        
        logger.info("Disabling engine (KL=0)...")
        klem_sender.send("0")
        time.sleep(0.3)
        
        logger.info("✓ Speed test complete!")
        logger.info("\nIf speeds were always 5 cm/s, there's a firmware/config issue.")
        logger.info("If speeds varied correctly, the system is working properly.")
        
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        logger.info("\nShutting down...")
        sh.stop()
        gw.stop()
        sh.join(timeout=3)
        gw.join(timeout=3)
        
        if sh.is_alive():
            sh.terminate()
            sh.join(timeout=1)
        if gw.is_alive():
            gw.terminate()
            gw.join(timeout=1)
        
        logger.info("Done.")


if __name__ == "__main__":
    main()
