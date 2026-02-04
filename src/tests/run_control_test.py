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
from src.utils.messages.allMessages import Klem, Control


def main():
    """
    Control test runner - tests various VCD (Velocity, Control, Duration) commands
    matching exactly how the dashboard sends them.
    
    Dashboard format:
    - Time: seconds * 10
    - Speed: cm/s * 10 (range: -50 to 50 cm/s)
    - Steer: degrees * 10 (range: -25 to 25 degrees)
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger("control_test")

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
    control_sender = messageHandlerSender(queueList, Control)

    try:
        # Enable engine
        logger.info("=" * 70)
        logger.info("ENABLING ENGINE (KL=30)")
        logger.info("=" * 70)
        klem_sender.send("30")
        time.sleep(1.0)
        
        # Test sequence
        test_cases = [
            {
                "name": "Test 1: Forward 3s at 10 cm/s",
                "time": 3,
                "speed": 10,
                "steer": 0,
                "description": "Straight movement, low speed"
            },
            {
                "name": "Test 2: Forward 3s at 20 cm/s with slight right turn",
                "time": 3,
                "speed": 20,
                "steer": 5,
                "description": "Right turn, medium speed"
            },
            {
                "name": "Test 3: Forward 3s at 40 cm/s straight",
                "time": 3,
                "speed": 40,
                "steer": 0,
                "description": "High speed straight"
            },
            {
                "name": "Test 4: Backward 2s at -20 cm/s straight",
                "time": 2,
                "speed": -20,
                "steer": 0,
                "description": "Reverse movement"
            },
            {
                "name": "Test 5: Forward 3s at 20 cm/s with left turn",
                "time": 3,
                "speed": 20,
                "steer": -10,
                "description": "Left turn, medium speed"
            },
        ]

        logger.info("=" * 70)
        logger.info("VCD CONTROL TEST SEQUENCE")
        logger.info("=" * 70)
        
        for i, test in enumerate(test_cases, 1):
            logger.info(f"\n>>> {test['name']}")
            logger.info(f"    Description: {test['description']}")
            logger.info(f"    Sending: Time={test['time']}s, Speed={test['speed']}cm/s, Steer={test['steer']}°")
            
            # Send control command (multiply by 10 as dashboard does)
            control_sender.send({
                "Time": str(test['time'] * 10),
                "Speed": str(test['speed'] * 10),
                "Steer": str(test['steer'] * 10)
            })
            
            # Wait for command execution + margin
            wait_time = test['time'] + 1.0
            logger.info(f"    Running for ~{wait_time}s...")
            
            for j in range(int(wait_time)):
                time.sleep(1.0)
                logger.info(f"      ... {j+1}s / {int(wait_time)}s")
            
            logger.info(f"    ✓ {test['name']} complete")
            
            # Small pause between tests
            if i < len(test_cases):
                logger.info("    Pausing 1s before next test...")
                time.sleep(1.0)

        # Stop
        logger.info("\n" + "=" * 70)
        logger.info("STOPPING")
        logger.info("=" * 70)
        logger.info("Disabling engine (KL=0)...")
        klem_sender.send("0")
        time.sleep(0.3)
        
        logger.info("✓ Control test sequence complete!")
        
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
