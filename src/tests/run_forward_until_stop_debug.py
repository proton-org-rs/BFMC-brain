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
    DEBUG version that shows detailed status of commands being sent
    
    Minimal runner that starts gateway + serial and sends real commands
    to drive forward until a simulated stop condition.

    Steps:
    - Start Gateway and SerialHandler processes
    - Enable engine (KL=30)
    - Send speed=20 and steer=0 to go straight
    - After a timeout (simulated stop line), set speed=0 and KL=0
    """
    # Configure detailed logging
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger("forward_until_stop_debug")

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
    dashboard_ready.set()  # inform serial handler that dashboard is "ready"

    # Start processes
    logger.info("=" * 60)
    logger.info("STARTING PROCESSES")
    logger.info("=" * 60)
    
    gw = processGateway(queueList, logger, gateway_ready)
    sh = processSerialHandler(queueList, logger, serial_ready, dashboard_ready, debugging=True)  # Enable debug

    gw.daemon = False
    sh.daemon = False

    gw.start()
    logger.info("Gateway process started")
    
    sh.start()
    logger.info("Serial handler process started")

    # Wait until serial handler reports ready
    logger.info("Waiting for serial handler to be ready...")
    ready = serial_ready.wait(timeout=10)
    if not ready:
        logger.error("Serial handler not ready after timeout!")
        raise RuntimeError("Serial handler initialization failed")
    
    logger.info("✓ Serial handler ready")
    logger.info("Waiting for threads to initialize...")
    time.sleep(1.5)  # Give threads time to fully initialize
    logger.info("✓ Threads should be initialized")

    # Senders
    klem_sender = messageHandlerSender(queueList, Klem)
    control_sender = messageHandlerSender(queueList, Control)

    try:
        logger.info("=" * 60)
        logger.info("COMMAND SEQUENCE")
        logger.info("=" * 60)
        
        # Enable engine (KL=30)
        logger.info("Step 1: Enabling engine (KL=30)...")
        klem_sender.send("30")
        logger.debug("  -> KL=30 command sent to queue")
        
        # CRITICAL: Wait for engine to enable and sensors to load
        # threadWrite.load_config("sensors") is called after KL=30
        logger.info("  -> Waiting 1.0s for engine initialization...")
        time.sleep(1.0)
        logger.info("✓ Engine should be enabled now")

        # Send control command (same format as dashboard)
        # All values multiplied by 10 (like dashboard does)
        logger.info("Step 2: Sending VCD control command")
        logger.info("  -> Time=5s, Speed=20cm/s, Steer=0°")
        control_sender.send({
            "Time": str(5 * 10),      # 5 seconds * 10
            "Speed": str(20 * 10),    # 20 cm/s * 10
            "Steer": str(0 * 10)      # 0 degrees * 10
        })
        logger.debug("  -> Control command sent to queue")
        
        # Verify commands were sent
        time.sleep(0.2)
        logger.info("✓ VCD command sent")

        # Run until motion complete
        drive_time_s = 5.0 + 1.0  # 5s motion + 1s margin
        logger.info("=" * 60)
        logger.info(f"VEHICLE IN MOTION for ~{drive_time_s} seconds...")
        logger.info("=" * 60)
        
        for i in range(int(drive_time_s)):
            time.sleep(1.0)
            logger.debug(f"  ... {i+1}s elapsed")

        # Disable engine after motion complete
        logger.info("=" * 60)
        logger.info("STOPPING VEHICLE")
        logger.info("=" * 60)
        
        logger.info("Step 3: Disabling engine (KL=0)")
        klem_sender.send("0")
        logger.debug("  -> KL=0 command sent to queue")
        
        time.sleep(0.3)
        logger.info("✓ Stop command sent")

        logger.info("=" * 60)
        logger.info("SHUTTING DOWN PROCESSES")
        logger.info("=" * 60)
        
    except Exception as e:
        logger.error(f"Error during execution: {e}", exc_info=True)
        
    finally:
        # Gracefully stop processes
        logger.info("Stopping serial handler...")
        sh.stop()
        
        logger.info("Stopping gateway...")
        gw.stop()

        # Join processes with fallback terminate
        logger.debug("Joining serial handler (timeout=3s)...")
        sh.join(timeout=3)
        
        logger.debug("Joining gateway (timeout=3s)...")
        gw.join(timeout=3)

        if sh.is_alive():
            logger.warning("Serial handler still alive, terminating...")
            sh.terminate()
            sh.join(timeout=1)
        else:
            logger.info("✓ Serial handler stopped cleanly")
            
        if gw.is_alive():
            logger.warning("Gateway still alive, terminating...")
            gw.terminate()
            gw.join(timeout=1)
        else:
            logger.info("✓ Gateway stopped cleanly")
            
        logger.info("=" * 60)
        logger.info("DONE")
        logger.info("=" * 60)


if __name__ == "__main__":
    main()
