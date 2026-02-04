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
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import Klem, ToggleImuData, ImuData


def main():
    """
    IMU toggle test runner with real hardware.
    
    This test demonstrates:
    - Enabling IMU via #imu:1;;
    - Reading IMU data from NUCLEO
    - Disabling IMU via #imu:0;;
    - Properly shutting down
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger("imu_test")

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
    time.sleep(1.0)

    # Sender for toggling IMU
    toggle_imu_sender = messageHandlerSender(queueList, ToggleImuData)
    klem_sender = messageHandlerSender(queueList, Klem)
    
    # Subscriber for IMU data
    imu_data_subscriber = messageHandlerSubscriber(
        queueList,
        ImuData,
        "lastOnly",
        True
    )

    try:
        # Enable engine
        logger.info("=" * 70)
        logger.info("ENABLING ENGINE (KL=30)")
        logger.info("=" * 70)
        klem_sender.send("30")
        time.sleep(0.5)  # Reduced from 1.0s


        # Enable IMU
        logger.info("=" * 70)
        logger.info("ENABLING IMU (sending #imu:1;;)")
        logger.info("=" * 70)
        toggle_imu_sender.send(1)
        time.sleep(0.3)
        
        logger.info("Listening for IMU data for 5 seconds...")
        start_time = time.time()
        data_received = 0
        last_data = None
        
        # Listen for IMU data
        while time.time() - start_time < 5.0:
            imu_data = imu_data_subscriber.receive()
            
            if imu_data is not None and imu_data != last_data:
                data_received += 1
                last_data = imu_data
                logger.info(f"IMU Data #{data_received}: {imu_data}")
            
            time.sleep(0.1)  # 10 Hz check rate
        
        logger.info(f"\nReceived {data_received} IMU data packets in 5 seconds")
        
        # Disable IMU
        logger.info("\n" + "=" * 70)
        logger.info("DISABLING IMU (sending #imu:0;;)")
        logger.info("=" * 70)
        toggle_imu_sender.send(0)
        time.sleep(0.3)
        
        logger.info("Checking if IMU data stopped (2 seconds)...")
        start_time = time.time()
        data_after_disable = 0
        
        while time.time() - start_time < 2.0:
            imu_data = imu_data_subscriber.receive()
            
            if imu_data is not None and imu_data != last_data:
                data_after_disable += 1
                last_data = imu_data
                logger.warning(f"⚠️  IMU Data still coming: {imu_data}")
            
            time.sleep(0.1)
        
        # Summary
        logger.info("\n" + "=" * 70)
        logger.info("TEST SUMMARY")
        logger.info("=" * 70)
        
        if data_received > 0:
            logger.info(f"✓ IMU enabled successfully - received {data_received} packets")
        else:
            logger.error("✗ IMU enable failed - no data received")
            
        if data_after_disable == 0:
            logger.info(f"✓ IMU disabled successfully - no data after disable")
        else:
            logger.warning(f"⚠️  IMU disable may have issues - received {data_after_disable} packets after disable")
        
        logger.info("✓ IMU toggle test complete!")
        
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        logger.info("\nShutting down...")
        
        # Make sure IMU is disabled
        try:
            toggle_imu_sender.send(0)  # Disable IMU
            time.sleep(0.1)
        except:
            pass
        
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
    print("\n" + "=" * 70)
    print("BFMC IMU TOGGLE TEST - REAL HARDWARE")
    print("=" * 70)
    print("\nThis test will:")
    print("  1. Enable IMU sensor (#imu:1;;)")
    print("  2. Listen for IMU data for 5 seconds")
    print("  3. Disable IMU sensor (#imu:0;;)")
    print("  4. Verify IMU data stopped")
    print("  5. Report test results")
    print("\n⚠️  Make sure:")
    print("  - NUCLEO is connected via serial")
    print("  - IMU sensor is properly connected to NUCLEO")
    print("=" * 70)
    
    input("\nPress ENTER to start the test...")
    
    main()
