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
from src.utils.messages.allMessages import Klem, SpeedMotor, SteerMotor, StopLine, ToggleStopLine, Brake, ToggleImuData


def main():
    """
    Line follower test runner with real hardware.
    
    This test demonstrates:
    - Enabling motor via KL30
    - Setting a constant driving speed
    - Reading StopLine detection from IR sensor on NUCLEO
    - Stopping when stop line is detected
    - Properly shutting down
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger("line_follower_test")

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
    brake_sender = messageHandlerSender(queueList, Brake)
    toggle_stopline_sender = messageHandlerSender(queueList, ToggleStopLine)
    toggle_imu_sender = messageHandlerSender(queueList, ToggleImuData)

    # Subscriber for StopLine detection (from IR sensor on NUCLEO)
    stop_line_subscriber = messageHandlerSubscriber(
        queueList,
        StopLine,
        "lastOnly",
        True
    )

    # Configuration
    target_speed = 150  # cm/s
    max_runtime = 15  # seconds (safety timeout)
    steer = 150
    stop_line_detected = False
    start_time = None
    detection_time = None
    last_progress_log = 0  # For throttling progress logs

    try:
        # Enable engine
        logger.info("=" * 70)
        logger.info("ENABLING ENGINE (KL=30)")
        logger.info("=" * 70)
        klem_sender.send("30")
        time.sleep(0.5)  # Reduced from 1.0s
        
        # Enable IR sensor stopLine detection on NUCLEO
        # Sends: #stopLine:1;; to NUCLEO
        logger.info("=" * 70)
        logger.info("ENABLING STOPLINE IR SENSOR (sending #stopLine:1;;)")
        logger.info("=" * 70)
        time.sleep(0.3)  # Reduced from 1.0s

        # Set steering to straight
        logger.info("Setting steering to 0 (straight)")
        steer_sender.send(str(steer))
        time.sleep(0.5)  # Reduced from 0.2s

        logger.info("=" * 70)
        logger.info("DISABLEING IMU (sending #imu:0;;)")
        logger.info("=" * 70)
        toggle_imu_sender.send(0)
        time.sleep(0.3)


        # Start line following
        logger.info("=" * 70)
        logger.info("LINE FOLLOWER TEST")
        logger.info(f"Target speed: {target_speed} cm/s")
        logger.info("Waiting for IR sensor to detect stop line...")
        logger.info("=" * 70)
        
        logger.info(f"Setting speed to {target_speed} cm/s...")
        speed_sender.send(str(target_speed))
        
        start_time = time.time()
        time.sleep(0.15)  # Reduced - give command minimal time to be processed
        
        logger.info(f"Robot driving at {target_speed} cm/s...")
        
        # Main control loop - optimized for fast response
        elapsed = 0
        loop_count = 0
        last_check_value = None  # Track last received value to avoid duplicate processing
        
        while elapsed < max_runtime and not stop_line_detected:
            loop_start = time.time()
            elapsed = loop_start - start_time
            loop_count += 1
            
            # Read IR sensor data from NUCLEO (non-blocking)
            # NUCLEO sends @stopLine:1;; when line detected, @stopLine:0;; otherwise
            stop_line_data = stop_line_subscriber.receive()
            
            if stop_line_data is not None and stop_line_data != last_check_value:
                last_check_value = stop_line_data  # Update to avoid re-processing same value
                
                # Parse the value - NUCLEO sends string "1" or "0"
                is_detected = str(stop_line_data).strip() == "1" or stop_line_data == True
                
                if is_detected:  # Stop line detected!
                    stop_line_detected = True
                    detection_time = elapsed
                    logger.info(f"\n{'*' * 70}")
                    logger.info(f"🛑 STOP LINE DETECTED BY IR SENSOR at {detection_time:.2f}s")
                    logger.info(f"  Loop iterations: {loop_count}")
                    logger.info(f"{'*' * 70}")
                    
                    # IMMEDIATELY BRAKE THE MOTORS
                    logger.info("BRAKING MOTORS...")
                    brake_sender.send("0")  # Emergency brake
                    speed_sender.send("0")  # Set speed to 0
                    break
            
            # Throttled progress indicator - log every 2 seconds max
            if elapsed - last_progress_log >= 2.0:
                logger.info(f"  ... running {elapsed:.1f}s - No stop line yet (loops: {loop_count})")
                last_progress_log = elapsed
            
            # Minimal sleep for high-frequency checking (100 Hz)
            # This allows near-instant response to IR sensor detection
            time.sleep(0.01)

        # Stop the robot (in case we exited due to timeout)
        logger.info("\n" + "=" * 70)
        logger.info("STOPPING ROBOT")
        logger.info("=" * 70)
        
        logger.info("Setting speed=0...")
        speed_sender.send("0")
        time.sleep(0.1)  # Reduced from 0.2s
        
        logger.info("Disabling engine (KL=0)...")
        klem_sender.send("0")
        time.sleep(0.15)  # Reduced from 0.3s
        
        # Summary
        logger.info("\n" + "=" * 70)
        logger.info("TEST SUMMARY")
        logger.info("=" * 70)
        
        if stop_line_detected:
            logger.info(f"✓ Stop line detected successfully!")
            logger.info(f"  Detection time: {detection_time:.2f}s")
            logger.info(f"  Distance traveled: ~{target_speed * detection_time / 100:.1f}cm")
            logger.info(f"  Total loop iterations: {loop_count}")
            logger.info(f"  Average loop frequency: {loop_count / detection_time:.1f} Hz")
            logger.info(f"  (Approximate, based on speed * time)")
        else:
            logger.error("✗ Stop line NOT detected (timeout after 60s)")
            logger.info("  Check:")
            logger.info("  1. Is IR sensor properly connected to NUCLEO?")
            logger.info("  2. Is StopLine detection working in image processing?")
            logger.info("  3. Is the stop line visible to the sensor?")
        
        logger.info("✓ Line follower test complete!")
        
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        logger.info("\nShutting down...")
        
        # Make sure motor is stopped and IR sensor disabled
        try:
            toggle_stopline_sender.send(0)  # Disable IR sensor
            time.sleep(0.05)
            speed_sender.send("0")          # Stop motor
            time.sleep(0.05)
            klem_sender.send("0")           # Disable engine
        except:
            pass
        
        time.sleep(0.15)  # Reduced from 0.2s
        
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
    print("BFMC LINE FOLLOWER TEST - REAL HARDWARE")
    print("=" * 70)
    print("\nThis test will:")
    print("  1. Enable the motor")
    print("  2. Set constant forward speed (20 cm/s)")
    print("  3. Monitor IR sensor for stop line detection")
    print("  4. Stop when stop line is detected")
    print("  5. Report test results")
    print("\n⚠️  Make sure:")
    print("  - Robot is in a safe, open space")
    print("  - Stop line is placed on the ground where robot will drive")
    print("  - IR sensor is properly calibrated")
    print("=" * 70)
    
    input("\nPress ENTER to start the test...")
    
    main()
