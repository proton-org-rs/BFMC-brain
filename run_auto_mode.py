#!/usr/bin/env python3
# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
#
# Script for running the car in AUTO mode with proper cleanup on exit.
#
# Usage:
#   python run_auto_mode.py
#
# The script will:
#   1. Initialize all required processes (camera, serial handler, autonomous driving)
#   2. Switch to AUTO mode
#   3. On exit (Ctrl+C), switch to STOP mode and gracefully shutdown all threads
#

import sys
import os
import time
import signal
import atexit

# Add src to path
sys.path.append(".")

import psutil
from multiprocessing import Queue, Event

# ===================================== IMPORTS ==================================

from src.utils.bigPrintMessages import BigPrint
from src.gateway.processGateway import processGateway as ProcessGateway
from src.hardware.camera.processCamera import processCamera as ProcessCamera
from src.hardware.serialhandler.processSerialHandler import processSerialHandler as ProcessSerialHandler
from src.AutonomousDriving.processAutonomousDriving import processAutonomousDriving as ProcessAutonomousDriving
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import StateChange, Klem, SpeedMotor, Brake
from src.statemachine.stateMachine import StateMachine
from src.statemachine.systemMode import SystemMode

import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger()

# ===================================== GLOBALS ==================================

all_processes = []
queue_list = None
gateway_process = None
is_shutting_down = False


# ===================================== HELPERS ==================================

def print_banner(text, char="="):
    """Print a banner with the given text."""
    width = 60
    print(f"\n{char * width}")
    print(f"{text.center(width)}")
    print(f"{char * width}\n")


def shutdown_process(process, name="process", timeout=3):
    """Helper function to gracefully shutdown a process."""
    if process is None:
        return
        
    print(f"  → Stopping {name}...")
    
    # Call stop() if available
    if hasattr(process, "stop") and callable(getattr(process, "stop")):
        try:
            process.stop()
        except Exception as e:
            print(f"    Error calling stop() on {name}: {e}")
    
    # Wait for process to finish
    process.join(timeout)
    if process.is_alive():
        print(f"    {name} won't stop, terminating...")
        process.terminate()
        process.join(timeout)
        if process.is_alive():
            print(f"    {name} still alive, killing...")
            process.kill()
    
    print(f"  ✓ {name} stopped")


def send_stop_commands():
    """Send emergency stop commands to the car."""
    global queue_list
    
    if queue_list is None:
        return
        
    try:
        print("  → Sending stop commands...")
        
        # Send brake command
        brake_sender = messageHandlerSender(queue_list, Brake)
        brake_sender.send("0")
        
        # Send zero speed
        speed_sender = messageHandlerSender(queue_list, SpeedMotor)
        speed_sender.send("0")
        
        # Disable engine (KL=0)
        klem_sender = messageHandlerSender(queue_list, Klem)
        klem_sender.send("0")
        
        time.sleep(0.2)  # Give time for commands to be processed
        print("  ✓ Stop commands sent")
        
    except Exception as e:
        print(f"  ✗ Error sending stop commands: {e}")


def switch_to_stop_mode():
    """Switch the system to STOP mode."""
    try:
        print("  → Switching to STOP mode...")
        
        # Send state change directly via queue
        state_sender = messageHandlerSender(queue_list, StateChange)
        state_sender.send("STOP")
        
        time.sleep(0.3)
        print("  ✓ STOP mode activated")
        
    except Exception as e:
        print(f"  ✗ Error switching to STOP mode: {e}")


def cleanup():
    """Cleanup function called on exit."""
    global is_shutting_down, all_processes, gateway_process
    
    if is_shutting_down:
        return
    is_shutting_down = True
    
    print_banner("SHUTTING DOWN", "!")
    
    # 1. Send stop commands to the car
    send_stop_commands()
    
    # 2. Switch to STOP mode
    switch_to_stop_mode()
    
    # 3. Wait a bit for mode change to propagate
    time.sleep(0.5)
    
    # 4. Shutdown all processes in reverse order
    print("\nStopping all processes...")
    
    for proc_name, proc in reversed(all_processes):
        shutdown_process(proc, proc_name)
    
    # 5. Shutdown gateway last
    if gateway_process is not None:
        shutdown_process(gateway_process, "Gateway")
    
    # 6. Cleanup StateMachine
    try:
        StateMachine.cleanup()
    except:
        pass
    
    print_banner("SHUTDOWN COMPLETE", "=")


def signal_handler(signum, frame):
    """Handle interrupt signals."""
    print(f"\n\nReceived signal {signum}, initiating shutdown...")
    cleanup()
    sys.exit(0)


# ===================================== MAIN ==================================

def main():
    global queue_list, gateway_process, all_processes
    
    # Pin to all available CPU cores
    available_cores = list(range(psutil.cpu_count()))
    psutil.Process(os.getpid()).cpu_affinity(available_cores)
    
    # Register cleanup handlers
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print_banner("BFMC AUTO MODE LAUNCHER", "=")
    
    # ===================================== SETUP QUEUES ==================================
    
    print("[1/6] Setting up message queues...")
    queue_list = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
        "Log": Queue(),
    }
    print("  ✓ Queues ready")
    
    # ===================================== INITIALIZE STATE MACHINE ==================================
    
    print("[2/6] Initializing state machine...")
    StateMachine.initialize_shared_state(queue_list)
    print("  ✓ State machine ready")
    
    # ===================================== START GATEWAY ==================================
    
    print("[3/6] Starting gateway process...")
    gateway_process = ProcessGateway(queue_list, logger)
    gateway_process.start()
    print("  ✓ Gateway started")
    
    # ===================================== START PROCESSES ==================================
    
    print("[4/6] Starting required processes...")
    
    # Camera process (required for lane detection frames)
    camera_ready = Event()
    camera_process = ProcessCamera(queue_list, logger, camera_ready, debugging=False)
    camera_process.daemon = True
    camera_process.start()
    all_processes.append(("Camera", camera_process))
    print("  ✓ Camera process started")
    
    # Serial handler (required for motor control)
    serial_ready = Event()
    dashboard_ready = Event()  # Dummy event since we don't use dashboard
    serial_process = ProcessSerialHandler(queue_list, logger, serial_ready, dashboard_ready, debugging=False)
    serial_process.daemon = True
    serial_process.start()
    all_processes.append(("SerialHandler", serial_process))
    print("  ✓ Serial handler started")
    
    # Wait for camera and serial to be ready
    print("  → Waiting for processes to initialize...")
    camera_ready.wait(timeout=10)
    serial_ready.wait(timeout=10)
    print("  ✓ All processes ready")
    
    # ===================================== SWITCH TO AUTO MODE ==================================
    
    print("[5/6] Switching to AUTO mode...")
    
    # Send state change to AUTO
    state_sender = messageHandlerSender(queue_list, StateChange)
    state_sender.send("AUTO")
    time.sleep(0.5)
    
    print("  ✓ AUTO mode activated")
    
    # ===================================== START AUTONOMOUS DRIVING ==================================
    
    print("[6/6] Starting autonomous driving process...")
    
    auto_ready = Event()
    auto_process = ProcessAutonomousDriving(queue_list, logger, auto_ready, debugging=False)
    auto_process.daemon = True
    auto_process.start()
    all_processes.append(("AutonomousDriving", auto_process))
    
    # Wait for autonomous driving to initialize
    auto_ready.wait(timeout=10)
    print("  ✓ Autonomous driving started")
    
    # ===================================== RUNNING ==================================
    
    print_banner("AUTO MODE ACTIVE", "*")
    print("The car is now driving autonomously!")
    print("Press Ctrl+C to stop and return to STOP mode.\n")
    
    # Keep the script running
    try:
        while True:
            time.sleep(1)
            
            # Optional: print status every 10 seconds
            # You could add monitoring here
            
    except KeyboardInterrupt:
        pass  # Will be handled by signal handler


if __name__ == "__main__":
    main()
