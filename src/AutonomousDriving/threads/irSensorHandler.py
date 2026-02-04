# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

"""
IR Sensor Handler Module for Stop Line Detection

This module provides IR sensor handling for stop line detection:
- Subscribes to StopLine messages from NUCLEO
- Implements state machine for stop line handling
- Provides stop/resume control based on IR sensor input

State Machine:
    DRIVING -> STOPPED (when IR detects line)
    STOPPED -> IGNORING (after stop_duration)
    IGNORING -> DRIVING (after ignore_duration)

Usage:
    ir_handler = IRSensorHandler(queueList, logger)
    ir_handler.start()
    
    # In main loop:
    should_drive = ir_handler.update()
    if should_drive:
        # Continue driving
    else:
        # Stop the car
    
    # On shutdown:
    ir_handler.stop()
"""

import time
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import StopLine


class IRSensorHandler:
    """
    IR Sensor Handler for stop line detection.
    
    This class handles:
    - Subscribing to IR sensor stop line messages
    - State machine for stop line behavior
    - Timing for stop duration and ignore period
    """
    
    # State constants
    STATE_DRIVING = "driving"
    STATE_STOPPED = "stopped"
    STATE_IGNORING = "ignoring"
    
    def __init__(self, queueList, logger, stop_duration=3.0, ignore_duration=5.0, debugging=False):
        """
        Initialize the IR sensor handler.
        
        Args:
            queueList: Dictionary of message queues
            logger: Logger object for debugging output
            stop_duration: How long to stop when line detected (seconds)
            ignore_duration: How long to ignore detections after resuming (seconds)
            debugging: Enable debug logging
        """
        self.queueList = queueList
        self.logger = logger
        self.debugging = debugging
        
        # Configuration
        self.stop_duration = stop_duration
        self.ignore_duration = ignore_duration
        
        # State machine
        self.current_state = self.STATE_DRIVING
        self.state_start_time = None
        
        # Message subscriber
        self.stop_line_subscriber = None
        self.last_stop_line_value = None
        
        # Running flag
        self.is_running = False
        
        # Callbacks
        self.on_stop_callback = None
        self.on_resume_callback = None
        
        self.logger.info("[IRSensorHandler] Initialized")

    def start(self):
        """Start the IR sensor handler."""
        if self.is_running:
            return
            
        self.logger.info("[IRSensorHandler] Starting...")
        
        # Initialize subscriber
        self.stop_line_subscriber = messageHandlerSubscriber(
            self.queueList,
            StopLine,
            "lastOnly",
            True
        )
        
        # Reset state
        self.current_state = self.STATE_DRIVING
        self.state_start_time = time.time()
        self.last_stop_line_value = None
        self.is_running = True
        
        self.logger.info("[IRSensorHandler] ✓ Started - monitoring for stop lines")

    def stop(self):
        """Stop the IR sensor handler."""
        if not self.is_running:
            return
            
        self.logger.info("[IRSensorHandler] Stopping...")
        self.is_running = False
        self.current_state = self.STATE_DRIVING
        self.logger.info("[IRSensorHandler] ✓ Stopped")

    def set_callbacks(self, on_stop=None, on_resume=None):
        """
        Set callback functions for state changes.
        
        Args:
            on_stop: Function called when stopping at line
            on_resume: Function called when resuming after stop
        """
        self.on_stop_callback = on_stop
        self.on_resume_callback = on_resume

    def _check_ir_sensor(self):
        """
        Check if IR sensor detected a stop line.
        
        Returns:
            bool: True if new stop line detected, False otherwise
        """
        if self.stop_line_subscriber is None:
            return False
            
        stop_line_data = self.stop_line_subscriber.receive()
        
        if stop_line_data is None:
            return False
            
        # Check if this is a new detection
        if stop_line_data == self.last_stop_line_value:
            return False
            
        self.last_stop_line_value = stop_line_data
        
        # Parse the detection value
        is_detected = str(stop_line_data).strip() == "1" or stop_line_data == True
        
        if is_detected and self.debugging:
            self.logger.info("[IRSensorHandler] IR sensor triggered!")
            
        return is_detected

    def update(self):
        """
        Update the IR sensor handler state machine.
        
        This method should be called in the main control loop.
        
        Returns:
            bool: True if car should continue driving, False if stopped
        """
        if not self.is_running:
            return True
            
        current_time = time.time()
        
        # STATE: DRIVING - Normal driving, watching for stop lines
        if self.current_state == self.STATE_DRIVING:
            if self._check_ir_sensor():
                self.logger.info("[IRSensorHandler] 🛑 STOP LINE DETECTED!")
                self.logger.info(f"[IRSensorHandler] Stopping for {self.stop_duration} seconds...")
                
                self.current_state = self.STATE_STOPPED
                self.state_start_time = current_time
                
                # Call stop callback
                if self.on_stop_callback:
                    self.on_stop_callback()
                    
                return False  # Should stop
        
        # STATE: STOPPED - Waiting at stop line
        elif self.current_state == self.STATE_STOPPED:
            elapsed = current_time - self.state_start_time
            
            if elapsed >= self.stop_duration:
                self.logger.info("[IRSensorHandler] ✓ Resuming driving...")
                self.logger.info(f"[IRSensorHandler] Ignoring stop lines for {self.ignore_duration}s")
                
                self.current_state = self.STATE_IGNORING
                self.state_start_time = current_time
                
                # Call resume callback
                if self.on_resume_callback:
                    self.on_resume_callback()
                    
                return True  # Can drive again
            else:
                return False  # Still stopped
        
        # STATE: IGNORING - Driving but ignoring stop line detections
        elif self.current_state == self.STATE_IGNORING:
            elapsed = current_time - self.state_start_time
            
            if elapsed >= self.ignore_duration:
                self.logger.info("[IRSensorHandler] ✓ Stop line detection re-enabled")
                self.current_state = self.STATE_DRIVING
                self.state_start_time = current_time
            
            # Clear any pending detections during ignore period
            self._check_ir_sensor()
            
            return True  # Continue driving
        
        return True  # Default: continue driving

    def get_state(self):
        """
        Get the current state of the handler.
        
        Returns:
            str: Current state (STATE_DRIVING, STATE_STOPPED, or STATE_IGNORING)
        """
        return self.current_state

    def is_stopped(self):
        """
        Check if currently stopped at a line.
        
        Returns:
            bool: True if stopped, False otherwise
        """
        return self.current_state == self.STATE_STOPPED

    def get_remaining_stop_time(self):
        """
        Get remaining stop time if stopped.
        
        Returns:
            float: Remaining seconds, or 0 if not stopped
        """
        if self.current_state != self.STATE_STOPPED:
            return 0
            
        elapsed = time.time() - self.state_start_time
        remaining = self.stop_duration - elapsed
        return max(0, remaining)

    def force_resume(self):
        """Force resume driving (skip wait)."""
        if self.current_state == self.STATE_STOPPED:
            self.logger.info("[IRSensorHandler] Force resuming...")
            self.current_state = self.STATE_IGNORING
            self.state_start_time = time.time()
            
            if self.on_resume_callback:
                self.on_resume_callback()
