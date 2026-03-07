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
Autonomous Driving Thread

This is the main thread for autonomous driving in AUTO mode.
It coordinates:
- LaneDetection: Camera-based lane keeping
- IRSensorHandler: Stop line detection via IR sensor

The car drives autonomously following lane lines. When the IR sensor
detects a stop line, the car stops for 3 seconds then continues.
"""

import time
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import (
    Klem, SpeedMotor, SteerMotor, Brake
)

# Import helper modules
from src.AutonomousDriving.threads.laneDetection import LaneDetection
from src.AutonomousDriving.threads.irSensorHandler import IRSensorHandler


class threadAutonomousDriving(ThreadWithStop):
    """
    Main thread for autonomous driving in AUTO mode.
    
    This thread coordinates lane detection (camera) and IR sensor handling
    to provide autonomous lane-following with stop line detection.
    
    Features:
    - Vision-based lane keeping using PD controller
    - IR sensor-based stop line detection
    - Automatic stop for 3 seconds at stop lines
    - State machine for robust stop line handling
    
    Args:
        queueList: Dictionary of message queues
        logger: Logger object for debugging
        debugging: Enable debug logging
    """

    def __init__(self, queueList, logger, debugging=False):
        super(threadAutonomousDriving, self).__init__(pause=0.01)  # 10ms loop
        self.queueList = queueList
        self.logger = logger
        self.debugging = debugging
        
        # Configuration
        self.target_speed = 150  # Default speed (can be changed)
        
        # Components (initialized in start)
        self.lane_detection = None
        self.ir_sensor = None
        
        # State
        self.initialized = False
        self.is_driving = False
        self.last_steering = 0
        
        # Message handlers
        self._init_message_handlers()
        
        self.logger.info("[AutonomousDriving] Thread initialized")

    def _init_message_handlers(self):
        """Initialize message senders."""
        self.klem_sender = messageHandlerSender(self.queueList, Klem)
        self.speed_sender = messageHandlerSender(self.queueList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queueList, SteerMotor)
        self.brake_sender = messageHandlerSender(self.queueList, Brake)

    # ======================================= MOTOR CONTROL ==========================================
    def _stop_car(self):
        """Stop the car (called by IR sensor handler)."""
        self.logger.info("[AutonomousDriving] 🛑 Stopping car...")
        self.brake_sender.send("0")
        self.speed_sender.send("0")
        self.is_driving = False

    def _resume_car(self):
        """Resume driving (called by IR sensor handler)."""
        self.logger.info(f"[AutonomousDriving] ▶ Resuming at speed {self.target_speed}")
        self.speed_sender.send(str(self.target_speed))
        self.is_driving = True

    def _send_steering(self, angle):
        """Send steering command if changed."""
        if angle != self.last_steering:
            self.steer_sender.send(str(angle))
            self.last_steering = angle

    # ======================================= START ==========================================
    def start_autonomous_driving(self):
        """Initialize and start autonomous driving."""
        self.logger.info("[AutonomousDriving] ═══════════════════════════════════════")
        self.logger.info("[AutonomousDriving] Starting Autonomous Driving System...")
        self.logger.info("[AutonomousDriving] ═══════════════════════════════════════")
        
        # 1. Initialize Lane Detection (Camera)
        self.logger.info("[AutonomousDriving] [1/4] Initializing Lane Detection...")
        self.lane_detection = LaneDetection(self.queueList, self.logger, self.debugging)
        if not self.lane_detection.start():
            self.logger.error("[AutonomousDriving] ✗ Failed to start lane detection!")
            return False
        self.logger.info("[AutonomousDriving] ✓ Lane Detection ready")
        
        # 2. Initialize IR Sensor Handler
        self.logger.info("[AutonomousDriving] [2/4] Initializing IR Sensor Handler...")
        self.ir_sensor = IRSensorHandler(
            self.queueList, 
            self.logger,
            stop_duration=3.0,
            ignore_duration=5.0,
            debugging=self.debugging
        )
        self.ir_sensor.set_callbacks(
            on_stop=self._stop_car,
            on_resume=self._resume_car
        )
        self.ir_sensor.start()
        self.logger.info("[AutonomousDriving] ✓ IR Sensor Handler ready")
        
        # 3. Enable engine
        self.logger.info("[AutonomousDriving] [3/4] Enabling engine (KL=30)...")
        self.klem_sender.send("30")
        time.sleep(0.1)
        self.logger.info("[AutonomousDriving] ✓ Engine enabled")
        
        # 4. Start driving
        self.logger.info("[AutonomousDriving] [4/4] Starting to drive...")
        self.steer_sender.send("0")
        time.sleep(0.05)
        self.speed_sender.send(str(self.target_speed))
        self.is_driving = True
        
        self.logger.info("[AutonomousDriving] ═══════════════════════════════════════")
        self.logger.info("[AutonomousDriving] ✓ AUTONOMOUS DRIVING ACTIVE!")
        self.logger.info(f"[AutonomousDriving]   Speed: {self.target_speed}")
        self.logger.info("[AutonomousDriving]   Lane Detection: ACTIVE")
        self.logger.info("[AutonomousDriving]   IR Sensor: MONITORING")
        self.logger.info("[AutonomousDriving] ═══════════════════════════════════════")
        
        return True

    # ======================================= STOP ==========================================
    def stop_autonomous_driving(self):
        """Stop autonomous driving and cleanup."""
        self.logger.info("[AutonomousDriving] Stopping autonomous driving...")
        
        # 1. Stop camera/lane detection FIRST to prevent new steering commands
        if self.lane_detection is not None:
            self.logger.info("[AutonomousDriving] Stopping lane detection...")
            self.lane_detection.stop()
            self.lane_detection = None
            self.logger.info("[AutonomousDriving] ✓ Lane detection stopped")

        # 2. Stop IR sensor handler
        if self.ir_sensor is not None:
            self.ir_sensor.stop()
            self.ir_sensor = None

        # 3. Stop the car - brake, zero speed and steer
        self.brake_sender.send("0")
        self.speed_sender.send("0")
        self.steer_sender.send("0")
        self.is_driving = False

        # 4. Give serial handler time to flush commands to NUCLEO
        time.sleep(0.05)
        
        # NOTE: Do NOT send KL=0 here. Engine state (KL) is managed by
        # main.py during mode transitions to avoid disabling the engine
        # for subsequent modes (MANUAL, LEGACY, etc.)
        
        self.logger.info("[AutonomousDriving] ✓ Autonomous driving stopped")

    # ======================================= MAIN LOOP ==========================================
    def thread_work(self):
        """
        Main thread work - called repeatedly.
        
        This method:
        1. Initializes on first call
        2. Updates IR sensor state machine
        3. Gets steering from lane detection (if driving)
        4. Sends steering command
        """
        # Initialize on first call
        if not self.initialized:
            if self.start_autonomous_driving():
                self.initialized = True
            else:
                self.logger.error("[AutonomousDriving] Initialization failed!")
                time.sleep(1)
            return
        
        try:
            # 1. Update IR sensor state machine
            #    This returns False if we should stop (at stop line)
            should_drive = self.ir_sensor.update()
            
            # 2. If driving, get steering from lane detection
            if should_drive and self.is_driving:
                steering = self.lane_detection.get_steering_angle()
                if steering is not None:
                    self._send_steering(steering)
                    
        except Exception as e:
            self.logger.error(f"[AutonomousDriving] Error in main loop: {e}", exc_info=True)

    def stop(self):
        """Override stop to cleanup before stopping thread."""
        if self.initialized:
            self.stop_autonomous_driving()
            self.initialized = False
            # removed broken self.kl line
        
        super(threadAutonomousDriving, self).stop()
        self.logger.info("[AutonomousDriving] Thread stopped")

    # ======================================= PUBLIC API ==========================================
    def set_speed(self, speed):
        """
        Set the target speed.
        
        Args:
            speed: New speed value
        """
        self.target_speed = speed
        if self.is_driving:
            self.speed_sender.send(str(speed))
            self.logger.info(f"[AutonomousDriving] Speed changed to {speed}")
