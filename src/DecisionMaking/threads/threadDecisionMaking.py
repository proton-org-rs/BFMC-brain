"""
Decision Making Thread

This is the main thread for autonomous driving with intelligent decision making.
It replaces/enhances the basic logic from threadAutonomousDriving by adding:
- Structured DrivingMode state machine (instead of scattered bool flags)
- Path Planning integration (graph map awareness)
- Clean mode transition logic with behaviors per mode

It works the same way as threadAutonomousDriving:
1. Uses LaneDetection from AutonomousDriving to get steering + YOLO detections
2. Controls the car directly via SpeedMotor / SteerMotor / Brake / Klem senders
3. Handles stop lines, traffic lights, signs, highway, etc.

Architecture:
    ┌──────────────────────────────────────────────────────────────┐
    │                    threadDecisionMaking                       │
    │                                                              │
    │  ┌────────────┐    ┌───────────────┐    ┌────────────────┐  │
    │  │LaneDetection│───>│  DrivingMode  │───>│ Motor Control  │  │
    │  │ (camera +   │    │  State Machine│    │ (speed, steer, │  │
    │  │  YOLO det.) │    │  + Behaviors  │    │  brake, klem)  │  │
    │  └────────────┘    └───────┬───────┘    └────────────────┘  │
    │                            │                                 │
    │                     ┌──────┴──────┐                          │
    │                     │ PathPlanning │                          │
    │                     │ + GraphMap   │                          │
    │                     └─────────────┘                          │
    └──────────────────────────────────────────────────────────────┘

Detection labels from YOLO (LaneDetection.get_drive_params()):
    "priority", "crosswalk_blue", "stop", "parking",
    "highway_enter", "highway_exit", "no_entry", "roundabout",
    "one_way", "red", "yellow", "green", "off"
"""

import time
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import (
    Klem, SpeedMotor, SteerMotor, Brake,
    CurrentDrivingMode,
    Location,
)

# Reuse LaneDetection from AutonomousDriving (READ-ONLY import, no edits)
from src.AutonomousDriving.threads.laneDetection import LaneDetection

from src.DecisionMaking.DrivingModes.drivingMode import DrivingMode, DrivingModeContext
from src.DecisionMaking.PathPlanning.pathPlanning import PathPlanning


class threadDecisionMaking(ThreadWithStop):
    """
    Main Decision Making thread with real vehicle control.

    Combines:
    - LaneDetection (camera frames -> steering + YOLO sign detection)
    - DrivingMode state machine (clean mode transitions)
    - Motor control (speed / steer / brake / klem senders)
    - PathPlanning (graph map for upcoming features)

    This thread IS the brain of autonomous driving when DecisionMaking
    process is active.

    Args:
        queueList:    Dictionary of message queues (inter-process communication).
        logger:       Logger for debug/info messages.
        graphml_path: Path to the competition track GraphML file.
        debugging:    Enable verbose debug logging.
    """

    # ─── Speed constants ──────────────────────────────────────────
    SPEED_NORMAL = 150
    SPEED_HIGHWAY = 300
    SPEED_SLOW = 100      # for crosswalks, parking, roundabout…
    SPEED_STOP = 0

    def __init__(self, queueList, logger, graphml_path, debugging=False):
        super(threadDecisionMaking, self).__init__(pause=0.01)  # 10ms loop (same as AD)
        self.queueList = queueList
        self.logger = logger
        self.debugging = debugging

        # ─── Driving Mode Context ────────────────────────────────
        self.mode_context = DrivingModeContext()

        # ─── Path Planning ───────────────────────────────────────
        self.path_planning = PathPlanning(graphml_path, logger)

        # ─── LaneDetection (will be created on first thread_work) ─
        self.lane_detection = None

        # ─── Vehicle state ───────────────────────────────────────
        self.target_speed = self.SPEED_NORMAL
        self.initialized = False
        self.is_driving = False
        self.last_steering = 0

        # ─── Detection flags (mirroring AD logic) ────────────────
        self.toStop = False
        self.waiting = False
        self.stop_start_time = 0
        self.stop_line_detected = False

        # Traffic light flags
        self.redLight = False
        self.yellowLight = False
        self.greenLight = False

        # Road type flags
        self.highway = False

        # ─── GPS/Localization ────────────────────────────────────
        self._current_position = None

        # ─── Message handlers ────────────────────────────────────
        self._init_message_handlers()
        self._init_subscribers()

        self._log("Decision Making thread initialized")

    # ═══════════════════════════ LOGGING ═══════════════════════════════

    def _log(self, message: str, level: str = "info"):
        prefix = "\033[1;97m[ DecisionMaking ]\033[0m"
        if level == "info":
            self.logger.info(f"{prefix} {message}")
        elif level == "warning":
            self.logger.warning(f"{prefix} \033[1;93mWARNING\033[0m {message}")
        elif level == "debug" and self.debugging:
            self.logger.debug(f"{prefix} \033[1;90mDEBUG\033[0m {message}")

    # ═══════════════════════════ MESSAGE HANDLERS ═════════════════════

    def _init_message_handlers(self):
        """Initialize motor-control senders (same as threadAutonomousDriving)."""
        self.klem_sender = messageHandlerSender(self.queueList, Klem)
        self.speed_sender = messageHandlerSender(self.queueList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queueList, SteerMotor)
        self.brake_sender = messageHandlerSender(self.queueList, Brake)
        # Driving mode publisher (for dashboard / other listeners)
        self.driving_mode_sender = messageHandlerSender(self.queueList, CurrentDrivingMode)

    def _init_subscribers(self):
        """Subscribe to GPS/localization updates."""
        self.location_subscriber = messageHandlerSubscriber(
            self.queueList, Location, "lastOnly", True
        )

    # ═══════════════════════════ MOTOR CONTROL ════════════════════════

    def _stop_car(self):
        """Stop the car (speed=0)."""
        self._log("Stopping car...")
        self.speed_sender.send("0")
        self.is_driving = False

    def _resume_car(self):
        """Resume driving at target_speed."""
        self._log(f"Resuming at speed {self.target_speed}")
        self.speed_sender.send(str(self.target_speed))
        self.is_driving = True

    def _send_steering(self, angle):
        """Send steering only when changed."""
        if angle != self.last_steering:
            self.steer_sender.send(str(angle))
            self.last_steering = angle

    def _set_speed(self, speed):
        """Change target speed and send immediately if driving."""
        self.target_speed = speed
        if self.is_driving:
            self.speed_sender.send(str(speed))
            self._log(f"Speed changed to {speed}")

    # ═══════════════════════════ DRIVING MODE PUBLISH ═════════════════

    def _publish_mode(self, mode: DrivingMode, data: dict = None):
        """Publish current driving mode to message bus."""
        msg = f"DRIVING_MODE:{mode.name}"
        if data:
            params = ",".join(f"{k}={v}" for k, v in data.items())
            msg += f"|{params}"
        self.driving_mode_sender.send(msg)

    def _transition_mode(self, new_mode: DrivingMode, data: dict = None):
        """
        Attempt a mode transition. Logs and publishes on change.

        Returns True if the mode actually changed.
        """
        changed = self.mode_context.set_mode(new_mode, data)
        if changed:
            prev_name = self.mode_context.previous_mode.name if self.mode_context.previous_mode else "None"
            self._log(f"Mode: {prev_name} -> \033[1;96m{new_mode.name}\033[0m")
            self._publish_mode(new_mode, data)
        return changed

    # ═══════════════════════ INITIALIZATION ═══════════════════════════

    def _start_driving(self) -> bool:
        """Initialize lane detection, engine, and start driving."""
        self._log("═══════════════════════════════════════")
        self._log("Starting Decision Making Driving System...")
        self._log("═══════════════════════════════════════")

        # 1. Initialize Lane Detection (subscribes to camera frames)
        self._log("[1/3] Initializing Lane Detection...")
        self.lane_detection = LaneDetection(self.queueList, self.logger, self.debugging)
        if not self.lane_detection.start():
            self.logger.error("[DecisionMaking] Failed to start lane detection!")
            return False
        self._log("Lane Detection ready")

        # 2. Enable engine (KL=30)
        self._log("[2/3] Enabling engine (KL=30)...")
        self.klem_sender.send("30")
        time.sleep(0.1)
        self._log("Engine enabled")

        # 3. Start driving
        self._log("[3/3] Starting to drive...")
        self.steer_sender.send("0")
        time.sleep(0.05)
        self.is_driving = True
        self._transition_mode(DrivingMode.LANE_FOLLOWING)

        self._log("═══════════════════════════════════════")
        self._log("DECISION MAKING DRIVING ACTIVE!")
        self._log(f"  Speed: {self.target_speed}")
        self._log("  Lane Detection: ACTIVE")
        self._log("  DrivingMode: LANE_FOLLOWING")
        self._log("═══════════════════════════════════════")
        return True

    def _stop_driving(self):
        """Stop lane detection and motors."""
        self._log("Stopping driving...")

        # 1. Stop lane detection
        if self.lane_detection is not None:
            self.lane_detection.stop()
            self.lane_detection = None
            self._log("Lane detection stopped")

        # 2. Stop the car
        self.brake_sender.send("0")
        self.speed_sender.send("0")
        self.steer_sender.send("0")
        self.is_driving = False
        time.sleep(0.05)

        self._log("Driving stopped")

    # ═══════════════════════ THREAD LIFECYCLE ═════════════════════════

    def stop(self):
        """Override stop to cleanup before stopping thread."""
        if self.initialized:
            self._stop_driving()
            self.initialized = False
        super(threadDecisionMaking, self).stop()
        self._log("Thread stopped")

    # ═══════════════════════ MAIN LOOP ════════════════════════════════

    def thread_work(self):
        """
        Main work cycle — mirrors the logic from threadAutonomousDriving
        but routes everything through the DrivingMode state machine.

        Flow:
        1. Initialize on first call
        2. If not waiting at stop line -> get detection + steering from LaneDetection
        3. Handle detection -> transition DrivingMode
        4. Execute behavior for current mode (stop/resume/speed change)
        5. Send steering if driving
        6. If waiting -> check timer, resume when done
        """
        # ─── First call: initialize ──────────────────────────────
        if not self.initialized:
            if self._start_driving():
                self.initialized = True
            else:
                self.logger.error("[DecisionMaking] Initialization failed!")
                time.sleep(1)
            return

        # ─── Update GPS position for path planning ───────────────
        self._process_location()

        # ─── Main driving logic ──────────────────────────────────
        try:
            if not self.waiting:
                # Get steering + detection from LaneDetection (same as AD)
                steer, detection, self.stop_line_detected = self.lane_detection.get_drive_params()

                if self.debugging:
                    self._log(f"detection: {detection}, stop_line: {self.stop_line_detected}", "debug")

                # ─── Handle detection ────────────────────────────
                self._handle_detection(detection)

                # ─── Stop line + toStop check ────────────────────
                if self.stop_line_detected and self.toStop:
                    self._log("STOP LINE detected, stopping for 2 seconds...")
                    self._stop_car()
                    self.waiting = True
                    self.stop_start_time = time.time()
                    self._transition_mode(DrivingMode.STOP, {"source": "stop_line"})

                # ─── Send steering if driving ────────────────────
                if self.is_driving:
                    self._send_steering(steer)
            else:
                # ─── Waiting at stop line ────────────────────────
                if time.time() - self.stop_start_time >= 2:
                    self._log("Waited 2 seconds, resuming driving...")
                    self._resume_car()
                    self.waiting = False
                    self.toStop = False
                    self._transition_mode(DrivingMode.LANE_FOLLOWING)

                # Drain camera frames to keep the queue from backing up
                if self.lane_detection is not None:
                    self.lane_detection.drain_frame()

        except Exception as e:
            self.logger.error(f"[DecisionMaking] Error in main loop: {e}", exc_info=True)

    # ═══════════════════════ DETECTION HANDLER ════════════════════════

    def _handle_detection(self, detection):
        """
        Handle a YOLO detection label from LaneDetection.

        This maps each detection string to a DrivingMode transition
        and executes the corresponding behavior (speed change, stop, etc.).

        Detection labels from the YOLO model:
            "priority"       -> keep driving (LANE_FOLLOWING)
            "crosswalk_blue" -> PEDESTRIAN_CROSSING (slow down, check for pedestrians)
            "stop"           -> arm stop line trigger (toStop flag)
            "parking"        -> PARKING mode
            "highway_enter"  -> HIGHWAY mode (speed up)
            "highway_exit"   -> LANE_FOLLOWING (restore speed)
            "no_entry"       -> NO_ENTRY
            "roundabout"     -> ROUNDABOUT
            "one_way"        -> ONE_WAY
            "red"            -> TRAFFIC_LIGHT_RED (stop car)
            "yellow"         -> TRAFFIC_LIGHT_YELLOW (stop car)
            "green"          -> TRAFFIC_LIGHT_GREEN (resume car)
            "off"            -> no action
            None             -> no detection
        """
        if detection is None:
            return

        # ─── Priority sign ───────────────────────────────────────
        if detection == "priority":
            # Priority road — just keep driving normally
            pass

        # ─── Crosswalk / Pedestrian crossing ─────────────────────
        elif detection == "crosswalk_blue":
            if self._transition_mode(DrivingMode.PEDESTRIAN_CROSSING):
                self._log("Pedestrian crossing detected")
                # TODO: check for pedestrians with ultrasonic sensor
                pass

        # ─── Stop sign ───────────────────────────────────────────
        elif detection == "stop":
            if not self.toStop:
                self.toStop = True
                self._log("Stop sign detected — armed for stop line")

        # ─── Parking ─────────────────────────────────────────────
        elif detection == "parking":
            if self._transition_mode(DrivingMode.PARKING):
                self._log("Parking zone detected")
                # TODO: parking maneuver if needToPark is True
                pass

        # ─── Highway enter ───────────────────────────────────────
        elif detection == "highway_enter":
            if not self.highway:
                self._log("Highway entrance detected — increasing speed!")
                self.highway = True
                self._set_speed(self.SPEED_HIGHWAY)
                self._transition_mode(DrivingMode.HIGHWAY)

        # ─── Highway exit ────────────────────────────────────────
        elif detection == "highway_exit":
            if self.highway:
                self._log("Highway exit detected — restoring speed")
                self.highway = False
                self._set_speed(self.SPEED_NORMAL)
                self._transition_mode(DrivingMode.LANE_FOLLOWING)

        # ─── No entry ────────────────────────────────────────────
        elif detection == "no_entry":
            self._transition_mode(DrivingMode.NO_ENTRY)

        # ─── Roundabout ──────────────────────────────────────────
        elif detection == "roundabout":
            self._transition_mode(DrivingMode.ROUNDABOUT)

        # ─── One way ─────────────────────────────────────────────
        elif detection == "one_way":
            self._transition_mode(DrivingMode.ONE_WAY)

        # ─── Traffic lights ──────────────────────────────────────
        elif detection == "red":
            if not self.redLight:
                self._log("RED LIGHT — stopping")
                self.redLight = True
                self.yellowLight = False
                self.greenLight = False
                self._stop_car()
                self._transition_mode(DrivingMode.TRAFFIC_LIGHT_RED)

        elif detection == "yellow":
            if not self.yellowLight:
                self._log("YELLOW LIGHT — stopping")
                self.redLight = False
                self.yellowLight = True
                self.greenLight = False
                self._stop_car()
                self._transition_mode(DrivingMode.TRAFFIC_LIGHT_YELLOW)

        elif detection == "green":
            if not self.greenLight:
                self._log("GREEN LIGHT — resuming")
                self.redLight = False
                self.yellowLight = False
                self.greenLight = True
                self._resume_car()
                self._transition_mode(DrivingMode.TRAFFIC_LIGHT_GREEN)

        elif detection == "off":
            # Traffic light off — no action
            pass

    # ═══════════════════════ GPS / PATH PLANNING ═════════════════════

    def _process_location(self):
        """Process GPS/localization updates and feed to path planning."""
        message = self.location_subscriber.receive()
        if message is not None:
            try:
                if isinstance(message, dict):
                    x = float(message.get("x", 0))
                    y = float(message.get("y", 0))
                    self._current_position = (x, y)
                    self.path_planning.update_position(x, y)
                    self._log(f"Position: ({x:.2f}, {y:.2f})", "debug")
            except (ValueError, TypeError) as e:
                self._log(f"Invalid position: {e}", "warning")

    # ═══════════════════════ PUBLIC API ════════════════════════════════

    def get_current_mode(self) -> DrivingMode:
        """Get the current driving mode."""
        return self.mode_context.current_mode

    def get_mode_context(self) -> DrivingModeContext:
        """Get the full mode context."""
        return self.mode_context

    def set_route(self, node_ids: list):
        """Set the planned route on the path planner."""
        self.path_planning.set_route(node_ids)
        self._log(f"Route set: {len(node_ids)} waypoints")

    def compute_route(self, start_id: str, end_id: str) -> bool:
        """Compute a route between two nodes."""
        return self.path_planning.compute_route(start_id, end_id)

    def set_speed(self, speed):
        """Public API for changing speed."""
        self._set_speed(speed)
