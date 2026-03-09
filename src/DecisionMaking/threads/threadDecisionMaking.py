"""
Decision Making Thread — Planning & Intelligence Layer

This thread is the BRAIN of the autonomous driving system.
It does NOT control motors or camera directly. Instead:
- Receives DetectionEvent from AutonomousDriving (YOLO labels + stop line)
- Manages the DrivingMode state machine (transitions between modes)
- Sends high-level commands to AutonomousDriving (SpeedCommand, StopCommand)
- Uses PathPlanning for map-aware navigation

Architecture (Master/Slave):

    ┌─────────────────────────────────────────────────────────────┐
    │               DecisionMaking (Planner — this thread)         │
    │                                                              │
    │   DetectionEvent ──→ DrivingMode ──→ SpeedCommand/StopCommand│
    │   (from AD)          State Machine    (to AD)                │
    │                          │                                   │
    │                    PathPlanning                               │
    │                    + GraphMap                                 │
    └──────────────┬──────────────────────────────────┬────────────┘
                   │ subscribe                         │ publish
                   ▲                                   ▼
    ┌──────────────┴──────────────────────────────────┴────────────┐
    │              AutonomousDriving (Executor)                     │
    │                                                              │
    │   Camera ──→ LaneDetection ──→ Steering (local)              │
    │              YOLO ──→ DetectionEvent (publish)                │
    │              SpeedCommand/StopCommand ──→ Motor Control       │
    └──────────────────────────────────────────────────────────────┘

Communication:
    AD → DM:  DetectionEvent  {"detection": "red"|None, "stop_line": True|False}
    DM → AD:  SpeedCommand    "150", "300", etc.
    DM → AD:  StopCommand     "stop" or "resume"
"""

import time
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import (
    CurrentDrivingMode,
    Location,
    DetectionEvent,
    SpeedCommand,
    StopCommand,
)

from src.DecisionMaking.DrivingModes.drivingMode import DrivingMode, DrivingModeContext
from src.DecisionMaking.PathPlanning.pathPlanning import PathPlanning


class threadDecisionMaking(ThreadWithStop):
    """
    Decision Making planner thread.

    Receives detection events from AutonomousDriving and sends
    high-level commands (speed, stop, resume) back to it.
    Manages the DrivingMode state machine and PathPlanning.

    Args:
        queueList:    Dictionary of message queues.
        logger:       Logger for debug/info messages.
        graphml_path: Path to the competition track GraphML file.
        debugging:    Enable verbose debug logging.
    """

    # ─── Speed constants ──────────────────────────────────────────
    SPEED_NORMAL = 150
    SPEED_HIGHWAY = 300
    SPEED_SLOW = 100
    SPEED_STOP = 0

    def __init__(self, queueList, logger, graphml_path, debugging=False):
        super(threadDecisionMaking, self).__init__(pause=0.01)
        self.queueList = queueList
        self.logger = logger
        self.debugging = debugging

        # ─── Driving Mode Context ────────────────────────────────
        self.mode_context = DrivingModeContext()

        # ─── Path Planning ───────────────────────────────────────
        self.path_planning = PathPlanning(graphml_path, logger)

        # ─── State ───────────────────────────────────────────────
        self.initialized = False

        # Detection flags (same semantics as old AD)
        self.toStop = False
        self.waiting = False
        self.stop_start_time = 0

        # Traffic light flags
        self.redLight = False
        self.yellowLight = False
        self.greenLight = False

        # Road type flags
        self.highway = False

        # GPS
        self._current_position = None

        # ─── Message handlers ────────────────────────────────────
        self._init_senders()
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

    def _init_senders(self):
        """Senders: commands to AD + driving mode for dashboard."""
        self.speed_command_sender = messageHandlerSender(self.queueList, SpeedCommand)
        self.stop_command_sender = messageHandlerSender(self.queueList, StopCommand)
        self.driving_mode_sender = messageHandlerSender(self.queueList, CurrentDrivingMode)

    def _init_subscribers(self):
        """Subscribe to detection events from AD and GPS updates."""
        self.detection_subscriber = messageHandlerSubscriber(
            self.queueList, DetectionEvent, "fifo", True
        )
        self.location_subscriber = messageHandlerSubscriber(
            self.queueList, Location, "lastOnly", True
        )

    # ═══════════════════════════ MODE MANAGEMENT ═════════════════════

    def _publish_mode(self, mode: DrivingMode, data: dict = None):
        """Publish current driving mode to message bus."""
        msg = f"DRIVING_MODE:{mode.name}"
        if data:
            params = ",".join(f"{k}={v}" for k, v in data.items())
            msg += f"|{params}"
        self.driving_mode_sender.send(msg)

    def _transition_mode(self, new_mode: DrivingMode, data: dict = None):
        """Attempt a mode transition. Logs and publishes on change."""
        changed = self.mode_context.set_mode(new_mode, data)
        if changed:
            prev = self.mode_context.previous_mode.name if self.mode_context.previous_mode else "None"
            self._log(f"Mode: {prev} -> \033[1;96m{new_mode.name}\033[0m")
            self._publish_mode(new_mode, data)
        return changed

    # ═══════════════════════ THREAD LIFECYCLE ═════════════════════════

    def stop(self):
        """Override stop to cleanup."""
        super(threadDecisionMaking, self).stop()
        self._log("Thread stopped")

    # ═══════════════════════ MAIN LOOP ════════════════════════════════

    def thread_work(self):
        """
        Main work cycle — receives detections from AD and sends commands.

        Flow:
        1. Initialize on first call (no hardware — just logging)
        2. Process all queued DetectionEvents from AD
        3. Handle each detection → transition DrivingMode → send command
        4. Manage stop line timer (2s wait → resume)
        5. Update GPS for path planning
        """
        # ─── First call: just mark as ready ──────────────────────
        if not self.initialized:
            self._log("═══════════════════════════════════════")
            self._log("Decision Making PLANNER active")
            self._log("  Waiting for DetectionEvents from AD...")
            self._log("  PathPlanning: loaded")
            self._log("  DrivingMode: LANE_FOLLOWING")
            self._log("═══════════════════════════════════════")
            self._transition_mode(DrivingMode.LANE_FOLLOWING)
            self.initialized = True
            return

        try:
            self._thread_work_inner()
        except Exception as e:
            self._log(f"CRASH in thread_work: {e}", "warning")
            print(f"\033[1;91m[DecisionMaking] CRASH in thread_work: {e}\033[0m")
            import traceback
            traceback.print_exc()

    def _thread_work_inner(self):
        # ─── Update GPS position for path planning ───────────────
        self._process_location()

        # ─── Process all queued detection events from AD (FIFO) ──
        for _ in range(20):
            msg = self.detection_subscriber.receive()
            if msg is None:
                break

            detection = msg.get("detection")
            stop_line = msg.get("stop_line", False)

            if self.debugging:
                self._log(f"detection={detection}, stop_line={stop_line}", "debug")

            # Handle detection → mode transition → command
            self._handle_detection(detection)

            # Stop line check (armed by "stop" sign detection)
            if stop_line and self.toStop and not self.waiting:
                self._log("STOP LINE — commanding AD to stop for 2s")
                self.stop_command_sender.send("stop")
                self.waiting = True
                self.stop_start_time = time.time()
                self._transition_mode(DrivingMode.STOP, {"source": "stop_line"})

        # ─── Timer for stop line wait ────────────────────────────
        if self.waiting:
            if time.time() - self.stop_start_time >= 2:
                self._log("2 seconds elapsed — commanding AD to resume")
                self.stop_command_sender.send("resume")
                self.waiting = False
                self.toStop = False
                self._transition_mode(DrivingMode.LANE_FOLLOWING)

    # ═══════════════════════ DETECTION HANDLER ════════════════════════

    def _handle_detection(self, detection):
        """
        Handle a YOLO detection label received from AD.

        Maps each detection string to a DrivingMode transition
        and sends the appropriate command (SpeedCommand / StopCommand)
        to AutonomousDriving.
        """
        if detection is None:
            return

        # ─── Priority sign ───────────────────────────────────────
        if detection == "priority":
            pass

        # ─── Crosswalk / Pedestrian crossing ─────────────────────
        elif detection == "crosswalk_blue":
            if self._transition_mode(DrivingMode.PEDESTRIAN_CROSSING):
                self._log("Pedestrian crossing detected")
                # TODO: check for pedestrians with ultrasonic sensor

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

        # ─── Highway enter ───────────────────────────────────────
        elif detection == "highway_enter":
            if not self.highway:
                self._log("Highway entrance — commanding speed 300")
                self.highway = True
                self.speed_command_sender.send(str(self.SPEED_HIGHWAY))
                self._transition_mode(DrivingMode.HIGHWAY)

        # ─── Highway exit ────────────────────────────────────────
        elif detection == "highway_exit":
            if self.highway:
                self._log("Highway exit — commanding speed 150")
                self.highway = False
                self.speed_command_sender.send(str(self.SPEED_NORMAL))
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
                self._log("RED LIGHT — commanding stop")
                self.redLight = True
                self.yellowLight = False
                self.greenLight = False
                self.stop_command_sender.send("stop")
                self._transition_mode(DrivingMode.TRAFFIC_LIGHT_RED)

        elif detection == "yellow":
            if not self.yellowLight:
                self._log("YELLOW LIGHT — commanding stop")
                self.redLight = False
                self.yellowLight = True
                self.greenLight = False
                self.stop_command_sender.send("stop")
                self._transition_mode(DrivingMode.TRAFFIC_LIGHT_YELLOW)

        elif detection == "green":
            if not self.greenLight:
                self._log("GREEN LIGHT — commanding resume")
                self.redLight = False
                self.yellowLight = False
                self.greenLight = True
                self.stop_command_sender.send("resume")
                self._transition_mode(DrivingMode.TRAFFIC_LIGHT_GREEN)

        elif detection == "off":
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
