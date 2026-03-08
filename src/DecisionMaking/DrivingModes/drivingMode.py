"""
Driving Mode Definitions for Decision Making

Defines all possible driving modes (states) that the vehicle can be in
during autonomous operation. The Decision Making component transitions
between these modes based on detected objects, signs, and path planning.

Each mode dictates how the AutonomousDriving component should behave:
- Speed adjustments
- Special maneuver logic
- Alert behaviors (lights, signals)
"""

from enum import Enum, auto


class DrivingMode(Enum):
    """
    Enumeration of all possible driving modes.
    
    The Decision Making component sets the active mode based on:
    1. Detected traffic signs / objects (camera, sensors)
    2. Path Planning context (upcoming road type from the map)
    3. External signals (semaphores, traffic communication)
    """

    # ─── Normal driving ───────────────────────────────────────────────
    LANE_FOLLOWING = auto()
    """Default mode: follow lane lines at cruise speed."""

    # ─── Intersection handling ────────────────────────────────────────
    CROSSING = auto()
    """Approaching or navigating an intersection (priority/non-priority)."""

    ROUNDABOUT = auto()
    """Approaching or navigating a roundabout."""

    # ─── Pedestrian / obstacle ────────────────────────────────────────
    PEDESTRIAN_CROSSING = auto()
    """Approaching a pedestrian crossing — slow down, check for pedestrians."""

    OBJECT_ON_ROAD = auto()
    """Object detected on the road — stop or overtake."""

    # ─── Special road sections ────────────────────────────────────────
    TUNNEL = auto()
    """Entering/inside a tunnel — turn on headlights, adjust speed."""

    HIGHWAY = auto()
    """Driving on a highway section — higher speed, lane keeping."""

    ONE_WAY = auto()
    """Driving on a one-way road section."""

    PARKING = auto()
    """Entering a parking zone — slow speed, look for parking spot."""

    # ─── Traffic signals ──────────────────────────────────────────────
    TRAFFIC_LIGHT_RED = auto()
    """Red traffic light detected — stop and wait."""

    TRAFFIC_LIGHT_YELLOW = auto()
    """Yellow traffic light — prepare to stop."""

    TRAFFIC_LIGHT_GREEN = auto()
    """Green traffic light — proceed."""

    # ─── Emergency / special ──────────────────────────────────────────
    STOP = auto()
    """Stop sign detected or emergency stop required."""

    NO_ENTRY = auto()
    """No-entry sign detected — cannot proceed, reroute."""

    RAMP = auto()
    """Ramp / slope section — adjust speed for incline."""


class DrivingModeContext:
    """
    Holds the current driving mode and its metadata.
    
    This class encapsulates the active driving mode along with
    contextual information such as when the mode was activated,
    how long it should remain active, and related parameters.
    """

    def __init__(self):
        self._current_mode = DrivingMode.LANE_FOLLOWING
        self._previous_mode = None
        self._mode_data = {}
        self._mode_start_time = None

    @property
    def current_mode(self) -> DrivingMode:
        return self._current_mode

    @property
    def previous_mode(self) -> DrivingMode:
        return self._previous_mode

    @property
    def mode_data(self) -> dict:
        """Extra data associated with the current mode (e.g., which direction at crossing)."""
        return self._mode_data

    def set_mode(self, new_mode: DrivingMode, data: dict = None):
        """
        Transition to a new driving mode.
        
        Args:
            new_mode: The new DrivingMode to activate.
            data: Optional dict with mode-specific parameters.
                  Example for CROSSING: {"direction": "left", "priority": False}
                  Example for PARKING: {"spot_id": 2, "side": "right"}
        
        Returns:
            bool: True if mode actually changed, False if already in that mode.
        """
        import time

        if new_mode == self._current_mode:
            return False

        self._previous_mode = self._current_mode
        self._current_mode = new_mode
        self._mode_data = data if data is not None else {}
        self._mode_start_time = time.time()
        return True

    def revert_to_previous(self) -> bool:
        """Revert to the previous driving mode."""
        if self._previous_mode is not None:
            return self.set_mode(self._previous_mode)
        return False

    def get_mode_duration(self) -> float:
        """Get how many seconds the current mode has been active."""
        import time
        if self._mode_start_time is None:
            return 0.0
        return time.time() - self._mode_start_time

    def is_mode(self, mode: DrivingMode) -> bool:
        """Check if the current mode matches the given mode."""
        return self._current_mode == mode

    def __repr__(self):
        return (
            f"DrivingModeContext(current={self._current_mode.name}, "
            f"previous={self._previous_mode.name if self._previous_mode else 'None'})"
        )
