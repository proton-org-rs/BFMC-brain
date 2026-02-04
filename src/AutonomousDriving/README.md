# Autonomous Driving in LEGACY Mode

## Overview

This module implements autonomous driving functionality for LEGACY mode. The robot drives forward at a constant speed, monitors IR sensors for stop line detection, and performs stop-wait-resume cycles when lines are detected.

## Behavior

The autonomous driving system operates with the following logic:

1. **Normal Driving State**
   - Robot drives forward at constant speed (default: 350 cm/s)
   - IR sensor continuously monitors for stop lines
   - Steering set to straight (0°)

2. **Stop Line Detection**
   - When IR sensor detects a stop line
   - Robot immediately brakes and stops
   - Enters "paused at line" state

3. **Pause Duration**
   - Robot remains stopped for 3 seconds (configurable)
   - This allows time for traffic/pedestrians/etc.

4. **Resume and Ignore Period**
   - After 3 seconds, robot resumes driving at the same speed
   - For the next 5 seconds (configurable), stop line detection is **ignored**
   - This prevents multiple stops when crossing multiple parallel lines
   - After 5 seconds, returns to normal driving state

## State Machine

```
STOPPED → DRIVING → PAUSED_AT_LINE → IGNORING_LINE → DRIVING
   ↑                     ↓ (line detected)        ↓ (5s elapsed)
   └─────────────────────┴──────────────────────────┘
```

### States:
- **STOPPED**: Initial state, engine off
- **DRIVING**: Normal forward motion, monitoring for stop lines
- **PAUSED_AT_LINE**: Stopped at stop line for 3 seconds
- **IGNORING_LINE**: Driving but ignoring stop line detections for 5 seconds

## Configuration

Edit `threadAutonomousDriving.py` to modify:

```python
self.target_speed = 350  # cm/s - forward driving speed
self.stop_duration = 3.0  # seconds - how long to stop at line
self.ignore_duration = 5.0  # seconds - how long to ignore lines after resuming
```

## Files

- **`processAutonomousDriving.py`**: Process wrapper for autonomous driving
- **`threads/threadAutonomousDriving.py`**: Main thread implementing autonomous logic
- Updated **`main.py`**: Integration with main process manager
- Updated **`systemMode.py`**: Added autonomous_driving config to LEGACY mode

## Activation

Autonomous driving is **automatically activated** when:
- System is switched to **LEGACY mode** via dashboard

The process will:
1. Enable the engine (KL=30)
2. Enable IR sensor stop line detection
3. Start driving forward
4. Begin monitoring for stop lines

## Deactivation

Autonomous driving is **automatically deactivated** when:
- System is switched from LEGACY mode to any other mode (MANUAL, AUTO, STOP, etc.)

The process will:
1. Stop the robot (speed=0)
2. Disable the engine (KL=0)
3. Disable IR sensor

## Integration with System

### System Mode Configuration

The LEGACY mode now includes the autonomous_driving process:

```python
LEGACY = {
    "mode": "legacy",
    "autonomous_driving": {
        "process": {
            "enabled": True,  # Auto-start when entering LEGACY mode
        }
    },
    # ... other processes
}
```

### Process Lifecycle

The autonomous driving process is managed by `main.py` along with other system processes:
- Starts when LEGACY mode is activated
- Stops when switching to another mode
- Uses the same message queue system as other processes

## Dependencies

- **IR Sensor (NUCLEO)**: Must be connected and working
- **Serial Handler**: For communication with NUCLEO
- **Message Queue System**: For inter-process communication

## Messages Used

- **Klem**: Engine enable/disable (KL=30/0)
- **SpeedMotor**: Set driving speed
- **SteerMotor**: Set steering angle
- **Brake**: Emergency brake
- **ToggleStopLine**: Enable/disable IR sensor (1/0)
- **StopLine** (received): Stop line detection status from NUCLEO

## Testing

To test the autonomous driving without full system:

```python
# In src/Brain/src/data/AutonomousDriving/
python processAutonomousDriving.py
```

For full system testing with hardware:
1. Place robot in open space
2. Place stop line markers on ground
3. Switch dashboard to LEGACY mode
4. Robot should drive, stop at lines, wait 3s, continue

## Safety Notes

⚠️ **Important Safety Considerations:**

1. **Open Space Required**: Test only in safe, open areas
2. **Stop Line Visibility**: Ensure stop lines are visible to IR sensor
3. **IR Sensor Calibration**: Verify sensor is properly calibrated
4. **Emergency Stop**: Dashboard STOP mode or MANUAL mode for immediate control
5. **Speed Limits**: Default speed is 350 cm/s (~3.5 m/s or 12.6 km/h)

## Troubleshooting

### Robot doesn't start driving
- Check if engine is enabled (KL=30)
- Verify serial connection to NUCLEO
- Check if process is actually running (logs)

### Robot doesn't stop at lines
- Verify IR sensor is enabled
- Check IR sensor calibration
- Ensure stop line is visible/detectable
- Check logs for StopLine messages

### Robot stops multiple times at same line
- Increase `ignore_duration` (default 5s)
- Check if multiple lines are too close together

### Robot doesn't resume after stopping
- Check logs for state transitions
- Verify `stop_duration` hasn't been set too high
- Check if speed command is being sent

## Logs

Enable debugging to see detailed logs:

```python
processAutonomousDriving = processAutonomousDriving(
    queueList, 
    logging, 
    autonomous_driving_ready, 
    debugging=True  # Enable debug logs
)
```

Log messages include:
- `[AutonomousDriving] Starting autonomous driving sequence...`
- `[AutonomousDriving] STOP LINE DETECTED!`
- `[AutonomousDriving] Stopping for 3 seconds...`
- `[AutonomousDriving] Resuming driving...`
- `[AutonomousDriving] Ignoring stop lines for 5s`
- `[AutonomousDriving] Stop line detection re-enabled`

## Future Improvements

Possible enhancements:
- Variable speed based on conditions
- Multiple stop line patterns recognition
- Integration with traffic light detection
- Obstacle avoidance
- Lane keeping assistance
- Speed adjustment on curves
