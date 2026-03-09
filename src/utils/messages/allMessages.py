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

from enum import Enum

####################################### processCamera #######################################
class mainCamera(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 1
    msgType = "str"

class serialCamera(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 2
    msgType = "str"

class Recording(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 3
    msgType = "bool"

class Signal(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 4
    msgType = "str"

class LaneKeeping(Enum):
    Queue = "General"
    Owner = "threadCamera" # here you will send an offset of the car position between the lanes of the road + - from 0 point to dashboard
    msgID = 5
    msgType = "int"

class laneCamera(Enum):
    """
    Lane detection camera stream - 640x480 RGB frame as numpy bytes.
    Used for autonomous driving lane detection without base64 encoding overhead.
    """
    Queue = "General"
    Owner = "threadCamera"
    msgID = 6
    msgType = "dict"  # Raw numpy array bytes for low-latency processing

################################# processCarsAndSemaphores ##################################
class Cars(Enum):
    Queue = "General"
    Owner = "threadCarsAndSemaphores"
    msgID = 1
    msgType = "dict"

class Semaphores(Enum):
    Queue = "General"
    Owner = "threadCarsAndSemaphores"
    msgID = 2
    msgType = "dict"

################################# From Dashboard ##################################
class SpeedMotor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 1
    msgType = "str"

class SteerMotor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 2
    msgType = "str"

class Control(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 3
    msgType = "dict"

class Brake(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 4
    msgType = "str"

class Record(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 5
    msgType = "str"

class Config(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 6
    msgType = "dict"

class Klem(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 7
    msgType = "str"

class DrivingMode(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 8
    msgType = "str"

class ToggleInstant(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 9
    msgType = "str"

class ToggleBatteryLvl(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 10
    msgType = "str"

class ToggleImuData(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 11
    msgType = "str"

class ToggleResourceMonitor(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 12
    msgType = "str"

class State(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 13
    msgType = "str"

class Brightness(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 14
    msgType = "str"

class Contrast(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 15
    msgType = "str"

class DropdownChannelExample(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 16
    msgType = "str"

class SliderChannelExample(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 17
    msgType = "str"

class ControlCalib(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 18
    msgType = "dict"

class IsAlive(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 19
    msgType = "bool"

class RequestSteerLimits(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 20
    msgType = "bool"


################################# From Nucleo ##################################
class BatteryLvl(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 1
    msgType = "int"

class ImuData(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 2
    msgType = "str"

class InstantConsumption(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 3
    msgType = "float"

class ResourceMonitor(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 4
    msgType = "dict"

class CurrentSpeed(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 5
    msgType = "float"

class CurrentSteer(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 6
    msgType = "float"

class ImuAck(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 7
    msgType = "str"
    
class ShutDownSignal(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 8
    msgType = "str"

class CalibPWMData(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 9
    msgType = "dict"

class CalibRunDone(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 10
    msgType = "bool"

class AliveSignal(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 11
    msgType = "bool"

class SteeringLimits(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 12
    msgType = "dict"

class StopLine(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 13
    msgType = "bool"

class ToggleStopLine(Enum):
    Queue = "General"
    Owner = "Dashboard"
    msgID = 14
    msgType = "int"

################################# From Locsys ##################################
class Location(Enum):
    Queue = "General"
    Owner = "threadTrafficCommunication"
    msgID = 1
    msgType = "dict"

######################    From processSerialHandler  ###########################
class EnableButton(Enum):
    Queue = "General"
    Owner = "threadWrite"
    msgID = 1
    msgType = "bool"

class WarningSignal(Enum):
    Queue = "General"
    Owner = "brain"
    msgID = 2
    msgType = "str"

class SerialConnectionState(Enum):
    Queue = "General"
    Owner = "processSerialHandler"
    msgID = 3
    msgType = "bool"

################################# From AutonomousDriving ##################################
class DetectionEvent(Enum):
    """Detection from YOLO/camera published by AD for DecisionMaking.
    
    Published every frame that has a YOLO detection or stop line.
    msgValue: {"detection": "red"|"stop"|...|None, "stop_line": True|False}
    """
    Queue = "General"
    Owner = "threadAutonomousDriving"
    msgID = 1
    msgType = "dict"

################################# From DecisionMaking ##################################
class CurrentDrivingMode(Enum):
    """Current driving mode published by the Decision Making component.
    
    msgValue format: "DRIVING_MODE:<MODE_NAME>" or "DRIVING_MODE:<MODE_NAME>|key=val,..."
    Examples:
        "DRIVING_MODE:LANE_FOLLOWING"
        "DRIVING_MODE:ROUNDABOUT|source=sign"
        "DRIVING_MODE:CROSSING|source=path_planning"
        "DRIVING_MODE:OBJECT_ON_ROAD|object_type=PEDESTRIAN,distance=1.2"
    """
    Queue = "General"
    Owner = "threadDecisionMaking"
    msgID = 1
    msgType = "str"

class SignDetection(Enum):
    """Traffic sign detection from camera/AI — consumed by Decision Making.
    
    msgValue format: "SIGN:<sign_type>:<confidence>"
    Examples:
        "SIGN:ROUNDABOUT:0.85"
        "SIGN:CROSSING:0.92"
        "SIGN:PEDESTRIAN_CROSSING:0.78"
        "SIGN:TUNNEL:0.90"
        "SIGN:HIGHWAY:0.88"
        "SIGN:PARKING:0.95"
        "SIGN:STOP:0.99"
        "SIGN:NO_ENTRY:0.91"
    """
    Queue = "General"
    Owner = "threadDecisionMaking"
    msgID = 2
    msgType = "str"

class ObjectDetection(Enum):
    """Object detection from camera/AI — consumed by Decision Making.
    
    msgValue format: "OBJECT:<object_type>:<distance_meters>"
    Examples:
        "OBJECT:PEDESTRIAN:1.5"
        "OBJECT:CAR:3.0"
        "OBJECT:OBSTACLE:0.8"
    """
    Queue = "General"
    Owner = "threadDecisionMaking"
    msgID = 3
    msgType = "str"

class TrafficLightDetection(Enum):
    """Traffic light state detection — consumed by Decision Making.
    
    msgValue format: "TRAFFIC_LIGHT:<color>"
    Examples:
        "TRAFFIC_LIGHT:RED"
        "TRAFFIC_LIGHT:YELLOW"
        "TRAFFIC_LIGHT:GREEN"
    """
    Queue = "General"
    Owner = "threadDecisionMaking"
    msgID = 4
    msgType = "str"

class SpeedCommand(Enum):
    """Speed command from DecisionMaking to AutonomousDriving.
    
    DM sends target speed; AD sets SpeedMotor accordingly.
    msgValue: "150", "300", etc.
    """
    Queue = "General"
    Owner = "threadDecisionMaking"
    msgID = 5
    msgType = "str"

class StopCommand(Enum):
    """Stop/resume command from DecisionMaking to AutonomousDriving.
    
    DM tells AD when to stop or resume driving.
    msgValue: "stop" or "resume"
    """
    Queue = "General"
    Owner = "threadDecisionMaking"
    msgID = 6
    msgType = "str"

################################# From StateMachine ##################################
class StateChange(Enum):
    Queue = "Critical"
    Owner = "stateMachine"
    msgID = 1
    msgType = "str"

### It will have this format: {"WarningName":"name1", "WarningID": 1}
