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
    msgType = "float"

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
    
class WarningSignal(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 7
    msgType = "str"

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
    msgID = 3
    msgType = "str"

### It will have this format: {"WarningName":"name1", "WarningID": 1}
