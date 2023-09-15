from dotmap import DotMap
pepper = DotMap()

############## PEPPER SETUP ###############
pepper.ip = "10.0.1.214"
pepper.port = 9559

############## PEPPER SPEECH ##############
pepper.speech.language = "English"
pepper.speech.animated = True
pepper.speech.rspd = 88
pepper.speech.comma_pause = 150
pepper.speech.period_pause = 300

############## PEPPER MOTION ##############
pepper.motion.head_period = 0.3

############## PEPPER EVENTS ##############
pepper.events = [
    "ALBasicAwareness/HumanLost:ALMemory",
    "ALBasicAwareness/HumanTracked:ALMemory",
    "BackBumperPressed:ALMemory",    
    "HandLeftBackTouched:ALMemory",
    "HandRightBackTouched:ALMemory",    
    "LeftBumperPressed:ALMemory",    
    "ALTextToSpeech/TextDone:ALMemory"   
]

    
'''
This file contains the lists of ALMemory keys used in the ROS nodes of this project
'''

laser_keys = [
    # Right X
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/X/Sensor/Value",
    # Right Y
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/Y/Sensor/Value",
    # Front X
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/X/Sensor/Value",
    # Front Y
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/Y/Sensor/Value",
    # Left X
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/X/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/X/Sensor/Value",
    # Left Y
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/Y/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/Y/Sensor/Value"
]
'''
Keys needed to read the base lasers
'''

operation_mode_keys = [
    "Device/SubDeviceList/Platform/LaserSensor/Right/Reg/OperationMode/Actuator/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Reg/OperationMode/Actuator/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Reg/OperationMode/Sensor/Value", "Device/SubDeviceList/Platform/LaserSensor/Left/Reg/OperationMode/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Reg/OperationMode/Actuator/Value", "Device/SubDeviceList/Platform/LaserSensor/Front/Reg/OperationMode/Sensor/Value",
]
'''
Keys needed to read and set the operation mode of the lasers actuators
'''

shovel_keys = [
    "Device/SubDeviceList/Platform/LaserSensor/Front/Shovel/Seg01/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Shovel/Seg01/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Shovel/Seg02/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Shovel/Seg02/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Shovel/Seg03/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Shovel/Seg03/Y/Sensor/Value",
]
'''
Keys needed to read the data of the shovel sensors
'''

bumpers_keys = [
    "Device/SubDeviceList/Platform/FrontRight/Bumper/Sensor/Value",
    "Device/SubDeviceList/Platform/FrontLeft/Bumper/Sensor/Value",
    "Device/SubDeviceList/Platform/Back/Bumper/Sensor/Value"
]
'''
Keys needed to read bumpers status.
Bumpers can be pressed or un-pressed
'''

sonar_keys = [
    "Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value",
    "Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value"
]
'''
Keys needed to read the data of sonar sensors
'''

move_conf_keys = [
    "MaxVelXY", "MaxVelTheta", "MaxAccXY", "MaxAccTheta", "MaxJerkXY", "MaxJerkTheta"
]
'''
Keys needed to read and set the limits of Pepper controller
'''

rhand_key = "Device/SubDeviceList/RHand/Touch/Back/Sensor/Value"
'''
Key needed to read the status of right Pepper's hand.
The hand status can be pressed or un-pressed
'''

GO = "0"
'''
Status variable
'''

STOP = "1"
'''
Status variable
'''

ACTIVE = "1"
'''
Status variable
'''

DISABLED = "0"
'''
Status variable
'''

PRESSED = 1.0
'''
Status variable
'''

UNPRESSED = 0.0
'''
Status variable
'''

STAND_INIT = "StandInit"
'''
Status variable for the stand robot posture
'''

CROUCH = "Crouch"
'''
Status variable for the crouch robot posture
'''

PING_PORT = 9090
'''
Port used by own server to check the connection among the robot and the remote PC
'''

PING_TIMEOUT = 1.0
'''
Timeout in seconds before the connection is considered broken
'''

security_sys_keys = {}
'''
Dictionary used to store memory keys used to communicate with the robot
'''
security_sys_keys["mem_key"] = "Navigation"
security_sys_keys["nao_ip"] = security_sys_keys["mem_key"] + "/nao_ip"
security_sys_keys["debug"] = security_sys_keys["mem_key"] + "/debug"
security_sys_keys["stop"] = security_sys_keys["mem_key"] + "/stop"
security_sys_keys["security_distance_laser_key"] = security_sys_keys["mem_key"] + "/security_distance_laser"
security_sys_keys["security_distance_sonar_key"] = security_sys_keys["mem_key"] + "/security_distance_sonar"
security_sys_keys["security_distance_shovel_key"] = security_sys_keys["mem_key"] + "/security_distance_shovel"