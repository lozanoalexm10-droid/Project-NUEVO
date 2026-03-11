"""
TLV Type Definitions - Auto-generated from TLV_TypeDefs.json
This file is auto-generated - DO NOT EDIT MANUALLY
"""

# ============================================================================
# TLV Type Constants
# ============================================================================

SYS_HEARTBEAT = 1
SYS_STATUS = 2
SYS_CMD = 3
SYS_CONFIG = 4
SYS_SET_PID = 5
DC_ENABLE = 256
DC_SET_POSITION = 257
DC_SET_VELOCITY = 258
DC_SET_PWM = 259
DC_STATUS_ALL = 260
STEP_ENABLE = 512
STEP_SET_PARAMS = 513
STEP_MOVE = 514
STEP_HOME = 515
STEP_STATUS_ALL = 516
SERVO_ENABLE = 768
SERVO_SET = 769
SERVO_STATUS_ALL = 770
SENSOR_IMU = 1024
SENSOR_KINEMATICS = 1025
SENSOR_VOLTAGE = 1026
SENSOR_RANGE = 1027
SENSOR_MAG_CAL_CMD = 1028
SENSOR_MAG_CAL_STATUS = 1029
IO_SET_LED = 1280
IO_SET_NEOPIXEL = 1281
IO_STATUS = 1282

# Dictionary for programmatic access
TLV_TYPES = {
    'SYS_HEARTBEAT': 1,
    'SYS_STATUS': 2,
    'SYS_CMD': 3,
    'SYS_CONFIG': 4,
    'SYS_SET_PID': 5,
    'DC_ENABLE': 256,
    'DC_SET_POSITION': 257,
    'DC_SET_VELOCITY': 258,
    'DC_SET_PWM': 259,
    'DC_STATUS_ALL': 260,
    'STEP_ENABLE': 512,
    'STEP_SET_PARAMS': 513,
    'STEP_MOVE': 514,
    'STEP_HOME': 515,
    'STEP_STATUS_ALL': 516,
    'SERVO_ENABLE': 768,
    'SERVO_SET': 769,
    'SERVO_STATUS_ALL': 770,
    'SENSOR_IMU': 1024,
    'SENSOR_KINEMATICS': 1025,
    'SENSOR_VOLTAGE': 1026,
    'SENSOR_RANGE': 1027,
    'SENSOR_MAG_CAL_CMD': 1028,
    'SENSOR_MAG_CAL_STATUS': 1029,
    'IO_SET_LED': 1280,
    'IO_SET_NEOPIXEL': 1281,
    'IO_STATUS': 1282,
}

# Reverse map: integer type id → name string
TLV_NAMES = {v: k for k, v in TLV_TYPES.items()}
