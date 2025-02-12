from enum import Enum

class CMD(Enum):
    PING = 0
    GET_ACCEL_PITCH = 1
    GET_ACCEL_ROLL = 2
    CALIBRATE_ACCEL = 3
    GET_250_ACCEL = 4
    GET_GYROSCOPE_DATA =5
    GET_COMP_DATA = 6
    START_RECORD = 7
    STOP_RECORD = 8