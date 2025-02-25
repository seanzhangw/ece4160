from enum import Enum

class CMD(Enum):
    PING = 0
    ECHO = 1
    GET_ACCEL_PITCH = 2
    GET_ACCEL_ROLL = 3
    CALIBRATE_ACCEL = 4
    GET_250_ACCEL = 5
    GET_GYROSCOPE_DATA = 6
    GET_COMP_DATA = 7
    START_RECORD = 8
    STOP_RECORD = 9