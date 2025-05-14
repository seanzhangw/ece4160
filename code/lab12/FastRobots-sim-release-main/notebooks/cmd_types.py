from enum import Enum

class CMD(Enum):
    PING = 0
    GET_TOF1 = 1
    GET_TOF2 = 2
    FORWARD_CONTROLLED = 3
    FORWARD_EXTRAPOLATE = 4
    KALMAN_CONTROL = 5
    FORWARD = 6
    BACKWARD = 7
    YAW_CONTROL = 8
    LOCALIZE = 9
    OPEN_LOOP_NAV = 10
    STOP = 11
