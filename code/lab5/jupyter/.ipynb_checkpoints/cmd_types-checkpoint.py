from enum import Enum

class CMD(Enum):
    PING = 0
    GET_TOF1 = 1
    GET_TOF2 = 2
    GET_TOF_300 = 3
    FORWARD_CONTROLLED = 4
    FORWARD_EXTRAPOLATE = 5
    FORWARD = 6
    BACKWARD = 7
    STOP = 8
