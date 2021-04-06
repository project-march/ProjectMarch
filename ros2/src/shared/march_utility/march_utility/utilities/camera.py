from enum import Enum


class CameraSide(Enum):
    FRONT = 0
    BACK = 1

    @classmethod
    def from_string(cls, camera_string):
        if camera_string.lower() == "front":
            return cls.FRONT
        if camera_string.lower() == "back":
            return cls.BACK
        raise KeyError("This string does not correspond to a known camera side")
