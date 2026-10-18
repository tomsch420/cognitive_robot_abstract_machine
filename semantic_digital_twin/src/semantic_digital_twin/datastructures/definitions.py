from enum import Enum, IntEnum, auto


class JointStateType(Enum): ...


class GripperState(JointStateType):
    OPEN = auto()
    CLOSE = auto()
    MEDIUM = auto()


class TorsoState(JointStateType):
    HIGH = auto()
    MID = auto()
    LOW = auto()


class StaticJointState(JointStateType):
    PARK = auto()


class Axis(IntEnum):
    """
    An axis of a frame, by its index in a coordinate vector.
    """

    X = 0
    Y = 1
    Z = 2
