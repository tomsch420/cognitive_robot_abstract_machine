from __future__ import annotations

from enum import IntEnum, Enum
from typing import Union

import semantic_digital_twin.spatial_types.spatial_types as cas

goal_parameter = Union[str, float, bool, dict, list, IntEnum, None]


class LifeCycleValues(IntEnum):
    NOT_STARTED = 0
    RUNNING = 1
    PAUSED = 2
    DONE = 3
    FAILED = 4


class FloatEnum(float, Enum):
    """Enum where members are also (and must be) floats"""


class ObservationStateValues(FloatEnum):
    FALSE = cas.TrinaryFalse.to_np()[0]
    UNKNOWN = cas.TrinaryUnknown.to_np()[0]
    TRUE = cas.TrinaryTrue.to_np()[0]


class DefaultWeights(FloatEnum):
    WEIGHT_MAX = 10000.0
    WEIGHT_ABOVE_CA = 2500.0
    WEIGHT_COLLISION_AVOIDANCE = 50.0
    WEIGHT_BELOW_CA = 1.0
    WEIGHT_MIN = 0.0
