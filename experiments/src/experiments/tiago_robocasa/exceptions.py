"""
What can go wrong setting a RoboCasa task up for Tiago.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from krrood.exceptions import DataclassException
from semantic_digital_twin.spatial_types.spatial_types import Pose


@dataclass
class NoClearStandingPose(DataclassException):
    """
    Raised when Tiago touches the kitchen wherever it is stood to reach a target.
    """

    target: np.ndarray
    """
    What it was to reach.
    """

    tried: int
    """
    How many stands were tried.
    """

    def error_message(self) -> str:
        return (
            f"Tiago touches the kitchen at every one of the {self.tried} stands tried "
            f"for reaching {np.round(self.target, 3)}."
        )

    def suggest_correction(self) -> str:
        return "Look further back, or work with the other arm."


@dataclass
class NoReachableStandingPose(DataclassException):
    """
    Raised when every stand that keeps Tiago clear of the kitchen leaves the target
    outside the arm's working envelope.
    """

    target: np.ndarray
    """
    What it was to reach.
    """

    tried: int
    """
    How many stands were tried.
    """

    clear: int
    """
    How many of them kept Tiago clear of the kitchen.
    """

    def error_message(self) -> str:
        return (
            f"The arm reaches {np.round(self.target, 3)} from none of the {self.clear} "
            f"of {self.tried} stands that keep Tiago clear of the kitchen."
        )

    def suggest_correction(self) -> str:
        return "Reach for something nearer the robot, or work with the other arm."


@dataclass
class NoPlaceToLetGo(DataclassException):
    """
    Raised when the arm reaches nowhere over a receptacle that the object it holds could
    be let go above.
    """

    receptacle: str
    """
    What it was to be put into.
    """

    tried: int
    """
    How many stands in front of that receptacle were tried.
    """

    def error_message(self) -> str:
        return (
            f"From none of the {self.tried} stands in front of the {self.receptacle} "
            f"does the arm reach anywhere over it that the object could be let go."
        )

    def suggest_correction(self) -> str:
        return "Let go higher above the receptacle, or work with the other arm."


@dataclass
class NoWayToTheStand(DataclassException):
    """
    Raised when the kitchen's free space holds no way for the base to a stand.
    """

    stand: Pose
    """
    Where Tiago was to drive.
    """

    def error_message(self) -> str:
        return (
            f"The kitchen's free space holds no way for the base to "
            f"{np.round(self.stand.to_np()[:3, 3], 3)}."
        )

    def suggest_correction(self) -> str:
        return "Keep the way less far off the kitchen, or work from where it stands."


@dataclass
class NoGraspTheArmReaches(DataclassException):
    """
    Raised when the arm reaches none of the learned grasps of an object.
    """

    annotation_type: str
    """
    The kind of object the grasps were learned for.
    """

    tried: int
    """
    How many of them were tried.
    """

    def error_message(self) -> str:
        return (
            f"The arm reaches none of the {self.tried} grasps tried of those learned "
            f"for a {self.annotation_type}."
        )

    def suggest_correction(self) -> str:
        return "Try more of the drawn grasps, or stand somewhere else."
