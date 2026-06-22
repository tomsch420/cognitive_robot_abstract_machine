from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException


class DontPrintStackTrace:
    """
    Marker mixin for exceptions whose stack trace should not be printed.
    """


@dataclass
class GiskardException(DataclassException):
    """
    Base class for all errors raised by Giskard.
    """


@dataclass
class SetupException(GiskardException):
    """
    Base class for errors that occur while configuring Giskard.
    """


@dataclass
class PlanningException(GiskardException):
    """
    Base class for errors that occur while planning a motion.
    """


@dataclass
class MissingActionResultError(GiskardException):
    """
    Raised when a result message is requested before one has been set.
    """

    def error_message(self) -> str:
        return "No result message set."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class DuplicateNameException(GiskardException):
    """
    Raised when a name that must be unique is used more than once.
    """

    name: str
    """
    The name that already exists.
    """

    def error_message(self) -> str:
        return f'A constraint named "{self.name}" already exists.'

    def suggest_correction(self) -> str:
        return ""


@dataclass
class NoControlledJointsError(SetupException):
    """
    Raised when no joint is flagged as controlled.
    """

    def error_message(self) -> str:
        return "No joints are flagged as controlled."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class JointRegistrationRequiresStandaloneModeError(SetupException):
    """
    Raised when joints are registered outside of StandAlone mode.
    """

    def error_message(self) -> str:
        return "Joints only need to be registered in StandAlone mode."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class NoQPControllerConfigException(SetupException):
    """
    Raised when the motion statechart has constraints but no QP controller config was provided.
    """

    def error_message(self) -> str:
        return "Motion Statechart has constraints, but no QP controller config is provided."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class MaxTrajectoryLengthException(PlanningException):
    """
    Raised when a planned trajectory exceeds the maximum allowed length.
    """

    def error_message(self) -> str:
        return "Trajectory exceeded the maximum allowed length."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class SelfCollisionViolatedException(PlanningException):
    """
    Raised when a self-collision constraint is violated during planning.
    """

    def error_message(self) -> str:
        return "A self-collision constraint was violated during planning."

    def suggest_correction(self) -> str:
        return ""
