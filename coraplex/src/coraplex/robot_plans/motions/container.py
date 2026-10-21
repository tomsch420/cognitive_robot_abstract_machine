from dataclasses import dataclass

from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.open_close import Open, Close
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.robot_plans.motions.base import BaseMotion
from semantic_digital_twin.robots.robot_parts import Arm


@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container.
    """

    object_part: Body
    """
    Object designator for the drawer handle
    """
    arm: Arm
    """
    Arm that should be used.
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        tip = self.arm.end_effector.tool_frame
        return Open(
            tip_link=tip,
            environment_link=self.object_part,
            mechanism_weight=DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE,
        )


@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container.
    """

    object_part: Body
    """
    Object designator for the drawer handle.
    """

    arm: Arm
    """
    Arm that should be used.
    """

    closed_position: float = 0.01
    """
    The position of the mechanism's joint the container is closed to.
    """

    closing_tolerance: float = 0.01
    """
    How close the joint has to come to :attr:`closed_position` for the container to
    count as closed.
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        tip = self.arm.end_effector.tool_frame
        return Close(
            tip_link=tip,
            environment_link=self.object_part,
            goal_joint_state=self.closed_position,
            threshold=self.closing_tolerance,
            mechanism_weight=DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE,
        )
