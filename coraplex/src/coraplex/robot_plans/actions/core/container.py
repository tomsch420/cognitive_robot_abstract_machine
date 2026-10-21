from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass

from typing_extensions import Any, Dict

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.factories import (
    variable_from,
    ConditionType,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import sequential
from coraplex.plans.plan_node import PlanNode
from coraplex.querying.predicates import GripperIsFree
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.pick_up import TakesHoldOfAGrasp
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.robot_plans.motions.container import OpeningMotion, ClosingMotion
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.robot_parts import Arm, EndEffector
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF


@dataclass
class ContainerAction(ActionDescription, TakesHoldOfAGrasp):
    """
    Moves a container like object by its handle.

    Taking hold of the handle, letting go of it again and clearing it afterwards is the
    same whichever way the container is moved; only the motion working the mechanism
    differs.
    """

    handle: Handle
    """
    The handle of the container that should be moved.
    """

    arm: Arm
    """
    Arm that should be used.
    """

    release_clearance: float = 0.05
    """
    The gap in meters between the handle and the gripper once it has let go.
    """

    grasp: GraspCandidate | None = None
    """
    The grasp the handle is taken hold of by; ``None`` takes it at the handle's origin.
    """

    @property
    def handle_grasp(self) -> GraspCandidate:
        """
        The grasp the handle is taken hold of by: :attr:`grasp`, or the one at the
        handle's origin if none is given.
        """
        if self.grasp is not None:
            return self.grasp
        return GraspCandidate.from_body_origin(self.handle)

    @property
    @abstractmethod
    def _mechanism_motion(self) -> BaseMotion:
        """
        :return: The motion that moves the container while its handle is held.
        """

    def back_off_pose(self, grasp_pose: Pose, end_effector: EndEffector) -> Pose:
        """
        The tool frame goal that clears the released handle.

        The gripper leaves the way it came in, so that an open gripper still straddling
        the handle is drawn off it rather than across it.

        :param grasp_pose: The grasp frame the handle was held by.
        :param end_effector: The end effector that held it.
        :return: The pose the tool frame backs off to, in ``grasp_pose``'s frame.
        """
        return self.pre_grasp_pose(grasp_pose, end_effector, self.release_clearance)

    @property
    def _action_plan(self) -> PlanNode:
        handle_grasp = self.handle_grasp
        return sequential(
            [
                self.grasping_of(handle_grasp, self.arm),
                self._mechanism_motion,
                MoveGripperMotion(
                    GripperState.OPEN,
                    self.arm.end_effector,
                    allow_gripper_collision=True,
                ),
                MoveToolCenterPointMotion(
                    self.back_off_pose(handle_grasp.grasp_pose, self.arm.end_effector),
                    self.arm,
                    allow_gripper_collision=True,
                    position_threshold=self.position_threshold,
                    orientation_threshold=self.orientation_threshold,
                ),
            ]
        )


@dataclass
class OpenAction(ContainerAction):
    """
    Opens a container like object.
    """

    @property
    def _mechanism_motion(self) -> BaseMotion:
        return OpeningMotion(self.handle.root, self.arm)

    @staticmethod
    def pre_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The gripper with which to open the container has to be free.
        """
        return GripperIsFree(variables["arm"].end_effector)

    @staticmethod
    def post_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The container has to be open.

        The gripper is clear of the handle by then, so what it holds says nothing about
        whether the container was opened.
        """
        open_connection = kwargs["handle"].root.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )

        return variable_from(open_connection).position > 0.3


@dataclass
class CloseAction(ContainerAction):
    """
    Closes a container like object.
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

    @property
    def _mechanism_motion(self) -> BaseMotion:
        return ClosingMotion(
            self.handle.root,
            self.arm,
            closed_position=self.closed_position,
            closing_tolerance=self.closing_tolerance,
        )

    @staticmethod
    def post_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression | bool:
        """
        The container has to be closed.
        """
        close_connection = kwargs["handle"].root.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )

        return variable_from(close_connection).position < 0.1
