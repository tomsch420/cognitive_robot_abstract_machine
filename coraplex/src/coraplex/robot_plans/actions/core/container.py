"""
The actions that open and close a container by moving its handle.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


from krrood.entity_query_language.factories import (
    a,
    variable_from,
    variable,
)
from krrood.entity_query_language.query.match import Match
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.base import SequentialAction
from cramph.node import StatechartNode
from coraplex.robot_plans.actions.core.pick_up import (
    HasGraspChoice,
    TakesHoldOfAGrasp,
)
from coraplex.robot_plans.mixins import (
    HasApproachesGraspPoses,
    OperatesGripper,
)
from giskardpy.motion_statechart.graph_node import MotionStatechartNode
from giskardpy.motion_statechart.tasks.grasped_mechanism import (
    ArticulateGraspedMechanism,
)
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForEndEffector,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.robot_parts import Arm, EndEffector
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
)
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)
from semantic_digital_twin.world_description.world_entity import Body

# %% taking a container by its handle


@dataclass(eq=False, repr=False, kw_only=True)
class MovesContainerByItsHandle(
    TakesHoldOfAGrasp, OperatesGripper, SequentialAction, HasGraspChoice, ABC
):
    """
    An action that takes hold of a container's handle by :attr:`grasp`, moves the
    container with it, lets go and clears the handle.
    """

    release_clearance: float = 0.05
    """
    How far in meters the gripper backs off once it has let go, past where it leaves
    :attr:`moved_part_bounding_box`.
    """

    @classmethod
    def from_handle(cls, handle: Handle, arm: Arm) -> Match:
        """
        :param handle: The handle of the container.
        :param arm: The arm that takes hold of the handle.
        :return: This action on `handle`, with the grasp left open among the grasps
            the handle offers, the closest to where the robot stands when it is grounded
            first.
        """
        # A variable, so the grasps are ranked when the action is grounded, from where
        # the robot stands then.
        where_the_robot_stands = variable(
            Pose, domain=[Pose(reference_frame=arm._robot.root)]
        )
        return a(cls)(
            grasp=variable_from(
                GraspsByCloseness(
                    graspable=handle,
                    standing_position=where_the_robot_stands,
                    number_of_grasps=None,
                )
            ),
            arm=arm,
        )

    @abstractmethod
    def _create_mechanism_goal(self) -> MotionStatechartNode:
        """
        :return: A new goal moving the container while its handle is held.
        """

    @property
    def mechanism(self) -> ActiveConnection1DOF:
        """
        The connection the container moves along while its handle is held.
        """
        return self.grasp.graspable.root.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )

    @property
    def moved_part_bounding_box(self) -> VolumetricBoundingBox:
        """
        The box bounding the body :attr:`mechanism` moves or, where that body has no
        geometry of its own, the bodies with geometry below it, in the frame of the body
        :attr:`mechanism` moves.
        """
        part = self.mechanism.child
        bounded = (
            [part]
            if isinstance(part, Body) and part.has_collision()
            else [
                body
                for body in part._world.compute_descendent_child_kinematic_structure_entities(
                    part
                )
                if isinstance(body, Body) and body.has_collision()
            ]
        )
        return BoundingBoxCollection.merge_all(
            (
                body.collision.as_bounding_box_collection_in_frame(part)
                for body in bounded
            ),
            part,
        ).bounding_box()

    def back_off_pose(self, grasp_pose: Pose, end_effector: EndEffector) -> Pose:
        """
        The tool frame goal that clears the released handle and the container, back
        along the way the gripper came in, so an open gripper still straddling the
        handle is drawn off it rather than across it.

        :param grasp_pose: The grasp frame the handle was held by.
        :param end_effector: The end effector that held it.
        :return: The pose the tool frame backs off to, in `grasp_pose`'s frame.
        """
        distance = (
            HasApproachesGraspPoses.distance_out_of(
                self.moved_part_bounding_box, grasp_pose
            )
            + self.release_clearance
        )
        return HasApproachesGraspPoses.pre_grasp_pose(
            grasp_pose, end_effector, distance
        )

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper on the handle to touch its surroundings,
            from taking hold of the handle until it has backed off it.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
        end_effector = self.arm.end_effector
        return [
            self.grasping_of(self.grasp, self.arm),
            self._create_mechanism_goal(),
            self.gripper_goal_type.moving_gripper_to(
                end_effector.default_configuration(GripperState.OPEN)
            ),
            self.tool_center_point_goal(
                self.back_off_pose(self.grasp.grasp_pose, end_effector),
                self.arm,
            ),
        ]


# %% opening and closing


@dataclass(eq=False, repr=False, kw_only=True)
class OpenAction(MovesContainerByItsHandle):
    """
    Opens a container like object, driving its mechanism to its upper limit.
    """

    def _create_mechanism_goal(self) -> MotionStatechartNode:
        return ArticulateGraspedMechanism(
            tip_link=self.arm.end_effector.tool_frame,
            environment_link=self.grasp.graspable.root,
            goal_joint_state=self.mechanism.dof.limits.upper.position,
        )


@dataclass(eq=False, repr=False, kw_only=True)
class CloseAction(MovesContainerByItsHandle):
    """
    Closes a container like object.
    """

    closed_joint_position: float = 0.01
    """
    The joint position, in meters or radians, the container's mechanism is driven to, or
    its lower limit if that lies above.
    """

    closing_tolerance: float = ArticulateGraspedMechanism.threshold
    """
    How close the mechanism has to come to :attr:`closed_joint_position` for the
    container to count as closed.
    """

    def _create_mechanism_goal(self) -> MotionStatechartNode:
        return ArticulateGraspedMechanism(
            tip_link=self.arm.end_effector.tool_frame,
            environment_link=self.grasp.graspable.root,
            goal_joint_state=self.closed_joint_position,
            threshold=self.closing_tolerance,
        )
