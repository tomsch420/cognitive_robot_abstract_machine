"""
The part the HSR's RoboCasa tasks share where the HSR moves a fixture by its handle: a
drawer along its rails, a door about its hinge.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from typing_extensions import Callable

import numpy as np
from scipy.spatial.transform import Rotation
from segmind.detectors.mechanism_detector_nodes import OpeningDetector
from segmind.event_segmentation import Segmind
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.container import ContainerAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    PrismaticConnection,
    RevoluteConnection,
)

from experiments.hsr_robocasa.robocasa_task import HSRRoboCasaTask


@dataclass
class HSRHandleTask(HSRRoboCasaTask, ABC):
    """
    One of RoboCasa's tasks in which the HSR takes hold of a fixture's handle from the
    front and moves the fixture by it.
    """

    handle_reach: float = 0.6
    """
    How far in front of its base's centre the HSR takes hold of the handle: from the
    front, and with its hand level the arm reaches 0.37 m to 0.70 m ahead at a
    handle's height.
    """

    tool_frame_tolerance: float = 0.02
    """
    Following the fixture, the base moves sideways too, by 7 cm in closing a drawer,
    and the arm, swinging in its plane, cannot make up for that when backing off the
    released handle: it stops 1.6 cm to the side of where it backs off to.
    """

    open_ratio: float = 0.9
    """
    The share of its travel from which on RoboCasa counts the fixture open.
    """

    closed_ratio: float = 0.005
    """
    The share of its travel up to which RoboCasa counts the fixture closed.
    """

    @property
    @abstractmethod
    def handle(self) -> Handle:
        """
        The handle the fixture is moved by.
        """

    @property
    def moving_part(self) -> Body:
        """
        The part of the fixture that moves on its joint, carrying the handle.
        """
        return self.handle.root.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        ).child

    def build_segmind(self) -> Segmind:
        """
        :return: SegMind watching :attr:`moving_part` come to stand open or closed, as
            RoboCasa counts it.
        """
        return Segmind(
            world=self.world,
            detectors=[
                OpeningDetector(
                    tracked_object=self.moving_part,
                    open_ratio=self.open_ratio,
                    closed_ratio=self.closed_ratio,
                )
            ],
        )

    @property
    def handle_middle(self) -> np.ndarray:
        """
        Where the handle is taken hold of: its middle.
        """
        return self.extent(self.handle.root).center.to_np()[:3]

    @property
    def closing_shift(self) -> np.ndarray:
        """
        The transform, in the kitchen's frame, that moves the fixture's moving part
        from where it stands now to where it stands closed, at its joint's lower
        limit.
        """
        joint = self.moving_part.parent_connection
        travel = joint.dof.limits.lower.position - joint.position
        movement = np.eye(4)
        match joint:
            case RevoluteConnection():
                movement[:3, :3] = Rotation.from_rotvec(
                    joint.axis.to_np()[:3] * travel
                ).as_matrix()
            case PrismaticConnection():
                movement[:3, 3] = joint.axis.to_np()[:3] * travel
        world_T_connection = joint.child.global_pose.to_np() @ np.linalg.inv(
            joint.connection_T_child_expression.to_np()
        )
        return world_T_connection @ movement @ np.linalg.inv(world_T_connection)

    def handle_grasp(self) -> GraspCandidate:
        """
        :return: The grasp that takes the handle at :attr:`handle_middle`, approaching
            it from the HSR's side of the counter, the fingers closing across the bar
            from above and below; level where the fixture is closed, and turned with
            the fixture where it stands open.
        """
        handle = self.handle
        closing_shift = self.closing_shift
        closed_middle = (closing_shift @ np.append(self.handle_middle, 1.0))[:3]
        world_T_closed_grasp = HomogeneousTransformationMatrix.from_xyz_rpy(
            *closed_middle,
            roll=math.pi / 2,
            yaw=math.pi / 2,
            reference_frame=self.world.root,
        ).to_np()
        world_T_grasp = HomogeneousTransformationMatrix(
            np.linalg.inv(closing_shift) @ world_T_closed_grasp,
            reference_frame=self.world.root,
        )
        return GraspCandidate(
            handle, self.world.transform(world_T_grasp.pose, handle.root)
        )

    def handle_plan(
        self, context: Context, build_action: Callable[..., ContainerAction]
    ) -> Plan:
        """
        :param context: The context the actions are built in.
        :param build_action: Builds the action that moves the fixture by its handle,
            from the handle and the arm, and the grasp as ``grasp``.
        :return: The plan that stands in front of the handle, moves the arm to where
            taking hold of the handle starts its approach, and moves the fixture.
        """
        robot = context.robot
        arm = robot.all_arms[0]
        grasp = self.handle_grasp()
        action = build_action(self.handle, arm, grasp=grasp)
        pre_grasp = action.grasp_pose_sequence(
            grasp.world_T_grasp, arm.end_effector, grasp
        ).pre_grasp
        standing_pose = self.standing_pose(self.handle_middle, self.handle_reach)
        return sequential(
            [
                ParkArmsAction(robot.all_arms),
                NavigateAction(standing_pose),
                *self.arm_motions_to(pre_grasp, standing_pose),
                action,
                ParkArmsAction(robot.all_arms),
            ],
            context=context,
        ).plan
