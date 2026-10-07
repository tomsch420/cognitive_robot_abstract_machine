"""
The robots that pick objects up in the physical pick-up experiment, and where each of
them finds the objects it picks up.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from experiments.physical_pick_up.physical_simulation_preparation import (
    PR2PhysicalSimulationPreparation,
)
from semantic_digital_twin.api import RobotSpecification
from semantic_digital_twin.datastructures.definitions import (
    StaticJointState,
    TorsoState,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.grasping.surface_grasp import ParameterRange
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Color, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% where objects stand


@dataclass
class ObjectPlacement:
    """
    Where an object stands: the middle of its footprint, in the world frame, and how far
    it is turned about the vertical axis.
    """

    x: float
    """
    The x-coordinate of the middle of the object's footprint, in meters.
    """

    y: float
    """
    The y-coordinate of the middle of the object's footprint, in meters.
    """

    yaw: float = 0.0
    """
    How far the object is turned about the vertical axis from how it is modeled, in
    radians.
    """


@dataclass
class PickUpArea:
    """
    A rectangle on a horizontal surface where objects stand for a robot to pick them up,
    within reach of the robot's arm.
    """

    height: float
    """
    The height of the surface above the floor, in meters.
    """

    x: ParameterRange
    """
    Where the middle of an object's footprint may be along the world's x-axis.
    """

    y: ParameterRange
    """
    Where the middle of an object's footprint may be along the world's y-axis.
    """

    def middle(self) -> ObjectPlacement:
        """
        :return: The middle of the area, with the object turned as it is modeled.
        """
        return ObjectPlacement(
            x=(self.x.lower + self.x.upper) / 2, y=(self.y.lower + self.y.upper) / 2
        )

    def random_placement(
        self, generator: np.random.Generator, maximum_yaw: float
    ) -> ObjectPlacement:
        """
        :param generator: The source of randomness.
        :param maximum_yaw: How far the object may be turned either way, in radians.
        :return: A placement drawn uniformly within the area and the turn.
        """
        return ObjectPlacement(
            x=float(generator.uniform(self.x.lower, self.x.upper)),
            y=float(generator.uniform(self.y.lower, self.y.upper)),
            yaw=float(generator.uniform(-maximum_yaw, maximum_yaw)),
        )


# %% robots


class RobotSetup(ABC):
    """
    How a robot is put into a pick-up scene, which of its arms picks up, and where the
    objects it picks up stand.
    """

    @abstractmethod
    def spawn(self, world: World) -> AbstractRobot:
        """
        :param world: The world to put the robot into.
        :return: The robot, prepared for physical simulation and in its start
            configuration.
        """

    @abstractmethod
    def arm(self, robot: AbstractRobot) -> Arm:
        """
        :param robot: The robot this setup spawned.
        :return: The arm that picks objects up.
        """

    @abstractmethod
    def add_pick_up_area(self, world: World, robot: AbstractRobot) -> PickUpArea:
        """
        Add what the objects stand on, unless the robot brings it along.

        :param world: The world the robot was spawned into.
        :param robot: The robot this setup spawned.
        :return: Where objects stand for the robot to pick them up.
        """


@dataclass
class PR2Setup(RobotSetup):
    """
    A PR2 standing at the world's origin, facing a table along the x-axis, picking up
    with its left arm.
    """

    grip_torque: float = 8.0
    """
    The torque the finger servos press an object with, in newton meters.
    """

    table_scale: Scale = field(default_factory=lambda: Scale(0.6, 1.0, 0.72))
    """
    Depth, width and height of the table.
    """

    table_position: Point3 = field(default_factory=lambda: Point3(0.85, 0.0, 0.0))
    """
    Where the middle of the table stands on the floor.
    """

    reachable_x: ParameterRange = field(
        default_factory=lambda: ParameterRange(0.62, 0.74)
    )
    """
    Where along the x-axis the left arm reaches objects on the table.
    """

    reachable_y: ParameterRange = field(
        default_factory=lambda: ParameterRange(0.08, 0.22)
    )
    """
    Where along the y-axis the left arm reaches objects on the table.
    """

    def spawn(self, world: World) -> PR2:
        robot = RobotSpecification(PR2).spawn(world)
        PR2PhysicalSimulationPreparation(
            robot=robot, grip_torque=self.grip_torque
        ).apply()
        for arm in robot.all_arms:
            arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)
        robot.mobile_base.torso.get_joint_state_by_type(TorsoState.HIGH).apply_to(world)
        world.notify_state_change()
        return robot

    def arm(self, robot: PR2) -> Arm:
        return robot.left_arm

    def add_pick_up_area(self, world: World, robot: PR2) -> PickUpArea:
        body = Body(name=PrefixedName("table"))
        box = Box(
            origin=HomogeneousTransformationMatrix.from_xyz_rpy(reference_frame=body),
            scale=self.table_scale,
            color=Color(0.6, 0.45, 0.3, 1.0),
        )
        body.collision = ShapeCollection([box], reference_frame=body)
        body.visual = ShapeCollection([box], reference_frame=body)
        x, y, _ = self.table_position.to_np()[:3]
        with world.modify_world():
            world.add_connection(
                FixedConnection(
                    parent=world.root,
                    child=body,
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=x,
                        y=y,
                        z=self.table_scale.z / 2,
                        reference_frame=world.root,
                    ),
                )
            )
            world.add_semantic_annotation(Table(root=body))
        return PickUpArea(
            height=self.table_scale.z, x=self.reachable_x, y=self.reachable_y
        )


@dataclass
class TracySetup(RobotSetup):
    """
    Tracy at the world's origin, its two arms mounted at the near end of its own table,
    picking up with its left arm from that table.
    """

    reachable_x: ParameterRange = field(
        default_factory=lambda: ParameterRange(0.55, 0.75)
    )
    """
    Where along the x-axis the left arm reaches objects on the table.
    """

    reachable_y: ParameterRange = field(
        default_factory=lambda: ParameterRange(0.15, 0.35)
    )
    """
    Where along the y-axis the left arm reaches objects on the table.
    """

    def spawn(self, world: World) -> Tracy:
        robot = RobotSpecification(Tracy).spawn(world)
        for arm in robot.all_arms:
            arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)
        world.notify_state_change()
        return robot

    def arm(self, robot: Tracy) -> Arm:
        return robot.left_arm

    def add_pick_up_area(self, world: World, robot: Tracy) -> PickUpArea:
        return PickUpArea(
            height=robot.table.top_z, x=self.reachable_x, y=self.reachable_y
        )


# %% choosing a robot


class PickUpRobot(Enum):
    """
    The robots that can pick objects up in the experiment.
    """

    PR2 = PR2Setup()
    """
    A PR2 picking up from a table in front of it with its left arm.
    """

