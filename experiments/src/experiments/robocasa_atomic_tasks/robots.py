"""
The robots that perform RoboCasa's atomic tasks, and what each brings into the kitchen:
how it is put where RoboCasa starts its own robot, the arm it acts with, and where the
video of an episode is seen from.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import StrEnum

from experiments.robocasa_atomic_tasks.panda_omron import PandaOmron, PandaOmronBody
from experiments.robocasa_atomic_tasks.stands import StandCandidates
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.world_entity import Body

# %% what a robot brings


@dataclass
class TaskRobot(ABC):
    """
    A robot that performs RoboCasa's atomic tasks.
    """

    stand_candidates: StandCandidates = field(default_factory=StandCandidates)
    """
    Where the robot tries to stand relative to what it reaches for.
    """

    @abstractmethod
    def place_in(
        self, world: World, start_pose: HomogeneousTransformationMatrix
    ) -> AbstractRobot:
        """
        Put the robot into a kitchen, its base on a drive that moves it across the
        floor.

        :param world: The kitchen.
        :param start_pose: Where RoboCasa starts its own robot, in the world's root
            frame.
        :return: The robot, standing at ``start_pose``.
        """

    @abstractmethod
    def acting_arm(self, robot: AbstractRobot) -> Arm:
        """
        :param robot: The robot, as :meth:`place_in` put it into the kitchen.
        :return: The arm it performs the tasks with.
        """

    @abstractmethod
    def video_eye(self, robot: AbstractRobot) -> Body:
        """
        :param robot: The robot, as :meth:`place_in` put it into the kitchen.
        :return: The body the video of an episode is seen from.
        """

    @abstractmethod
    def bodies_hidden_from_the_video(self, robot: AbstractRobot) -> list[Body]:
        """
        :param robot: The robot, as :meth:`place_in` put it into the kitchen.
        :return: The bodies around :meth:`video_eye` the video would see from inside.
        """

    @abstractmethod
    def wrist_bodies(self, robot: AbstractRobot) -> list[Body]:
        """
        :param robot: The robot, as :meth:`place_in` put it into the kitchen.
        :return: The bodies of :meth:`acting_arm` just behind its gripper, which may
            brush what stands around a part the gripper pushes.
        """


# %% the PR2


class PR2WristLink(StrEnum):
    """
    The links of the PR2's right arm just behind its gripper.
    """

    FOREARM = "r_forearm_link"
    WRIST_FLEX = "r_wrist_flex_link"
    WRIST_ROLL = "r_wrist_roll_link"


class PR2HeadLink(StrEnum):
    """
    The links of the PR2's head the video is seen from.
    """

    CAMERA = "wide_stereo_optical_frame"
    """
    The frame of the camera in the PR2's head.
    """

    PAN = "head_pan_link"
    """
    The link the head, and everything on it, hangs from; left out of the video, as the
    camera turns without it.
    """


@dataclass
class PR2Robot(TaskRobot):
    """
    The PR2, acting with its right arm, as RoboCasa's robot does, and seen from the
    camera in its head.
    """

    def place_in(
        self, world: World, start_pose: HomogeneousTransformationMatrix
    ) -> PR2:
        pr2_world = URDFParser.from_file(PR2.get_ros_file_path()).create_world()
        with world.modify_world():
            drive = OmniDrive.create_with_dofs(
                parent=world.root, child=pr2_world.root, world=world
            )
            world.merge_world(pr2_world, drive)
            drive.has_hardware_interface = True
        drive.origin = start_pose
        robot = PR2.from_world(world)
        robot.mobile_base.full_body_controlled = True
        return robot

    def acting_arm(self, robot: PR2) -> Arm:
        return robot.right_arm

    def video_eye(self, robot: PR2) -> Body:
        return robot._world.get_body_by_name(PR2HeadLink.CAMERA)

    def bodies_hidden_from_the_video(self, robot: PR2) -> list[Body]:
        world = robot._world
        head = world.get_body_by_name(PR2HeadLink.PAN)
        return [
            body
            for body in [
                head,
                *world.compute_descendent_child_kinematic_structure_entities(head),
            ]
            if isinstance(body, Body)
        ]

    def wrist_bodies(self, robot: PR2) -> list[Body]:
        return [robot._world.get_body_by_name(link) for link in PR2WristLink]


# %% RoboCasa's own robot


@dataclass
class PandaOmronRobot(TaskRobot):
    """
    RoboCasa's own robot, seen from the top of its lift, where its arm is mounted.
    """

    def place_in(
        self, world: World, start_pose: HomogeneousTransformationMatrix
    ) -> PandaOmron:
        return RobotSpecification(PandaOmron, odom_T_robot_start=start_pose).spawn(
            world
        )

    def acting_arm(self, robot: PandaOmron) -> Arm:
        [arm] = robot.all_arms
        return arm

    def video_eye(self, robot: PandaOmron) -> Body:
        return robot._world.get_body_by_name(PandaOmronBody.ARM_BASE)

    def bodies_hidden_from_the_video(self, robot: PandaOmron) -> list[Body]:
        return [robot._world.get_body_by_name(PandaOmronBody.ARM_BASE)]

    def wrist_bodies(self, robot: PandaOmron) -> list[Body]:
        return [
            robot._world.get_body_by_name(link)
            for link in (
                PandaOmronBody.FOREARM,
                PandaOmronBody.WRIST,
                PandaOmronBody.FLANGE,
            )
        ]


# %% choosing one


class RobotModel(StrEnum):
    """
    The robots that can perform the tasks.
    """

    PR2 = "pr2"
    """
    The PR2.
    """

    PANDA_OMRON = "panda_omron"
    """
    RoboCasa's own robot, a Franka Panda on an Omron mobile base.
    """

    def task_robot(self) -> TaskRobot:
        """
        :return: What this robot brings into the kitchen.
        """
        if self is RobotModel.PANDA_OMRON:
            return PandaOmronRobot()
        return PR2Robot()
