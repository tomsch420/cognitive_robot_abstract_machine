"""
RoboCasa's own simulation as the judge of an episode: never stepped, but brought into
the state of the world our robot acts in, with RoboCasa's robot put where our robot is.
"""

from __future__ import annotations

import logging
from contextlib import contextmanager
from dataclasses import dataclass, field

import mujoco
import numpy as np
from typing_extensions import TYPE_CHECKING, Any, Iterator

from experiments.robocasa_atomic_tasks.robocasa_names import (
    RoboCasaRobotBodyPrefix,
    RoboCasaRobotFrame,
)
from experiments.robocasa_goals.simulation_mirror import RoboCasaSimulationMirror
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from robocasa.environments.kitchen.kitchen import Kitchen

logger = logging.getLogger(__name__)

# %% what RoboCasa judged


@dataclass
class JudgedObject:
    """
    One of the task's objects as RoboCasa's simulation held it when it judged the
    episode.
    """

    name: str
    """
    The name RoboCasa's task gives the object.
    """

    position: list[float]
    """
    The position of the object's root body in the world frame, in meters.
    """

    distance_to_gripper: float
    """
    How far the object's root body stood from the end effector site of RoboCasa's
    gripper, in meters.
    """

    touched_bodies: list[str]
    """
    The names of the bodies RoboCasa's simulation had the object's root body touch,
    sorted.
    """


@dataclass
class JudgedJointPosition:
    """
    The position of one of the kitchen's hinge or slide joints as RoboCasa's simulation
    held it when it judged the episode.
    """

    joint_name: str
    """
    The name of the joint in RoboCasa's simulation.
    """

    position: float
    """
    The position of the joint, in radians for a hinge and meters for a slide.
    """


@dataclass
class Judgement:
    """
    RoboCasa's verdict on an episode, with the state of its simulation it judged.
    """

    accomplished: bool
    """
    Whether RoboCasa's own check found the task accomplished.
    """

    judged_objects: list[JudgedObject]
    """
    Every object of the task, in the order RoboCasa's task lists them.
    """

    judged_joint_positions: list[JudgedJointPosition]
    """
    Every hinge and slide joint of the kitchen, without RoboCasa's robot, in the order
    of RoboCasa's model.
    """


# %% the judge


@dataclass
class RoboCasaJudge:
    """
    Judges an episode by RoboCasa's own success check, and answers what RoboCasa's
    simulation finds touching.
    """

    environment: Kitchen
    """
    The RoboCasa environment the episode was composed in.
    """

    world: World
    """
    The world our robot acts in.
    """

    robot: AbstractRobot
    """
    The robot acting.
    """

    arm: Arm
    """
    The arm the robot acts with, standing in for RoboCasa's robot's gripper.
    """

    mirror: RoboCasaSimulationMirror = field(init=False, repr=False)
    """
    Keeps RoboCasa's simulation in the state of the world.
    """

    pressed_geoms: set[str] = field(default_factory=set, init=False, repr=False)
    """
    The names of RoboCasa's geoms our robot's fingertips rest on right now.
    """

    def __post_init__(self) -> None:
        self.mirror = RoboCasaSimulationMirror(
            environment=self.environment, world=self.world
        )

    @contextmanager
    def pressing(self, *geom_names: str) -> Iterator[None]:
        """
        Have RoboCasa's contact checks find its gripper touching geoms while the context
        is entered.

        :param geom_names: The names of the geoms.
        """
        environment = self.environment
        gripper = environment.robots[0].gripper[RoboCasaRobotFrame.GRIPPER]
        check_contact = environment.check_contact

        def check_contact_with_presses(geoms_1: Any, geoms_2: Any = None) -> bool:
            if (
                geoms_1 is gripper
                and isinstance(geoms_2, str)
                and geoms_2 in self.pressed_geoms
            ):
                return True
            return check_contact(geoms_1, geoms_2)

        self.pressed_geoms.update(geom_names)
        environment.check_contact = check_contact_with_presses
        try:
            yield
        finally:
            environment.check_contact = check_contact
            self.pressed_geoms.difference_update(geom_names)

    def update_fixture_states(self) -> None:
        """
        Bring RoboCasa's simulation into the state of the world and have every fixture
        update its state from it, as RoboCasa does after each of its steps.
        """
        self.mirror.synchronize()
        for fixture in self.environment.fixtures.values():
            fixture.update_state(self.environment)

    def take_robocasas_robot_to_ours(self) -> None:
        """
        Put the frames of RoboCasa's robot that its checks read where our robot's
        counterparts are: the end effector site of its gripper on our robot's tool frame,
        and its base on our robot's base.

        Only the computed frames are moved, not the robot, so this holds until
        RoboCasa's simulation is next brought up to date.
        """
        model, data = self.environment.sim.model, self.environment.sim.data
        robocasa_robot = self.environment.robots[0]
        site = robocasa_robot.eef_site_id[RoboCasaRobotFrame.GRIPPER]
        data.site_xpos[site] = self.arm.end_effector.tool_frame.global_pose.to_np()[
            :3, 3
        ]
        base = model.body_name2id(RoboCasaRobotFrame.BASE)
        world_T_base = self.robot.root.global_pose.to_np()
        data.body_xpos[base] = world_T_base[:3, 3]
        data.body_xmat[base] = world_T_base[:3, :3].reshape(-1)

    def sees_contact(self, body: Body) -> bool:
        """
        :param body: The root body of an object, or of a loose part of a fixture.
        :return: Whether RoboCasa's simulation, brought into the state of the world,
            has the body touch anything but RoboCasa's own robot.
        """
        self.mirror.synchronize()
        model, data = self.environment.sim.model, self.environment.sim.data
        own = model.body_name2id(body.name.name)
        robocasa_robot_bodies = self.robocasa_robot_body_ids
        for contact in data.contact[: data.ncon]:
            first = model.geom_bodyid[contact.geom1]
            second = model.geom_bodyid[contact.geom2]
            if own in (first, second) and not robocasa_robot_bodies & {first, second}:
                return True
        return False

    @property
    def robocasa_robot_body_ids(self) -> set[int]:
        """
        The identifiers of the bodies of RoboCasa's own robot in its simulation.
        """
        model = self.environment.sim.model
        return {
            model.body_name2id(name)
            for name in model.body_names
            if (name or "").startswith(tuple(RoboCasaRobotBodyPrefix))
        }

    def judged_objects(self) -> list[JudgedObject]:
        """
        :return: Every object of the task as RoboCasa's simulation holds it right now.
        """
        model, data = self.environment.sim.model, self.environment.sim.data
        robocasa_robot = self.environment.robots[0]
        gripper = data.site_xpos[robocasa_robot.eef_site_id[RoboCasaRobotFrame.GRIPPER]]
        touching: dict[str, set[str]] = {}
        for contact in data.contact[: data.ncon]:
            first = model.body_id2name(model.geom_bodyid[contact.geom1])
            second = model.body_id2name(model.geom_bodyid[contact.geom2])
            touching.setdefault(first, set()).add(second)
            touching.setdefault(second, set()).add(first)
        judged = []
        for name, robocasa_object in self.environment.objects.items():
            position = data.body_xpos[self.environment.obj_body_id[name]]
            judged.append(
                JudgedObject(
                    name=name,
                    position=position.tolist(),
                    distance_to_gripper=float(np.linalg.norm(gripper - position)),
                    touched_bodies=sorted(
                        touching.get(robocasa_object.root_body, set())
                    ),
                )
            )
        return judged

    def judged_joint_positions(self) -> list[JudgedJointPosition]:
        """
        :return: Every hinge and slide joint of the kitchen, without RoboCasa's robot,
            at the position RoboCasa's simulation holds it at right now.
        """
        model, data = self.environment.sim.model, self.environment.sim.data
        robocasa_robot_bodies = self.robocasa_robot_body_ids
        one_coordinate_joints = {
            mujoco.mjtJoint.mjJNT_HINGE,
            mujoco.mjtJoint.mjJNT_SLIDE,
        }
        return [
            JudgedJointPosition(
                joint_name=model.joint_id2name(joint),
                position=float(data.qpos[model.jnt_qposadr[joint]]),
            )
            for joint in range(model.njnt)
            if model.jnt_type[joint] in one_coordinate_joints
            and model.jnt_bodyid[joint] not in robocasa_robot_bodies
        ]

    def pass_judgement(self) -> Judgement:
        """
        :return: RoboCasa's verdict on the world as it stands.
        """
        self.update_fixture_states()
        self.take_robocasas_robot_to_ours()
        return Judgement(
            accomplished=bool(self.environment._check_success()),
            judged_objects=self.judged_objects(),
            judged_joint_positions=self.judged_joint_positions(),
        )
