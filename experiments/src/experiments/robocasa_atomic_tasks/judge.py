"""
RoboCasa's own simulation as the judge of an episode: never stepped, but brought into
the state of the world the PR2 acts in, with RoboCasa's robot put where the PR2 is.
"""

from __future__ import annotations

import logging
from contextlib import contextmanager
from dataclasses import dataclass, field

import numpy as np
from typing_extensions import TYPE_CHECKING, Any, Iterator

from experiments.robocasa_atomic_tasks.robocasa_names import (
    RoboCasaRobotBodyPrefix,
    RoboCasaRobotFrame,
)
from experiments.robocasa_goals.simulation_mirror import RoboCasaSimulationMirror
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from robocasa.environments.kitchen.kitchen import Kitchen

logger = logging.getLogger(__name__)


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
    The world the PR2 acts in.
    """

    robot: PR2
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
    The names of RoboCasa's geoms the PR2's fingertips rest on right now.
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

    def take_robocasas_robot_to_the_pr2(self) -> None:
        """
        Put the frames of RoboCasa's robot that its checks read where the PR2's
        counterparts are: the end effector site of its gripper on the PR2's tool frame,
        and its base on the PR2's base.

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
        robocasa_robot_bodies = {
            model.body_name2id(name)
            for name in model.body_names
            if (name or "").startswith(tuple(RoboCasaRobotBodyPrefix))
        }
        for contact in data.contact[: data.ncon]:
            first = model.geom_bodyid[contact.geom1]
            second = model.geom_bodyid[contact.geom2]
            if own in (first, second) and not robocasa_robot_bodies & {first, second}:
                return True
        return False

    def log_judgement(self, logged_contacts: int = 6) -> None:
        """
        Log what RoboCasa's simulation has every object of the task touch, and how far
        the gripper stands from it.

        :param logged_contacts: How many of the bodies an object touches are logged at
            most.
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
        for name, robocasa_object in self.environment.objects.items():
            position = data.body_xpos[self.environment.obj_body_id[name]]
            logger.info(
                "Judged %s at %s, %.2f m from the gripper, touching %s",
                name,
                np.round(position, 3).tolist(),
                float(np.linalg.norm(gripper - position)),
                sorted(touching.get(robocasa_object.root_body, set()))[
                    :logged_contacts
                ],
            )

    def succeeded(self) -> bool:
        """
        :return: Whether RoboCasa's own check finds the task accomplished in the world
            as it stands.
        """
        self.update_fixture_states()
        self.take_robocasas_robot_to_the_pr2()
        return bool(self.environment._check_success())
