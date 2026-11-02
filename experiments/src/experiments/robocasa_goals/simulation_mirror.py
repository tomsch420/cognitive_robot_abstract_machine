"""
Keeping RoboCasa's own simulation in the state of the world a robot acts in, so that
RoboCasa's checks judge that world.

The world is parsed from the MJCF document RoboCasa composed, so its bodies and joints
carry RoboCasa's names. RoboCasa's own robot stays in its simulation where RoboCasa
started it and is never moved.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
from typing_extensions import TYPE_CHECKING

from experiments.robocasa_goals.exceptions import (
    NotARoboCasaFixture,
    NotARoboCasaObject,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from robocasa.environments.kitchen.kitchen import Kitchen
    from robocasa.models.fixtures import Fixture

# %% the mirror


@dataclass
class RoboCasaSimulationMirror:
    """
    Copies the poses of the objects and of the loose parts of fixtures, and the
    positions of the fixture joints, of a world into the RoboCasa simulation it was
    parsed from.
    """

    environment: Kitchen
    """
    The RoboCasa environment whose simulation is kept in the state of :attr:`world`.
    """

    world: World
    """
    The world parsed from :attr:`environment`.
    """

    synchronized_state_version: int | None = field(default=None, init=False)
    """
    The version of the world's state last copied; ``None`` before the first copy.
    """

    @cached_property
    def fixture_joints(self) -> set[ActiveConnection1DOF]:
        """
        The connections of the world that move a joint of RoboCasa's fixtures.
        """
        joint_names = set(self.environment.sim.model.joint_names)
        return {
            connection
            for connection in self.world.connections
            if isinstance(connection, ActiveConnection1DOF)
            and connection.name.name in joint_names
        }

    @cached_property
    def free_fixture_joints(self) -> dict[str, str]:
        """
        The free joints of RoboCasa's simulation that move a loose part of a fixture
        rather than an object, such as the lid of a blender, each with the name of the
        body it moves, for the bodies the world has.
        """
        model = self.environment.sim.model
        object_bodies = {
            robocasa_object.root_body
            for robocasa_object in self.environment.objects.values()
        }
        world_bodies = {body.name.name for body in self.world.bodies}
        joints = {}
        for joint in range(model.njnt):
            if model.jnt_type[joint] != 0:
                continue
            body_name = model.body_id2name(model.jnt_bodyid[joint])
            if body_name in object_bodies or body_name not in world_bodies:
                continue
            joints[model.joint_id2name(joint)] = body_name
        return joints

    @cached_property
    def mirrored_joint_names(self) -> set[str]:
        """
        The names of the fixture joints the world holds mirrored, opening towards
        positive positions where RoboCasa's open towards negative ones: the loader
        mirrors every joint whose range runs from a negative position up to 0.
        """
        model = self.environment.sim.model
        mirrored = set()
        for connection in self.fixture_joints:
            name = connection.name.name
            lower, upper = model.jnt_range[model.joint_name2id(name)]
            if lower < upper == 0.0:
                mirrored.add(name)
        return mirrored

    def synchronize(self) -> None:
        """
        Put every object and fixture joint of RoboCasa's simulation where it is in the
        world, unless the world has not changed since the last copy.
        """
        if self.world.state.version == self.synchronized_state_version:
            return
        data = self.environment.sim.data
        for robocasa_object in self.environment.objects.values():
            world_T_object = self.world.get_body_by_name(
                robocasa_object.root_body
            ).global_pose
            x, y, z, w = world_T_object.quaternion.to_np()
            data.set_joint_qpos(
                robocasa_object.joints[0],
                np.concatenate([world_T_object.to_np()[:3, 3], [w, x, y, z]]),
            )
        for joint_name, body_name in self.free_fixture_joints.items():
            world_T_body = self.world.get_body_by_name(body_name).global_pose
            x, y, z, w = world_T_body.quaternion.to_np()
            data.set_joint_qpos(
                joint_name,
                np.concatenate([world_T_body.to_np()[:3, 3], [w, x, y, z]]),
            )
        for connection in self.fixture_joints:
            name = connection.name.name
            sign = -1.0 if name in self.mirrored_joint_names else 1.0
            data.set_joint_qpos(name, sign * connection.position)
        self.environment.sim.forward()
        self.synchronized_state_version = self.world.state.version

    def object_name_of(self, body: Body) -> str:
        """
        :param body: The root body of one of the task's objects.
        :return: The name RoboCasa gives the object.
        :raises NotARoboCasaObject: If ``body`` is the root of no object.
        """
        name = next(
            (
                name
                for name, robocasa_object in self.environment.objects.items()
                if robocasa_object.root_body == body.name.name
            ),
            None,
        )
        if name is None:
            raise NotARoboCasaObject(body=body)
        return name

    def fixture_of(self, body: Body) -> Fixture:
        """
        :param body: The root body of one of the kitchen's fixtures.
        :return: The fixture.
        :raises NotARoboCasaFixture: If ``body`` is the root of no fixture.
        """
        fixture = next(
            (
                fixture
                for fixture in self.environment.fixtures.values()
                if fixture.root_body == body.name.name
            ),
            None,
        )
        if fixture is None:
            raise NotARoboCasaFixture(body=body)
        return fixture
