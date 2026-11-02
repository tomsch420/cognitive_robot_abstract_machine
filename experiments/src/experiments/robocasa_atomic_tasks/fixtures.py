"""
The bodies of the kitchen's fixtures, and what the robot may touch while it acts on
them.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass

import numpy as np
from typing_extensions import TYPE_CHECKING, Iterator, Sequence

from experiments.robocasa_atomic_tasks.geometry import BodyBounds
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidCollisionBetweenGroups,
)
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import CoordinateBounds
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from robocasa.environments.kitchen.kitchen import Kitchen
    from robocasa.models.fixtures import Fixture

# %% the bodies of the fixtures


@dataclass
class KitchenFixtures:
    """
    The bodies of the fixtures RoboCasa composed the kitchen of, as the world holds
    them.
    """

    environment: Kitchen
    """
    The RoboCasa environment that names the fixtures.
    """

    world: World
    """
    The world holding their bodies.
    """

    bounds: BodyBounds
    """
    The bounding boxes of the world's bodies.
    """

    def bodies_of(self, fixture: Fixture) -> list[Body]:
        """
        :param fixture: One of RoboCasa's fixtures.
        :return: Every body of the fixture with collision geometry.
        """
        return [
            body
            for body in self.world.bodies_with_collision
            if body.name.name.startswith(fixture.naming_prefix)
        ]

    def bodies_near(self, box: CoordinateBounds, margin: float = 0.15) -> list[Body]:
        """
        :param box: A box in the world's root frame.
        :param margin: How far beyond the box a fixture's body may begin.
        :return: The bodies of the fixtures whose bounding boxes come within
            ``margin`` of the box.
        """
        prefixes = [
            fixture.naming_prefix for fixture in self.environment.fixtures.values()
        ]
        near = []
        for body in self.world.bodies_with_collision:
            if not any(body.name.name.startswith(prefix) for prefix in prefixes):
                continue
            body_box = self.bounds.of_branch(body)
            if np.all(body_box.lower <= box.upper + margin) and np.all(
                body_box.upper >= box.lower - margin
            ):
                near.append(body)
        return near

    def bodies_of_the_fixture_of(self, body: Body) -> list[Body]:
        """
        :param body: A body of one of the kitchen's fixtures, such as a knob.
        :return: Every body of that fixture with collision geometry; just ``body`` if
            it belongs to no fixture RoboCasa names.
        """
        prefixes = [
            fixture.naming_prefix
            for fixture in self.environment.fixtures.values()
            if body.name.name.startswith(fixture.naming_prefix)
        ]
        if not prefixes:
            return [body]
        prefix = max(prefixes, key=len)
        return [
            other
            for other in self.world.bodies_with_collision
            if other.name.name.startswith(prefix)
        ]

    def bodies_around(self, part: Body, margin: float = 0.3) -> list[Body]:
        """
        :param part: A part of one of the kitchen's fixtures, such as a knob.
        :param margin: How far from the part the bodies of other fixtures may begin.
        :return: The bodies of the part's fixture and of the fixtures near the part,
            such as the wall cabinet above a faucet.
        """
        near = self.bodies_near(self.bounds.of_branch(part), margin=margin)
        return list(dict.fromkeys(self.bodies_of_the_fixture_of(part) + near))


# %% what the robot may touch


@dataclass
class CollisionAllowances:
    """
    Changes, for a while, which bodies collision avoidance keeps the robot from.
    """

    world: World
    """
    The world whose collision rules are changed.
    """

    @contextmanager
    def bodies_may_touch(
        self, group_a: Sequence[Body], group_b: Sequence[Body]
    ) -> Iterator[None]:
        """
        Let every body of one group touch every body of the other while the context is
        entered.

        :param group_a: The bodies of one group.
        :param group_b: The bodies of the other.
        """
        rule = AllowCollisionBetweenGroups(
            body_group_a=list(group_a), body_group_b=list(group_b)
        )
        with self.rules_applied(ignore_rules=[rule]):
            yield

    @contextmanager
    def gripper_may_touch(
        self, arm: Arm, bodies: Sequence[Body], arm_buffer_zone_distance: float = 0.01
    ) -> Iterator[None]:
        """
        Let an arm's gripper touch some bodies, and the rest of the arm come nearly as
        close to them as it likes, while the context is entered; everything else is
        avoided as before.

        :param arm: The arm.
        :param bodies: The bodies, such as those of the fixture acted on.
        :param arm_buffer_zone_distance: How close to the bodies collision avoidance
            lets the arm come before pushing it away, in metres.
        """
        gripper_rule = AllowCollisionBetweenGroups(
            body_group_a=list(arm.end_effector.bodies_with_collision),
            body_group_b=list(bodies),
        )
        arm_rule = AvoidCollisionBetweenGroups(
            body_group_a=list(arm.bodies_with_collision),
            body_group_b=list(bodies),
            buffer_zone_distance=arm_buffer_zone_distance,
            violated_distance=0.0,
        )
        with self.rules_applied(ignore_rules=[gripper_rule], default_rules=[arm_rule]):
            yield

    @contextmanager
    def rules_applied(
        self,
        ignore_rules: Sequence[CollisionRule] = (),
        default_rules: Sequence[CollisionRule] = (),
    ) -> Iterator[None]:
        """
        Add collision rules while the context is entered.

        :param ignore_rules: Rules that let bodies collide.
        :param default_rules: Rules that set how far bodies are kept apart.
        """
        collision_manager = self.world.collision_manager
        with self.world.modify_world():
            for rule in default_rules:
                collision_manager.add_default_rule(rule)
            for rule in ignore_rules:
                collision_manager.add_ignore_collision_rule(rule)
        collision_manager.update_collision_matrix()
        try:
            yield
        finally:
            with self.world.modify_world():
                for rule in ignore_rules:
                    collision_manager.ignore_collision_rules.remove(rule)
                for rule in default_rules:
                    collision_manager.default_rules.remove(rule)
            collision_manager.update_collision_matrix()
