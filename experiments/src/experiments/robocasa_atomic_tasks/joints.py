"""
The joints of RoboCasa's fixtures, as RoboCasa states their positions and as the world
holds them, and the parts hanging from them.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from typing_extensions import TYPE_CHECKING

from experiments.robocasa_atomic_tasks.exceptions import NoRackAtLevel
from experiments.robocasa_atomic_tasks.geometry import BodyBounds
from experiments.robocasa_atomic_tasks.robocasa_names import RackKind
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
    KnobWithHandle,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    PrismaticConnection,
)
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from robocasa.environments.kitchen.kitchen import Kitchen
    from robocasa.models.fixtures import Fixture

# %% a joint's range


@dataclass
class JointRange:
    """
    The limits of one of RoboCasa's joints, as RoboCasa states them.
    """

    lower: float
    """
    The lower limit.
    """

    upper: float
    """
    The upper limit.
    """

    def at_fraction(self, fraction: float) -> float:
        """
        :param fraction: How far from the lower to the upper limit a position lies.
        :return: The position.
        """
        return self.lower + fraction * (self.upper - self.lower)

    def at_share(self, share: float) -> float:
        """
        :param share: How far along its travel the joint stands, as RoboCasa normalizes
            it for a door: 0 closed, 1 fully open.
        :return: The position, closed at the limit nearer to zero.
        """
        if self.lower < 0:
            return self.upper - share * (self.upper - self.lower)
        return self.at_fraction(share)

    @property
    def mirrored(self) -> bool:
        """
        Whether the world holds the joint mirrored, opening towards positive positions
        where RoboCasa's opens towards negative ones.
        """
        return self.lower < self.upper == 0.0


# %% the joints of the kitchen


@dataclass
class FixtureJoints:
    """
    The joints of the kitchen's fixtures, by the names RoboCasa gives them.
    """

    environment: Kitchen
    """
    The RoboCasa environment, which states the joints' ranges.
    """

    world: World
    """
    The world holding the joints.
    """

    def connection(self, name: str) -> ActiveConnection1DOF:
        """
        :param name: The name RoboCasa gives one of its fixtures' joints.
        :return: The joint, as it is in the world.
        """
        return next(
            connection
            for connection in self.world.connections
            if isinstance(connection, ActiveConnection1DOF)
            and connection.name.name == name
        )

    def range_of(self, name: str) -> JointRange:
        """
        :param name: The name of one of RoboCasa's joints.
        :return: Its range, as RoboCasa states it.
        """
        model = self.environment.sim.model
        lower, upper = model.jnt_range[model.joint_name2id(name)]
        return JointRange(lower=float(lower), upper=float(upper))

    def world_position_of(self, name: str, robocasa_position: float) -> float:
        """
        :param name: The name of one of RoboCasa's joints.
        :param robocasa_position: A position of the joint as RoboCasa states it.
        :return: The same position as the world states it.
        """
        if self.range_of(name).mirrored:
            return -robocasa_position
        return robocasa_position

    def world_position_at_share(self, name: str, share: float) -> float:
        """
        :param name: The name of one of RoboCasa's joints.
        :param share: See :meth:`JointRange.at_share`.
        :return: The position of the joint at that share, as the world states it.
        """
        return self.world_position_of(name, self.range_of(name).at_share(share))

    def world_position_along_range(self, name: str, fraction: float) -> float:
        """
        :param name: The name of one of RoboCasa's joints.
        :param fraction: How far along the joint's range from its lower to its upper
            limit, as RoboCasa states them, a position lies, as RoboCasa normalizes a
            knob or a faucet handle.
        :return: The position, as the world states it.
        """
        return self.world_position_of(name, self.range_of(name).at_fraction(fraction))

    def rack_joint_name(self, fixture: Fixture, level: int) -> str:
        """
        :param fixture: An oven or a toaster oven.
        :param level: The level of the rack RoboCasa refers to, 0 at the bottom.
        :return: The name of the joint the rack or tray at that level slides on, or of
            the bottom one's if the fixture has none at that level, as RoboCasa falls
            back to it.
        :raises NoRackAtLevel: If the fixture has neither.
        """
        joint_names = set(self.environment.sim.model.joint_names)
        for candidate_level in (level, 0):
            for kind in RackKind:
                name = kind.joint_name(fixture.naming_prefix, candidate_level)
                if name in joint_names:
                    return name
        raise NoRackAtLevel(fixture_name=fixture.name, level=level)

    # %% parts hanging from the joints

    def part_moved_by(self, name: str) -> Handle:
        """
        :param name: The name RoboCasa gives one of its fixtures' joints.
        :return: The part hanging from the joint, annotated as what it is taken hold of
            by: its handle where it has one, otherwise the part itself.
        """
        connection = self.connection(name)
        part = connection.child
        if not part.has_collision():
            part = next(
                body
                for body in self.world.compute_descendent_child_kinematic_structure_entities(
                    part
                )
                if isinstance(body, Body) and body.has_collision()
            )
        handle = next(
            (
                handle
                for handle in self.world.get_semantic_annotations_by_type(Handle)
                if handle.root.has_collision()
                and (
                    handle.root is part
                    or BodyBounds.joint_above(handle.root) is connection
                )
            ),
            None,
        )
        if handle is not None:
            return handle
        handle = Handle(root=part)
        with self.world.modify_world():
            self.world.add_semantic_annotation(handle)
        return handle

    def knob_turned_by(self, joint: ActiveConnection1DOF) -> KnobWithHandle | None:
        """
        :param joint: A joint of the kitchen.
        :return: The knob the joint turns, if it is one turned by a bar on its face.
        """
        return next(
            (
                knob
                for knob in self.world.get_semantic_annotations_by_type(KnobWithHandle)
                if knob.turning_joint is joint
            ),
            None,
        )

    # %% moving a joint

    @staticmethod
    def travel_of(joint: ActiveConnection1DOF) -> float:
        """
        :param joint: A joint of the world.
        :return: The distance between its limits.
        """
        limits = joint.dof.limits
        return limits.upper.position - limits.lower.position

    @staticmethod
    def within_limits(joint: ActiveConnection1DOF, position: float) -> float:
        """
        :param joint: A joint of the world.
        :param position: A position of it.
        :return: The position, moved within the joint's limits.
        """
        limits = joint.dof.limits
        return float(np.clip(position, limits.lower.position, limits.upper.position))

    def push_direction(
        self,
        joint: ActiveConnection1DOF,
        world_position: float,
        point: np.ndarray | None = None,
        probing_distance: float = 1e-3,
    ) -> np.ndarray | None:
        """
        :param joint: A joint of the world.
        :param world_position: The position it is to be pushed to.
        :param point: A point on the part the joint moves, in the world's root frame.
        :param probing_distance: How far the joint is moved to see which way the point
            goes.
        :return: The direction the joint moves ``point`` in on the way there, or its
            part if it slides it; ``None`` if it turns the part and no point is given.
        """
        if point is None and not isinstance(joint, PrismaticConnection):
            return None
        world_T_part = joint.child.global_pose.to_np()
        before = (
            world_T_part[:3, 3] if point is None else np.asarray(point, dtype=float)
        )
        part_point = np.linalg.inv(world_T_part) @ np.append(before, 1.0)
        towards = 1.0 if world_position >= joint.position else -1.0
        with self.world.reset_state_context():
            joint.position = joint.position + probing_distance * towards
            after = (joint.child.global_pose.to_np() @ part_point)[:3]
        return (after - before) / np.linalg.norm(after - before)

    def snap_to_end_stop(
        self, joint: ActiveConnection1DOF, end_stop: float, share: float = 0.1
    ) -> None:
        """
        Let a switch or lever pushed close to the end of its travel snap there, as a
        real one's detent does.

        :param joint: The joint pushed.
        :param end_stop: The end of its travel it was pushed towards.
        :param share: How close to the end, as a share of the travel, the joint has to
            come to snap there.
        """
        if abs(joint.position - end_stop) <= share * self.travel_of(joint):
            joint.position = end_stop
