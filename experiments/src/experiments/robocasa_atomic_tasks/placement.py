"""
Where an object may be put down, read off RoboCasa's own description of the kitchen, and
how a released object comes to rest where nothing simulates gravity.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from functools import cached_property

import numpy as np
from typing_extensions import TYPE_CHECKING, Sequence

from experiments.robocasa_atomic_tasks.fixtures import KitchenFixtures
from experiments.robocasa_atomic_tasks.geometry import BodyBounds
from experiments.robocasa_atomic_tasks.judge import RoboCasaJudge
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from robocasa.environments.kitchen.kitchen import Kitchen
    from robocasa.models.fixtures import Fixture
    from robocasa.models.fixtures.counter import Counter

logger = logging.getLogger(__name__)


# %% the inside of a fixture


@dataclass
class FixtureRegion:
    """
    A box as RoboCasa states the inside of a fixture: a corner, and the three corners
    next to it along the box's width, depth and height.
    """

    corner: np.ndarray
    """
    The corner the others lie next to.
    """

    along_width: np.ndarray
    """
    The corner next to it along the box's width.
    """

    along_depth: np.ndarray
    """
    The corner next to it along the box's depth.
    """

    along_height: np.ndarray
    """
    The corner next to it along the box's height.
    """

    @classmethod
    def from_corners(cls, corners: Sequence[np.ndarray]) -> FixtureRegion:
        """
        :param corners: The four corners, in the order RoboCasa's
            ``get_int_sites`` gives them.
        :return: The region.
        """
        corner, along_width, along_depth, along_height = (
            np.asarray(point, dtype=float) for point in corners
        )
        return cls(
            corner=corner,
            along_width=along_width,
            along_depth=along_depth,
            along_height=along_height,
        )

    @property
    def height(self) -> float:
        """
        How far the box reaches up from its floor.
        """
        return float(self.along_height[2] - self.corner[2])


# %% spots to put an object down on


@dataclass
class PlacementSpots:
    """
    Finds the spots an object may be put down on, given as where the middle of its
    underside is to lie, in the world's root frame.
    """

    environment: Kitchen
    """
    The RoboCasa environment that describes the kitchen.
    """

    world: World
    """
    The world the object is put down in.
    """

    bounds: BodyBounds
    """
    The bounding boxes of the world's bodies.
    """

    fixtures: KitchenFixtures
    """
    The bodies of the kitchen's fixtures.
    """

    small_fixture_footprint: float = 0.5
    """
    The area, in square metres, below which a fixture's footprint counts as small enough
    for it to stand on a counter and stand in the way of an object put down.
    """

    rounding_tolerance: float = 1e-9
    """
    How far beyond the end of a row of spots its last spot may lie, in metres.
    """

    def in_region(
        self,
        region: FixtureRegion,
        body: Body,
        spacing: float = 0.04,
        margin: float = 0.02,
        ignored: Sequence[Body] = (),
        height_tolerance: float = 0.05,
        dropped_in: bool = False,
    ) -> list[np.ndarray]:
        """
        :param region: The inside of a fixture.
        :param body: The object to be put down inside it.
        :param spacing: How far apart the spots lie, in metres.
        :param margin: How far the object stays from the sides of the region, in
            metres.
        :param ignored: Bodies the object may overlap, see :meth:`is_free`.
        :param height_tolerance: How far the object may stick out of the top of the
            region, in metres.
        :param dropped_in: Whether the spots lie at the top of the region rather than on
            its floor, for an object let go of above a deep fixture such as a sink.
        :return: The free spots in the region where the object fits, horizontally and in
            height, the ones nearest the middle of the region's front first.
        """
        box = self.bounds.of_branch(body)
        half_footprint = np.max(box.upper[:2] - box.lower[:2]) / 2 + margin
        object_height = box.upper[2] - box.lower[2]
        width_axis = region.along_width - region.corner
        depth_axis = region.along_depth - region.corner
        width, depth = np.linalg.norm(width_axis), np.linalg.norm(depth_axis)
        if (
            region.height + height_tolerance < object_height
            or width < 2 * half_footprint
            or depth < 2 * half_footprint
        ):
            return []
        width_axis, depth_axis = width_axis / width, depth_axis / depth
        floor = region.along_height if dropped_in else region.corner
        spots = [
            floor + across * width_axis + inwards * depth_axis
            for across in np.arange(
                half_footprint,
                width - half_footprint + self.rounding_tolerance,
                spacing,
            )
            for inwards in np.arange(
                half_footprint,
                depth - half_footprint + self.rounding_tolerance,
                spacing,
            )
        ]
        front_middle = floor + width_axis * width / 2
        spots.sort(key=lambda spot: np.linalg.norm((spot - front_middle)[:2]))
        return [spot for spot in spots if self.is_free(spot, body, ignored=ignored)]

    def inside(
        self,
        fixture: Fixture,
        body: Body,
        region_names: Sequence[str] | None = None,
        margin: float = 0.02,
        dropped_in: bool = False,
    ) -> list[np.ndarray]:
        """
        :param fixture: One of RoboCasa's fixtures.
        :param body: The object to be put down inside it.
        :param region_names: Which of the fixture's inside regions to use; all if not
            given.
        :param margin: See :meth:`in_region`.
        :param dropped_in: See :meth:`in_region`.
        :return: Spots inside the fixture, region after region, lowest region first.
        """
        regions = fixture.get_int_sites(relative=False)
        names = [
            name
            for name in sorted(regions, key=lambda name: regions[name][0][2])
            if region_names is None or name in region_names
        ]
        ignored = self.fixtures.bodies_of(fixture)
        return [
            spot
            for name in names
            for spot in self.in_region(
                FixtureRegion.from_corners(regions[name]),
                body,
                margin=margin,
                ignored=ignored,
                dropped_in=dropped_in,
            )
        ]

    def on_counter(
        self,
        counter: Counter,
        body: Body,
        near: np.ndarray,
        spacing: float = 0.05,
        margin: float = 0.03,
        number: int = 12,
    ) -> list[np.ndarray]:
        """
        :param counter: One of RoboCasa's counters.
        :param body: The object to be put down on it.
        :param near: The point the spots are to lie near.
        :param spacing: How far apart the spots lie, in metres.
        :param margin: How far the object stays from the sides of the regions RoboCasa
            places objects in, in metres.
        :param number: How many spots are returned at most.
        :return: Free spots in the regions of the counter's top RoboCasa places objects
            in, nearest ``near`` first.
        """
        box = self.bounds.of_branch(body)
        half_footprint = np.max(box.upper[:2] - box.lower[:2]) / 2 + margin
        rotation = np.array(counter.rot) if np.ndim(counter.rot) else counter.rot
        yaw = float(rotation) if np.ndim(rotation) == 0 else float(rotation[2])
        world_R_counter = np.array(
            [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
        )
        spots = []
        for region in counter.get_reset_regions(self.environment).values():
            center = np.asarray(counter.pos, dtype=float) + np.append(
                world_R_counter @ np.asarray(region["offset"][:2]), region["offset"][2]
            )
            half_size = np.asarray(region["size"]) / 2 - half_footprint
            if np.any(half_size <= 0):
                continue
            spots.extend(
                center + np.append(world_R_counter @ [across, inwards], 0.0)
                for across in np.arange(
                    -half_size[0], half_size[0] + self.rounding_tolerance, spacing
                )
                for inwards in np.arange(
                    -half_size[1], half_size[1] + self.rounding_tolerance, spacing
                )
            )
        spots.sort(key=lambda spot: np.linalg.norm((spot - near)[:2]))
        counter_bodies = self.fixtures.bodies_of(counter)
        free = []
        for spot in spots:
            if len(free) >= number:
                break
            if self.is_free(spot, body, ignored=counter_bodies):
                free.append(spot)
        return free

    def on_receptacle(
        self,
        receptacle: Body,
        share: float = 0.25,
        number_around_the_middle: int = 6,
    ) -> list[np.ndarray]:
        """
        :param receptacle: An object other objects are put on or in, such as a plate,
            a bowl or a pan.
        :param share: How far from its middle the spots may lie, as a share of the
            receptacle's horizontal radius.
        :param number_around_the_middle: How many spots lie around the middle.
        :return: Spots above the receptacle's rim, its middle first.
        """
        box = self.bounds.of_branch(receptacle)
        middle = np.append((box.lower[:2] + box.upper[:2]) / 2, box.upper[2])
        radius = share * np.min(box.upper[:2] - box.lower[:2]) / 2
        return [middle] + [
            middle + radius * np.array([np.cos(angle), np.sin(angle), 0.0])
            for angle in np.linspace(
                0, 2 * np.pi, number_around_the_middle, endpoint=False
            )
        ]

    def is_free(
        self,
        spot: np.ndarray,
        body: Body,
        clearance: float = 0.01,
        ignored: Sequence[Body] = (),
    ) -> bool:
        """
        :param spot: Where the middle of the object's underside is to lie.
        :param body: The object.
        :param clearance: How far above the spot the object is held when tested.
        :param ignored: Bodies the object may overlap, such as those of the fixture it
            is put into, whose bounding boxes enclose their inside.
        :return: Whether the object's bounding box at the spot overlaps that of none of
            the :attr:`possible_obstructions` but those ignored.
        """
        box = self.bounds.of_branch(body)
        size = box.upper - box.lower
        spot_lower = np.asarray(spot) + np.array(
            [-size[0] / 2, -size[1] / 2, clearance]
        )
        spot_upper = spot_lower + size
        skipped = (
            {body}
            | set(
                self.world.compute_descendent_child_kinematic_structure_entities(body)
            )
            | set(ignored)
        )
        for other in self.possible_obstructions:
            if other in skipped:
                continue
            other_box = self.bounds.of_branch(other)
            if np.all(other_box.lower < spot_upper) and np.all(
                other_box.upper > spot_lower
            ):
                return False
        return True

    @cached_property
    def possible_obstructions(self) -> list[Body]:
        """
        The bodies that may stand where an object is to be put down: those of the task's
        objects, and of the fixtures small enough to stand on a counter, such as a
        toaster.
        """
        object_bodies = []
        for robocasa_object in self.environment.objects.values():
            root = self.world.get_body_by_name(robocasa_object.root_body)
            object_bodies.extend(
                part
                for part in [
                    root,
                    *self.world.compute_descendent_child_kinematic_structure_entities(
                        root
                    ),
                ]
                if isinstance(part, Body) and part.has_collision()
            )
        small_fixture_bodies = []
        for fixture in self.environment.fixtures.values():
            bodies = self.fixtures.bodies_of(fixture)
            if not bodies:
                continue
            boxes = [self.bounds.of_branch(body) for body in bodies]
            lower = np.min([box.lower for box in boxes], axis=0)
            upper = np.max([box.upper for box in boxes], axis=0)
            if np.prod(upper[:2] - lower[:2]) < self.small_fixture_footprint:
                small_fixture_bodies.extend(bodies)
        return object_bodies + small_fixture_bodies

    def pose_with_underside_at(
        self, body: Body, spot: np.ndarray, clearance: float = 0.01
    ) -> Pose:
        """
        :param body: An object.
        :param spot: Where the middle of its underside is to lie.
        :param clearance: How far above the spot the underside is held when the object
            is let go of, in metres.
        :return: The pose of the object, oriented as it is now, that puts the middle of
            its underside ``clearance`` above the spot.
        """
        box = self.bounds.of_branch(body)
        world_T_body = body.global_pose.to_np()
        underside_middle = np.append((box.lower[:2] + box.upper[:2]) / 2, box.lower[2])
        offset = world_T_body[:3, 3] - underside_middle
        world_T_target = world_T_body.copy()
        world_T_target[:3, 3] = np.asarray(spot) + offset + np.array([0, 0, clearance])
        return HomogeneousTransformationMatrix(
            world_T_target, reference_frame=self.world.root
        ).pose


# %% coming to rest without gravity


@dataclass
class KinematicSettling:
    """
    Lets a released object come to rest on what is below it where nothing simulates
    gravity: lowers it until it touches something, in the world and in RoboCasa's own
    model, so that RoboCasa finds the two in contact.
    """

    world: World
    """
    The world the object rests in.
    """

    robot: AbstractRobot
    """
    The robot, which the object does not rest on.
    """

    bounds: BodyBounds
    """
    The bounding boxes of the world's bodies.
    """

    judge: RoboCasaJudge
    """
    RoboCasa's simulation, which has to see the object touch what it rests on.
    """

    def settle(
        self,
        body: Body,
        step: float = 0.003,
        maximum_drop: float = 0.4,
        maximum_judge_gap: float = 0.03,
        overlap_margin: float = 0.01,
    ) -> None:
        """
        Lower a released object until it touches what is below it; an object touching
        something already, or still held by the robot, stays where it is.

        :param body: The object.
        :param step: How far the object moves at a time, in metres.
        :param maximum_drop: How far the object is lowered at most, in metres.
        :param maximum_judge_gap: How much further the object is lowered at most for
            RoboCasa's own model to see it touch what it rests on, in metres.
        :param overlap_margin: How far beside the object horizontally a body may begin
            and still count as below it, in metres.
        """
        if body in self.world.get_kinematic_structure_entities_of_branch(
            self.robot.root
        ):
            return
        detector = self.world.collision_manager.collision_detector
        own = {body} | {
            part
            for part in self.world.compute_descendent_child_kinematic_structure_entities(
                body
            )
            if isinstance(part, Body)
        }
        own_with_collision = [part for part in own if part.has_collision()]
        others = self.bodies_below(body, own, maximum_drop, overlap_margin)

        def touches() -> bool:
            detector.sync_world_state()
            return any(
                detector.check_collision_between_bodies(part, other, 0.0) is not None
                for part in own_with_collision
                for other in others
            )

        dropped = 0.0
        while not touches() and dropped < maximum_drop:
            self.lower(body, step)
            dropped += step
        further = 0.0
        while not self.judge.sees_contact(body) and further < maximum_judge_gap:
            self.lower(body, step)
            further += step
        logger.info(
            "Settled %s: lowered %.3f m, and %.3f m more for RoboCasa to see contact",
            body.name,
            dropped,
            further,
        )

    def bodies_below(
        self,
        body: Body,
        own: set[Body],
        maximum_drop: float,
        overlap_margin: float,
    ) -> list[Body]:
        """
        :param body: An object.
        :param own: The bodies of the object.
        :param maximum_drop: How far below the object a body may begin, in metres.
        :param overlap_margin: How far beside the object horizontally a body may begin,
            in metres.
        :return: The bodies with collision geometry, but the robot's and the object's,
            the object may come to rest on.
        """
        box = self.bounds.of_branch(body)
        robot_bodies = set(self.robot.bodies)
        below = []
        for other in self.world.bodies_with_collision:
            if other in own or other in robot_bodies:
                continue
            other_box = self.bounds.of_branch(other)
            if (
                np.all(other_box.lower[:2] <= box.upper[:2] + overlap_margin)
                and np.all(other_box.upper[:2] >= box.lower[:2] - overlap_margin)
                and other_box.upper[2] >= box.lower[2] - maximum_drop
                and other_box.lower[2] <= box.upper[2]
            ):
                below.append(other)
        return below

    @staticmethod
    def lower(body: Body, height: float) -> None:
        """
        :param body: An object hanging from a connection of its own.
        :param height: How far it is lowered, in metres.
        """
        connection = body.parent_connection
        origin = connection.origin.to_np()
        origin[2, 3] -= height
        connection.origin = HomogeneousTransformationMatrix(
            origin, reference_frame=connection.parent
        )
