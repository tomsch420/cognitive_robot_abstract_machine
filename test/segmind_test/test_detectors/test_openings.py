"""
Tests for detecting a drawer or door being opened and being closed.
"""

from dataclasses import dataclass

import pytest
from giskardpy.motion_statechart.context import MotionStatechartContext
from segmind.datastructures.events import ClosedEvent, OpenedEvent, PartlyOpenEvent
from segmind.detectors.base import SegmindContext
from segmind.detectors.mechanism_detector_nodes import OpeningDetector
from segmind.episode_segmenter import EpisodeSegmenterExecutor
from segmind.statecharts.segmind_statechart import SegmindStatechart
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import Vector3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    PrismaticConnection,
)
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedom,
    DegreeOfFreedomLimits,
)
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% a drawer on its rails


@dataclass
class DrawerOnRails:
    """
    A world with a cabinet and a drawer sliding out of it, and the detector watching the
    drawer.
    """

    world: World
    """
    The world.
    """

    rails: PrismaticConnection
    """
    The joint the drawer slides on.
    """

    detector: OpeningDetector
    """
    The detector watching the drawer.
    """

    executor: EpisodeSegmenterExecutor
    """
    Ticks the detector.
    """

    def slide_to(self, position: float) -> None:
        """
        Put the drawer at ``position`` along its rails and tick the detector.
        """
        self.rails.position = position
        self.executor.tick()

    def events(self) -> list:
        """
        :return: Every event the detector reported, in order.
        """
        return self.executor.context.require_extension(
            SegmindContext
        ).logger.get_events()


def drawer_on_rails(starting_position: float) -> DrawerOnRails:
    """
    :param starting_position: Where along its rails the drawer stands when the detector
        is first ticked.
    """
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        world.add_body(root)
        cabinet = Body(
            name=PrefixedName("cabinet"),
            collision=ShapeCollection([Box(scale=Scale(0.5, 0.5, 0.5))]),
        )
        world.add_connection(FixedConnection(parent=root, child=cabinet))
        drawer = Body(
            name=PrefixedName("drawer"),
            collision=ShapeCollection([Box(scale=Scale(0.4, 0.4, 0.1))]),
        )
        degree_of_freedom = DegreeOfFreedom(
            name=PrefixedName("rails"),
            limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                lower_position=0.0, upper_position=0.4, maximum_speed=1.0
            ),
        )
        world.add_degree_of_freedom(degree_of_freedom)
        rails = PrismaticConnection(
            name=PrefixedName("rails"),
            parent=cabinet,
            child=drawer,
            axis=Vector3.X(reference_frame=drawer),
            raw_dof=degree_of_freedom,
        )
        world.add_connection(rails)
    rails.position = starting_position
    detector = OpeningDetector(tracked_object=drawer)
    executor = EpisodeSegmenterExecutor(context=MotionStatechartContext(world=world))
    executor.compile(SegmindStatechart().build_statechart([detector]))
    executor.tick()
    return DrawerOnRails(world, rails, detector, executor)


def test_a_drawer_pulled_out_past_its_open_ratio_is_reported_opened():
    drawer = drawer_on_rails(starting_position=0.0)
    full_travel = drawer.rails.dof.limits.upper.position

    drawer.slide_to(drawer.detector.open_ratio * full_travel + 0.01)

    [opened] = drawer.events()
    assert type(opened) is OpenedEvent
    assert opened.tracked_object is drawer.rails.child


def test_a_drawer_pushed_in_past_its_closed_ratio_is_reported_closed():
    drawer = drawer_on_rails(starting_position=0.2)
    full_travel = drawer.rails.dof.limits.upper.position

    drawer.slide_to(drawer.detector.closed_ratio * full_travel - 0.01)

    [closed] = drawer.events()
    assert type(closed) is ClosedEvent
    assert closed.tracked_object is drawer.rails.child


def test_a_drawer_that_starts_open_is_reported_only_once_it_changes():
    """
    The state a drawer is first seen in is where the run starts, not something that
    happened in it.
    """
    drawer = drawer_on_rails(starting_position=0.4)

    drawer.slide_to(0.39)

    assert drawer.events() == []


def test_a_closed_drawer_pulled_partly_out_is_reported_partly_open():
    drawer = drawer_on_rails(starting_position=0.0)

    drawer.slide_to(0.2)

    [partly_open] = drawer.events()
    assert type(partly_open) is PartlyOpenEvent
    assert partly_open.tracked_object is drawer.rails.child


def test_an_open_drawer_pushed_partly_in_is_reported_partly_open():
    drawer = drawer_on_rails(starting_position=0.4)

    drawer.slide_to(0.2)

    [partly_open] = drawer.events()
    assert type(partly_open) is PartlyOpenEvent
    assert partly_open.tracked_object is drawer.rails.child


def test_a_drawer_moved_within_partly_open_is_not_reported():
    drawer = drawer_on_rails(starting_position=0.2)

    drawer.slide_to(0.25)

    assert drawer.events() == []


def test_a_body_without_a_joint_is_never_reported():
    drawer = drawer_on_rails(starting_position=0.0)
    cabinet = drawer.rails.parent
    detector = OpeningDetector(tracked_object=cabinet)
    executor = EpisodeSegmenterExecutor(
        context=MotionStatechartContext(world=drawer.world)
    )
    executor.compile(SegmindStatechart().build_statechart([detector]))

    executor.tick()
    executor.tick()

    assert executor.context.require_extension(SegmindContext).logger.get_events() == []
