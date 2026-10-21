"""
The HSR performing RoboCasa's pick-and-place tasks in a RoboCasa kitchen lowered to
its height, simulated in MuJoCo with the object held by friction alone.
"""

import os
import pytest

pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.hsr_robocasa.pick_place import (
    HSRPickPlaceTask,
    PickPlaceCounterToSink,
    PickPlaceCounterToStove,
    PickPlaceDrawerToCounter,
    PickPlaceSinkToCounter,
    PickPlaceStoveToCounter,
    TaskObject,
)
from experiments.hsr_robocasa.robocasa_task import (
    FixtureType,
    HSRBArmPlane,
    LayoutKey,
    RoboCasaLayout,
    RowOffTheFloor,
)
from robocasa.models.scenes.scene_registry import LayoutType
from segmind.datastructures.events import LossOfGraspEvent, PlacingEvent
from semantic_digital_twin.robots.hsrb import HSRBJoint

runs_in_ci = pytest.mark.skipif(
    os.environ.get("CI", "false").lower() == "false",
    reason="MuJoCo tests only run in CI",
)

REACHED_POSE_TOLERANCE = 0.005
"""
How far, in metres, the tool frame may end up from a pose the arm plane is asked to
reach: the resolution of its searches.
"""


@pytest.fixture(
    scope="module",
    params=[
        PickPlaceCounterToStove,
        PickPlaceStoveToCounter,
        PickPlaceCounterToSink,
        PickPlaceSinkToCounter,
        PickPlaceDrawerToCounter,
    ],
)
def performed_task(request) -> HSRPickPlaceTask:
    """
    A pick-and-place task, performed in MuJoCo.
    """
    experiment = request.param(headless=True)
    experiment.build_world()
    experiment.run()
    return experiment


# %% the lowered kitchen


def test_the_counters_stand_at_the_height_the_kitchen_is_lowered_to():
    experiment = PickPlaceCounterToStove()
    layout = RoboCasaLayout.from_layout_type(experiment.layout)

    lowered = layout.lowered_fixtures(experiment.counter_height)

    counters = [
        fixture[LayoutKey.NAME]
        for group in layout.configuration.values()
        for row in group.values()
        if isinstance(row, list)
        for fixture in row
        if fixture.get(LayoutKey.TYPE) == FixtureType.COUNTER
    ]
    assert [lowered[counter][LayoutKey.SIZE][2] for counter in counters] == (
        pytest.approx([experiment.counter_height] * len(counters))
    )


def test_a_kitchen_island_is_lowered_with_the_counters():
    """
    A layout with a group of its own, such as an island, also states where the group
    stands; that is no fixture to lower, while the island's own fixtures are.
    """
    layout = RoboCasaLayout.from_layout_type(LayoutType.LAYOUT002)

    lowered = layout.lowered_fixtures(PickPlaceCounterToStove().counter_height)

    island = [
        fixture[LayoutKey.NAME]
        for row in layout.configuration[LayoutKey.ISLAND_GROUP].values()
        if isinstance(row, list)
        for fixture in row
        if isinstance(fixture, dict) and LayoutKey.SIZE in fixture
    ]
    assert island
    assert [name for name in island if name not in lowered] == []


def test_what_hangs_on_the_walls_keeps_its_height():
    experiment = PickPlaceCounterToStove()
    layout = RoboCasaLayout.from_layout_type(experiment.layout)

    lowered = layout.lowered_fixtures(experiment.counter_height)

    on_the_walls = [
        fixture[LayoutKey.NAME]
        for group in layout.configuration.values()
        for row_name, row in group.items()
        if row_name in RowOffTheFloor and isinstance(row, list)
        for fixture in row
    ]
    assert [name for name in on_the_walls if name in lowered] == []


# %% the arm plane


def test_the_arm_plane_puts_the_tool_frame_where_a_configuration_of_it_does():
    arm_plane = HSRBArmPlane()
    tool_frame = arm_plane.robot.all_arms[0].end_effector.tool_frame
    for joint, position in {
        HSRBJoint.ARM_LIFT: 0.4,
        HSRBJoint.ARM_FLEX: -0.8,
        HSRBJoint.WRIST_FLEX: -1.2,
    }.items():
        arm_plane.world.get_connection_by_name(joint).position = position
    base_T_tool = arm_plane.world.compute_forward_kinematics_np(
        arm_plane.robot.root, tool_frame
    )

    for joint, position in arm_plane.joint_positions_for(base_T_tool).items():
        arm_plane.world.get_connection_by_name(joint).position = position
    reached = arm_plane.world.compute_forward_kinematics_np(
        arm_plane.robot.root, tool_frame
    )

    assert reached[:3, 3] == pytest.approx(
        base_T_tool[:3, 3], abs=REACHED_POSE_TOLERANCE
    )


def test_the_arm_plane_turns_the_wrist_to_the_goals_roll_where_it_puts_the_tool_frame():
    arm_plane = HSRBArmPlane()
    tool_frame = arm_plane.robot.all_arms[0].end_effector.tool_frame
    for joint, position in {
        HSRBJoint.ARM_LIFT: 0.4,
        HSRBJoint.ARM_FLEX: -0.8,
        HSRBJoint.WRIST_FLEX: -1.2,
        HSRBJoint.WRIST_ROLL: 0.7,
    }.items():
        arm_plane.world.get_connection_by_name(joint).position = position
    base_T_tool = arm_plane.world.compute_forward_kinematics_np(
        arm_plane.robot.root, tool_frame
    )

    for joint, position in arm_plane.joint_positions_for(base_T_tool).items():
        arm_plane.world.get_connection_by_name(joint).position = position
    reached = arm_plane.world.compute_forward_kinematics_np(
        arm_plane.robot.root, tool_frame
    )

    assert reached == pytest.approx(base_T_tool, abs=REACHED_POSE_TOLERANCE)


# %% the run


@runs_in_ci
def test_segmind_sees_the_object_put_down_in_the_receptacle(
    performed_task: HSRPickPlaceTask,
):
    """
    Held by friction alone, the object travels to the receptacle and is put down in
    it, as SegMind, watching the run, sees it the last time it is let go of. What
    stands right below a thin receptacle, a plate's counter, may hold it up too.
    """
    food = performed_task.food.root
    events_of_the_food = [
        event
        for event in performed_task.detected_events
        if event.tracked_object is food
    ]
    last_release = max(
        event.timestamp
        for event in events_of_the_food
        if isinstance(event, LossOfGraspEvent)
    )

    put_down_on = {
        event.with_object
        for event in events_of_the_food
        if isinstance(event, PlacingEvent) and event.timestamp >= last_release
    }
    assert put_down_on & set(
        performed_task.colliding_bodies_below(performed_task.receptacle)
    )
