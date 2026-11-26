"""
Doors hinged at their bottom edge: where the base stands before the handle, how the
gripper comes at it, and letting the door down by it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from experiments.robocasa_atomic_tasks.let_down_doors import (
    HandleApproach,
    LetTheDoorDownByItsHandleAction,
    StandBeforeTheHandle,
    StandOnTheFloor,
)
from experiments.robocasa_atomic_tasks.panda_omron import PandaOmron, PandaOmronBody
from .conftest import box_body
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import ScrewAxisConnection
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
)
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% where the base stands


def test_a_stand_straight_back_from_the_handle_faces_the_oven():
    stand = StandBeforeTheHandle(sideways=0.0, distance=0.5, turn=0.0)

    on_the_floor = stand.stand_on_the_floor(
        handle_middle=np.array([1.0, 2.0]), heading_facing_the_oven=math.pi / 2
    )

    np.testing.assert_allclose(on_the_floor.position, [1.0, 1.5], atol=1e-12)
    assert on_the_floor.heading == pytest.approx(math.pi / 2)


def test_a_stand_to_the_right_lies_to_the_right_of_a_base_facing_the_oven():
    stand = StandBeforeTheHandle(sideways=0.3, distance=0.0, turn=0.0)

    on_the_floor = stand.stand_on_the_floor(
        handle_middle=np.array([1.0, 2.0]), heading_facing_the_oven=math.pi / 2
    )

    np.testing.assert_allclose(on_the_floor.position, [1.3, 2.0], atol=1e-12)


def test_a_turned_stand_turns_the_heading_to_the_left():
    stand = StandBeforeTheHandle(sideways=0.0, distance=0.0, turn=0.25)

    on_the_floor = stand.stand_on_the_floor(
        handle_middle=np.array([0.0, 0.0]), heading_facing_the_oven=1.0
    )

    assert on_the_floor.heading == pytest.approx(1.25)


def test_the_errors_of_a_reached_stand_are_its_distance_and_its_turn():
    stand = StandOnTheFloor(position=np.array([0.0, 0.0]), heading=math.pi - 0.1)
    reached = StandOnTheFloor(position=np.array([0.3, 0.4]), heading=-math.pi + 0.1)

    assert stand.position_error(reached=reached) == pytest.approx(0.5)
    assert stand.turn_error(reached=reached) == pytest.approx(0.2)


# %% a door hinged at its bottom edge


@pytest.fixture
def handle_on_a_let_down_door() -> Handle:
    """
    A handle 40 cm long, 5 cm in front of a door 60 cm wide and 50 cm tall and 20 cm
    above its middle.

    The door hangs from a horizontal hinge at (1, 2, 0.5) and faces the room along the
    world's -y, so that it opens towards the room.
    """
    world = World()
    with world.modify_world():
        oven = Body(name=PrefixedName("oven"))
        world.add_kinematic_structure_entity(oven)
        door = Body(
            name=PrefixedName("door"),
            collision=ShapeCollection([Box(scale=Scale(0.6, 0.04, 0.5))]),
        )
        world.add_connection(
            ScrewAxisConnection.create_with_dofs(
                world=world,
                parent=oven,
                child=door,
                screw_axis=ScrewAxisConnection.create_revolute_screw_axis(
                    axis=Vector3.X()
                ),
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    1.0, 2.0, 0.5, reference_frame=oven
                ),
                dof_limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                    lower_position=0.0, upper_position=1.05, maximum_speed=1.0
                ),
            )
        )
        handle = Handle(
            root=box_body(
                world,
                door,
                "handle",
                Scale(0.4, 0.02, 0.02),
                np.array([0.0, -0.05, 0.2]),
            )
        )
        world.add_semantic_annotation(handle)
    return handle


def test_a_stand_before_the_handle_faces_into_the_oven_from_the_handles_middle(
    handle_on_a_let_down_door: Handle,
):
    stand = StandBeforeTheHandle(sideways=-0.45, distance=0.6, turn=0.1)

    on_the_floor = stand.stand_before(handle=handle_on_a_let_down_door)

    expected = stand.stand_on_the_floor(
        handle_middle=np.array([1.0, 2.0 - 0.05]), heading_facing_the_oven=math.pi / 2
    )
    np.testing.assert_allclose(on_the_floor.position, expected.position, atol=1e-9)
    assert on_the_floor.heading == pytest.approx(expected.heading)


def test_the_grasp_from_the_front_comes_at_the_handle_towards_the_door(
    handle_on_a_let_down_door: Handle,
):
    grasp = HandleApproach.FROM_THE_FRONT.closest_grasp(
        handle=handle_on_a_let_down_door
    )

    np.testing.assert_allclose(
        grasp.world_T_grasp.to_np()[:3, 0],
        HandleApproach.FROM_THE_FRONT.value,
        atol=1e-9,
    )


# %% letting the door down


def test_letting_the_door_down_drives_to_the_stand_left_of_the_handle(
    panda_omron: PandaOmron, handle_on_a_let_down_door: Handle
):
    [arm] = panda_omron.all_arms

    letting_down = LetTheDoorDownByItsHandleAction.from_handle(
        handle=handle_on_a_let_down_door, arm=arm
    )

    expected = StandBeforeTheHandle.left_of_the_handle.stand_before(
        handle=handle_on_a_let_down_door
    )
    goal = letting_down.navigate.planar_goal_pose
    np.testing.assert_allclose(
        [float(goal.position.x), float(goal.position.y)], expected.position, atol=1e-9
    )
    assert float(goal.yaw) == pytest.approx(expected.heading)


def test_letting_the_door_down_pulls_it_all_the_way_by_the_grasp_from_the_front(
    panda_omron: PandaOmron, handle_on_a_let_down_door: Handle
):
    [arm] = panda_omron.all_arms

    letting_down = LetTheDoorDownByItsHandleAction.from_handle(
        handle=handle_on_a_let_down_door, arm=arm, release_clearance=0.2
    )

    assert letting_down.pull.opened_joint_position is None
    assert letting_down.pull.release_clearance == 0.2
    np.testing.assert_allclose(
        letting_down.pull.grasp_candidate.world_T_grasp.to_np(),
        HandleApproach.FROM_THE_FRONT.closest_grasp(
            handle=handle_on_a_let_down_door
        ).world_T_grasp.to_np(),
    )


def test_the_wrist_may_touch_the_door_and_its_handle(
    panda_omron: PandaOmron, handle_on_a_let_down_door: Handle
):
    [arm] = panda_omron.all_arms
    handle_body = handle_on_a_let_down_door.root

    allowance = LetTheDoorDownByItsHandleAction.wrist_allowance(
        arm=arm, handle_body=handle_body
    )

    assert {body.name.name for body in allowance.body_group_a} == {
        PandaOmronBody.FLANGE,
        PandaOmronBody.WRIST,
    }
    assert allowance.body_group_b == [
        handle_body.parent_connection.parent,
        handle_body,
    ]
