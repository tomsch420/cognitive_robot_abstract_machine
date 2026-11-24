"""
Trials of the PandaOmron moving RoboCasa's oven door.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from experiments.robocasa_atomic_tasks.open_oven_door_trials import (
    OpenOvenDoorParameters,
    OpenOvenDoorSpace,
    OpeningStrategy,
    PressOnTheInnerFace,
)
from experiments.robocasa_atomic_tasks.oven_door_trials import (
    BaseMotion,
    StandBeforeTheHandle,
    StandOnTheFloor,
)
from krrood.parametrization.model_registries import UniformPriorRegistry
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import RevoluteConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from krrood_spatial_math.spatial_types import Vector3

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


# %% pressing on the door's inner face


@pytest.fixture
def door() -> Body:
    """
    A door 60 cm wide, 4 cm thick and 50 cm tall on a horizontal hinge, its frame at its
    middle.
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
            RevoluteConnection.create_with_dofs(
                world=world, parent=oven, child=door, axis=Vector3.X()
            )
        )
    return door


def test_the_pressed_point_lies_on_the_inner_face_below_the_top_edge(door: Body):
    press = PressOnTheInnerFace(depth_below_the_top=0.05, sideways=0.1)

    np.testing.assert_allclose(
        press.pressed_point(door=door), [0.1, 0.02, 0.25 - 0.05], atol=1e-9
    )


# %% the trials of opening the door


@pytest.fixture(scope="module")
def drawn_trials() -> list[OpenOvenDoorParameters]:
    """
    Parameters drawn uniformly from the space of opening the door.
    """
    return OpenOvenDoorSpace().drawn_parameters(
        number_of_draws=64, model_registry=UniformPriorRegistry()
    )


def test_every_drawn_trial_lies_within_the_space(
    drawn_trials: list[OpenOvenDoorParameters],
):
    space = OpenOvenDoorSpace()

    for parameters in drawn_trials:
        for value, bounds in (
            (parameters.stand.sideways, space.sideways),
            (parameters.stand.distance, space.distance),
            (parameters.stand.turn, space.turn),
            (parameters.release_angle, space.release_angle),
            (parameters.release_clearance, space.release_clearance),
            (parameters.press.depth_below_the_top, space.press_depth),
            (parameters.press.sideways, space.press_sideways),
        ):
            assert bounds.lower <= value < bounds.upper


def test_the_draws_try_every_strategy_and_base_motion(
    drawn_trials: list[OpenOvenDoorParameters],
):
    assert {parameters.strategy for parameters in drawn_trials} == set(OpeningStrategy)
    assert {parameters.base_motion for parameters in drawn_trials} == set(BaseMotion)
