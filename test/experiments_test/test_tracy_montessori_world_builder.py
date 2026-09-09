"""
The Montessori scene the simulated Tracy demo runs its scripted trials in
(:class:`~experiments.tracy_experiments.montessori.world.TracyMontessoriWorldBuilder`):
that it stands the board on Tracy's own built-in table at the height Tracy's own mount
computes, that the robot it mounts is Tracy, and that a scenario bound to it actually
runs, headless, on Tracy's own geometry.

Every test here parses a real Tracy (the ``iai_tracy_description`` ROS package must be
built and sourced), unlike :mod:`test_montessori_scenarios`, which deliberately avoids
it.
"""

from __future__ import annotations

import pytest
from coraplex.datastructures.enums import Arms

from experiments.montessori.pieces import KNOWN_PIECE_BY_CATEGORY
from experiments.montessori.scenarios import (
    AskTheQuestion,
    LetTheSceneSettle,
    PieceLayout,
    PiecePlacement,
    SortingStep,
    TracyHoldsAPiece,
    TracyIsIdleWhileAPieceIsPushed,
    TracyParkBothArms,
    TracyPickThePieceUp,
    TracyWatchesTheSceneStandStill,
)
from experiments.montessori.semantics import MontessoriShapeCategory
from experiments.montessori.world import BOARD_SCALE
from experiments.tracy_experiments.equipment import (
    parse_tracy,
    tracy_table_mount_position,
)
from experiments.tracy_experiments.montessori.world import (
    SHAPE_ROW_START_Y,
    SHAPE_ROW_X,
    TableTopHeightNotYetKnownError,
    TracyMontessoriWorld,
    TracyMontessoriWorldBuilder,
)
from semantic_digital_twin.robots.tracy import Tracy


def _layout_with_one_cube() -> PieceLayout:
    """
    A layout standing a single cube at a position :mod:`~experiments.tracy_experiments.

    montessori.world` already places its own loose shapes at, so it stands on Tracy's
    own table without needing a table-specific layout area of its own.
    """
    return PieceLayout(
        placements=[
            PiecePlacement(
                piece=KNOWN_PIECE_BY_CATEGORY[MontessoriShapeCategory.CUBE],
                x=SHAPE_ROW_X,
                y=SHAPE_ROW_START_Y,
                yaw=0.0,
            )
        ]
    )


# %% building the scene


def test_reading_the_table_top_height_before_building_raises():
    builder = TracyMontessoriWorldBuilder()

    with pytest.raises(TableTopHeightNotYetKnownError):
        builder.table_top_z


def test_the_table_top_height_matches_tracys_own_mounted_table():
    builder = TracyMontessoriWorldBuilder()

    builder.build(Tracy)

    expected_tracy_world = parse_tracy()
    _, expected_table_top_z = tracy_table_mount_position(
        expected_tracy_world, x=builder.mount_x, y=builder.mount_y
    )
    assert builder.table_top_z == expected_table_top_z


def test_the_built_scene_is_tracys_own_montessori_world():
    builder = TracyMontessoriWorldBuilder()

    montessori = builder.build(Tracy)

    assert isinstance(montessori, TracyMontessoriWorld)


def test_the_mounted_robot_is_tracy():
    builder = TracyMontessoriWorldBuilder()

    montessori = builder.build(Tracy)

    assert isinstance(montessori.robot, Tracy)


def test_the_built_scenes_board_sits_on_the_computed_table_top():
    builder = TracyMontessoriWorldBuilder()

    montessori = builder.build(Tracy)
    montessori.world.update_forward_kinematics()

    board_position_z = float(montessori.board.root.global_transform.to_position().z)
    assert board_position_z == pytest.approx(builder.table_top_z + BOARD_SCALE.z / 2)


# %% running a scenario on this builder's scene


def test_the_static_run_bound_to_this_builder_leaves_the_scene_undisturbed():
    scenario = TracyWatchesTheSceneStandStill(
        layout=_layout_with_one_cube(), world_builder=TracyMontessoriWorldBuilder()
    )

    world = scenario.build_world()
    for step in scenario.steps(world):
        step.perform(world)

    assert scenario.goal(world)()


def test_the_pushed_piece_run_bound_to_this_builder_moves_the_piece():
    scenario = TracyIsIdleWhileAPieceIsPushed(
        layout=_layout_with_one_cube(),
        world_builder=TracyMontessoriWorldBuilder(),
        pushed_category=MontessoriShapeCategory.CUBE,
    )

    world = scenario.build_world()
    for step in scenario.steps(world):
        step.perform(world)

    assert scenario.goal(world)()


# %% picking and placing by real MuJoCo contact friction


def test_a_piece_picked_up_by_mujoco_contact_friction_is_actually_held():
    """
    :class:`TracyPickThePieceUp` (driven by :class:`~experiments.tracy_experiments.

    pick_and_place_action.PickUpActionMujoco`) against the left arm, whose reach to
    this board position is proven -- see :mod:`~experiments.tracy_experiments.
    montessori.montessori_demo_mujoco`'s own working demo.
    :data:`~experiments.montessori.scenarios.THE_ARM_THAT_SORTS` is the right arm in
    production (the left one is broken on the physical robot); reach convergence for
    the right arm, and for a full pick-and-place with either arm, at this board
    position are both separate, open tuning gaps this test does not cover.
    """
    scenario = TracyHoldsAPiece(
        layout=_layout_with_one_cube(),
        world_builder=TracyMontessoriWorldBuilder(),
        held_category=MontessoriShapeCategory.CUBE,
    )

    world = scenario.build_world()
    actuators = scenario._actuators
    scene = scenario.simulation
    TracyParkBothArms(name=SortingStep.PARK, actuators=actuators, scene=scene).perform(
        world
    )
    LetTheSceneSettle(name=SortingStep.SETTLE, scene=scene).perform(world)
    TracyPickThePieceUp(
        name=SortingStep.PICK_UP,
        category=MontessoriShapeCategory.CUBE,
        arm=Arms.LEFT,
        actuators=actuators,
        scene=scene,
    ).perform(world)
    AskTheQuestion(name=SortingStep.ANSWER, scene=scene).perform(world)

    assert scenario.goal(world)()
