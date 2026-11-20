"""
Tests of trying ways of letting go of a held object from a starting world stored in the
database.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from sqlalchemy import select

from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.release_trials import ReleaseParameterSpace
from experiments.grasp_learning.records import (
    ReleaseParameters,
    ReleaseResult,
    ReleaseStage,
    ReleaseTrial,
)
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.stored_runs import (
    SimulatedMoment,
    StoredTaskRun,
)
from krrood.parametrization.model_registries import UniformPriorRegistry

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

# %% drawing ways of letting go


@pytest.fixture(scope="module")
def drawn() -> list[ReleaseParameters]:
    """
    Ways of letting go drawn uniformly from the default space.
    """
    np.random.seed(0)
    return ReleaseParameterSpace().drawn_parameters(
        number_of_releases=20, model_registry=UniformPriorRegistry()
    )


def test_as_many_ways_of_letting_go_are_drawn_as_asked_for(drawn):
    """
    Every draw answers the statement, since no condition ties parameters together.
    """
    assert len(drawn) == 20


def test_every_drawn_way_of_letting_go_lies_in_the_space(drawn):
    """
    Every parameter lies between the bounds of the space.
    """
    space = ReleaseParameterSpace()
    for parameters in drawn:
        for value, bounds in [
            (parameters.opening, space.opening),
            (parameters.opening_force, space.opening_force),
            (parameters.settling_duration, space.settling_duration),
            (parameters.retreat_tilt, space.retreat_tilt),
            (parameters.retreat_turn, space.retreat_turn),
            (parameters.retreat_distance, space.retreat_distance),
            (parameters.retreat_velocity, space.retreat_velocity),
        ]:
            assert bounds.lower <= value < bounds.upper


# %% the direction the hand moves away in


def parameters_retreating(*, tilt: float, turn: float) -> ReleaseParameters:
    """
    :return: A way of letting go that moves the hand away tilted and turned so.
    """
    return ReleaseParameters(
        opening=0.9,
        opening_force=50.0,
        settling_duration=0.5,
        retreat_tilt=tilt,
        retreat_turn=turn,
        retreat_distance=0.1,
        retreat_velocity=0.05,
    )


approach_axis = np.array([0.0, 0.0, 1.0])
closing_axis = np.array([0.0, 1.0, 0.0])


def test_an_untilted_retreat_goes_back_along_the_approach():
    """
    Without tilt, the hand moves away exactly the way it came.
    """
    direction = parameters_retreating(tilt=0.0, turn=1.0).retreat_direction(
        approach_axis=approach_axis, closing_axis=closing_axis
    )
    assert np.allclose(direction, -approach_axis)


def test_a_retreat_tilted_a_quarter_turn_without_turning_goes_along_the_closing_axis():
    """
    Tilted a quarter turn and not turned, the hand moves along the closing axis.
    """
    direction = parameters_retreating(tilt=0.5 * math.pi, turn=0.0).retreat_direction(
        approach_axis=approach_axis, closing_axis=closing_axis
    )
    assert np.allclose(direction, closing_axis)


def test_a_retreat_turned_a_quarter_turn_goes_counterclockwise_from_the_closing_axis():
    """
    Turned a quarter turn, the tilt leans counterclockwise about the approach axis from
    the closing axis.
    """
    direction = parameters_retreating(
        tilt=0.5 * math.pi, turn=0.5 * math.pi
    ).retreat_direction(approach_axis=approach_axis, closing_axis=closing_axis)
    assert np.allclose(direction, np.cross(approach_axis, closing_axis))


def test_every_retreat_direction_is_a_unit_vector(drawn):
    """
    The direction has length one whatever the tilt and turn.
    """
    for parameters in drawn:
        direction = parameters.retreat_direction(
            approach_axis=approach_axis, closing_axis=closing_axis
        )
        assert np.linalg.norm(direction) == pytest.approx(1.0)


# %% storing a trial


def test_a_stored_release_trial_reads_back_as_it_was_stored():
    """
    A trial reads back with the parameters, result, stage and moments it was stored
    with.
    """
    from experiments.orm.ormatic_interface import ReleaseTrialDAO

    database = GraspDatabase.connect("sqlite:///:memory:")
    trial = ReleaseTrial(
        starting_world_id=1,
        parameters=parameters_retreating(tilt=0.3, turn=-1.0),
        result=ReleaseResult(
            finger_separation_after_opening=0.075,
            fingers_touched_the_object_after_opening=True,
            object_carried_along=0.2,
            fingers_touched_the_object_at_the_end=True,
            accomplished=False,
        ),
        failed_stage=ReleaseStage.RETREATING,
        run=StoredTaskRun(
            task="PickPlaceSinkToCounter",
            layout=LayoutType.LAYOUT001,
            style=StyleType.STYLE001,
            seed=0,
            robot=RobotModel.PANDA_OMRON,
            degree_of_freedom_names=["lever"],
            moments=[
                SimulatedMoment(simulation_time=0.0, positions=[0.0]),
                SimulatedMoment(simulation_time=0.5, positions=[0.1]),
            ],
            judgement=None,
        ),
    )
    database.add_trial(trial=trial)
    [stored] = database.session.scalars(select(ReleaseTrialDAO)).all()

    read = stored.from_dao()

    assert read.parameters == trial.parameters
    assert read.result == trial.result
    assert read.failed_stage == trial.failed_stage
    assert read.run.moments == trial.run.moments
