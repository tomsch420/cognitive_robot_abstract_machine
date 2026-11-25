"""
Trials of the PandaOmron moving the object of RoboCasa's PickPlaceCounterToOven onto the
oven tray.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from experiments.robocasa_atomic_tasks.counter_to_oven_trials import (
    CounterToOvenMeasurements,
    CounterToOvenParameters,
    CounterToOvenSpace,
    CounterToOvenTrial,
    CounterToOvenTrialsInADatabase,
)
from krrood.parametrization.model_registries import UniformPriorRegistry

# %% the trials of moving the object


@pytest.fixture(scope="module")
def drawn_trials() -> list[CounterToOvenParameters]:
    """
    Parameters drawn uniformly from the space of moving the object onto the tray.
    """
    return CounterToOvenSpace().drawn_parameters(
        number_of_draws=64, model_registry=UniformPriorRegistry()
    )


def test_every_drawn_trial_lies_within_the_space(
    drawn_trials: list[CounterToOvenParameters],
):
    space = CounterToOvenSpace()

    for parameters in drawn_trials:
        for value, bounds in (
            (
                parameters.surroundings_buffer_zone_distance,
                space.surroundings_buffer_zone_distance,
            ),
            (
                parameters.counter_buffer_zone_distance,
                space.counter_buffer_zone_distance,
            ),
            (
                parameters.hand_clearance_from_the_oven,
                space.hand_clearance_from_the_oven,
            ),
            (parameters.carry_height, space.carry_height),
            (parameters.carry_clearance, space.carry_clearance),
            (parameters.margin_inside_the_tray, space.margin_inside_the_tray),
        ):
            assert bounds.lower <= value < bounds.upper


# %% keeping the trials


def test_a_kept_trial_is_read_back_as_it_was_tried(
    drawn_trials: list[CounterToOvenParameters], tmp_path: Path
):
    store = CounterToOvenTrialsInADatabase(
        database_url=f"sqlite:///{tmp_path / 'trials.sqlite'}"
    )
    store.prepare()
    trial = CounterToOvenTrial(
        parameters=drawn_trials[0],
        measurements=CounterToOvenMeasurements(
            seed=3,
            object_distance_from_the_tray_middle=0.05,
            object_height_above_the_tray=-0.01,
            failure="MotionMadeNoProgress",
            failure_message="CartesianPose/position#11 stopped approaching a goal",
        ),
        succeeded=False,
    )

    store.add(trial=trial)

    [kept] = store.stored_trials()
    assert kept.parameters == trial.parameters
    assert kept.measurements == trial.measurements
    assert kept.succeeded is trial.succeeded
