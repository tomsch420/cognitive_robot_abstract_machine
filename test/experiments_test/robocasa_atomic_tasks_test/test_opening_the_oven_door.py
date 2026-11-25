"""
The PandaOmron opening RoboCasa's oven door in MuJoCo, the way the trial campaigns found
it works.
"""

from __future__ import annotations

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.open_oven_door_trials import (
    OpenOvenDoorParameters,
    OpenOvenDoorPerformer,
    OpenOvenDoorSetup,
    OpeningStrategy,
    PressOnTheInnerFace,
)
from experiments.robocasa_atomic_tasks.let_down_doors import StandBeforeTheHandle
from experiments.robocasa_atomic_tasks.oven_door_trials import BaseMotion

# %% the oven of a composed episode


@pytest.fixture(scope="module")
def performer(
    tmp_path_factory: pytest.TempPathFactory,
) -> Iterator[OpenOvenDoorPerformer]:
    """
    The performer of trials of opening the oven door, in OpenOven's episode with the
    PandaOmron in MuJoCo.
    """
    setup = OpenOvenDoorSetup(
        database_url=f"sqlite:///{tmp_path_factory.mktemp('trials') / 'trials.sqlite'}"
    )
    with setup.performing() as performer:
        yield performer


# %% opening the door


def test_pulling_the_door_down_from_beside_it_opens_the_oven(
    performer: OpenOvenDoorPerformer,
):
    trial = performer.perform(
        parameters=OpenOvenDoorParameters(
            stand=StandBeforeTheHandle(sideways=-0.45, distance=0.6, turn=0.1),
            strategy=OpeningStrategy.PULLING_ALL_THE_WAY,
            base_motion=BaseMotion.MOVING_WITH_THE_ARM,
            release_angle=0.5,
            release_clearance=0.12,
            press=PressOnTheInnerFace(depth_below_the_top=0.1, sideways=0.0),
        )
    )

    assert trial.succeeded
    assert trial.measurements.driving_failure is None
    assert trial.measurements.pulling_failure is None
