from __future__ import annotations

import pytest

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_goals.simulation_mirror import RoboCasaSimulationMirror
from experiments.robocasa_task_execution.counter_to_sink import CounterToSinkEpisode
from robocasa.models.scenes.scene_registry import LayoutType, StyleType
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
)

# %% a composed task


@pytest.fixture
def episode() -> CounterToSinkEpisode:
    """
    An episode of the counter to sink task, whose target object has come to rest on the
    counter.
    """
    if not RoboCasaDatasetLoader().directory.exists():
        pytest.skip(
            "RoboCasa assets not downloaded. Run "
            "'python -m robocasa.scripts.download_kitchen_assets' first."
        )
    episode = CounterToSinkEpisode.compose(
        LayoutType.LAYOUT001, StyleType.STYLE001, seed=10
    )
    episode.let_the_target_object_fall()
    return episode


@pytest.fixture
def mirror(episode) -> RoboCasaSimulationMirror:
    return RoboCasaSimulationMirror(
        environment=episode.environment, world=episode.task.world
    )
