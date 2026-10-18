"""
Learning grasp models for the graspable RoboCasa objects.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from experiments.grasp_learning.robocasa import (
    ParallelRoboCasaGraspLearning,
    ParallelRoboCasaPolicyIteration,
    RoboCasaModel,
)
from experiments.physical_pick_up.objects import GraspableRoboCasaObject
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
)

requires_robocasa_assets = pytest.mark.skipif(
    not (RoboCasaDatasetLoader().directory / "objects").is_dir(),
    reason="the RoboCasa object assets are not downloaded",
)

# %% the models learned for


@requires_robocasa_assets
def test_every_downloaded_model_of_the_chosen_objects_has_grasps_tried(
    tmp_path: Path,
):
    graspables = [GraspableRoboCasaObject.APPLE, GraspableRoboCasaObject.POT]
    learning = ParallelRoboCasaGraspLearning(
        stored_parts=tmp_path, log_directory=tmp_path, graspables=graspables
    )

    models = learning.models()

    assert models == [
        RoboCasaModel(graspable=graspable, instance_index=instance_index)
        for graspable in graspables
        for instance_index in range(
            len(learning.loader.object_model_files(graspable.value.category))
        )
    ]


def test_a_model_is_chosen_on_the_pipelines_command_line_by_object_and_instance():
    model = RoboCasaModel(graspable=GraspableRoboCasaObject.POT, instance_index=3)

    assert model.pipeline_arguments() == [
        "--robocasa-object",
        GraspableRoboCasaObject.POT.name.lower(),
        "--instance",
        "3",
    ]


def test_an_iteration_runs_over_every_model_of_its_object_with_its_own_workers(
    tmp_path: Path,
):
    iterating = ParallelRoboCasaPolicyIteration(
        stored_parts=tmp_path,
        log_directory=tmp_path,
        workers_per_iteration=6,
        results=tmp_path / "results",
        iteration_arguments=["--stopping", "l1"],
    )

    arguments = iterating.arguments_of(GraspableRoboCasaObject.POT)

    assert arguments == [
        "--robocasa-object",
        GraspableRoboCasaObject.POT.name.lower(),
        "--workers",
        "6",
        "--results",
        str(tmp_path / "results" / GraspableRoboCasaObject.POT.name.lower()),
        "--stopping",
        "l1",
    ]
