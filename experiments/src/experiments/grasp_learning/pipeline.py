"""
Learning where grasps lift an object, for one robot: try grasps, store them, learn a
model, verify it, and store the model for the object's annotation to use.

Run it with ``python -m experiments.grasp_learning.pipeline``; see ``--help``. The
database is the one :attr:`~experiments.grasp_learning.database.GraspDatabaseEnvironmentVariable.URI`
names.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from datetime import timedelta

import numpy as np
from typing_extensions import List, Optional

from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.models import GraspModel, GraspModelLearner
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
)
from experiments.physical_pick_up.pick_up_experiment import (
    PickUpExperiment,
    PickUpGraspTrier,
)
from experiments.physical_pick_up.scene import PickUpSceneChoice
from krrood.parametrization.model_registries import ModelRegistry
from semantic_digital_twin.grasping.grasp_trials import GraspTrials

# %% the pipeline


@dataclass
class GraspLearningPipeline:
    """
    Learns where grasps lift the object of an experiment for the experiment's robot.

    The robot tries grasps drawn uniformly from the regions the object's annotation
    states, with the object standing anywhere in its pick-up area. A model is learned
    from every stored attempt of the task, verified by trying grasps it expects to lift
    the object, and stored with how often those did. Every attempt is stored, the
    verifying ones too.
    """

    experiment: PickUpExperiment
    """
    The robot and the object.
    """

    database: GraspDatabase
    """
    Where attempts and models are stored.
    """

    number_of_attempts: int = 100
    """
    How many grasps drawn from the stated regions are tried.
    """

    number_of_verifying_attempts: int = 20
    """
    How many grasps drawn from the learned model are tried.
    """

    maximum_yaw: float = math.pi / 6
    """
    How far the object may be turned either way about the vertical axis, in radians.
    """

    seed: int = 0
    """
    Seeds where the object stands in each attempt.
    """

    learner: GraspModelLearner = field(default_factory=GraspModelLearner)
    """
    Learns the model.
    """

    _generator: np.random.Generator = field(init=False)
    """
    Draws where the object stands.
    """

    def __post_init__(self):
        self._generator = np.random.default_rng(self.seed)

    @property
    def task(self) -> GraspLearningTask:
        """
        :return: The task the attempts belong to.
        """
        scene = self.experiment.scene
        return GraspLearningTask.of_graspable(
            scene.graspable, type(scene.arm.end_effector).__name__
        )

    def run(self) -> GraspModel:
        """
        Try grasps, learn a model from all attempts of the task, verify it, and store
        it.

        :return: The stored model.
        """
        self.attempt(GraspSource.STATED_REGIONS, self.number_of_attempts)
        model = self.learner.learn(self.task, self.database.attempts(self.task))
        verifying = self.attempt(
            GraspSource.LEARNED_MODEL,
            self.number_of_verifying_attempts,
            model.model_registry(),
        )
        model.verified_lift_rate = sum(
            attempt.grasp.result.lifted for attempt in verifying
        ) / max(len(verifying), 1)
        self.database.add_model(model)
        return model

    def attempt(
        self,
        source: GraspSource,
        number_of_attempts: int,
        model_registry: Optional[ModelRegistry] = None,
    ) -> List[GraspAttempt]:
        """
        Try grasps and store every attempt as soon as it is tried.

        :param source: Where the grasps are drawn from.
        :param number_of_attempts: How many grasps are drawn.
        :param model_registry: The learned model, for grasps drawn from one.
        :return: The attempts, in the order they were tried.
        """
        scene = self.experiment.scene
        trier = PickUpGraspTrier(
            experiment=self.experiment,
            generator=self._generator,
            maximum_yaw=self.maximum_yaw,
        )
        trials = GraspTrials(
            graspable=scene.graspable,
            trier=trier,
            number_of_trials=number_of_attempts,
            model_registry=model_registry,
            require_lifting=source == GraspSource.LEARNED_MODEL,
        )
        task = self.task
        attempts = []
        for record in trials.run():
            attempt = GraspAttempt(
                task=task,
                robot=type(scene.robot).__name__,
                object_name=scene.graspable.root.name.name,
                source=source,
                placement=trier.placements[-1],
                grasp=record.grasp,
            )
            self.database.add_attempts([attempt])
            attempts.append(attempt)
        return attempts


# %% the command line


def main() -> None:
    """
    Run the pipeline from the command line and report the stored model.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    scene_choice = PickUpSceneChoice(parser)
    scene_choice.add_arguments()
    parser.add_argument(
        "--attempts",
        type=int,
        default=GraspLearningPipeline.number_of_attempts,
        help="grasps drawn from the stated regions",
    )
    parser.add_argument(
        "--verifying-attempts",
        type=int,
        default=GraspLearningPipeline.number_of_verifying_attempts,
        help="grasps drawn from the learned model",
    )
    parser.add_argument("--seed", type=int, default=GraspLearningPipeline.seed)
    arguments = parser.parse_args()
    pipeline = GraspLearningPipeline(
        experiment=PickUpExperiment(
            scene=scene_choice.scene(arguments), time_limit=timedelta(seconds=20)
        ),
        database=GraspDatabase.from_environment(),
        number_of_attempts=arguments.attempts,
        number_of_verifying_attempts=arguments.verifying_attempts,
        seed=arguments.seed,
    )
    model = pipeline.run()
    print(
        f"{model.task}: learned from {model.number_of_attempts} attempts that lifted "
        f"the object in {model.lift_rate_of_attempts:.0%}; grasps drawn from the model "
        f"lifted it in {model.verified_lift_rate:.0%}"
    )


if __name__ == "__main__":
    main()
