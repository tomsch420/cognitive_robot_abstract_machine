"""
Learning how to grasp an object by iterating a grasp policy until it stops changing.

The object's semantic annotation states where it may be grasped, as a statement over
:class:`~semantic_digital_twin.grasping.surface_grasp.SurfaceGrasp`. The robot first
tries grasps drawn uniformly from what the statement allows. Then each round learns a
relational probabilistic circuit from every attempt so far, tries the grasps the policy
chooses from it together with a share drawn uniformly again, and asks a stopping rule
whether the model still changes. Grasps are tried in a physical simulation, by many
worker processes at once, and every attempt and every model is stored in a database.

A new object takes its mesh, its annotation and its mass:

.. code-block:: python

    from pathlib import Path

    from experiments.grasp_learning.policy_iteration import PolicyIteration
    from experiments.grasp_learning.workers import IterationObject
    from semantic_digital_twin.semantic_annotations.semantic_annotations import Mug

    mug = IterationObject.from_mesh(Path("my_mug.stl"), Mug, mass=0.3, hollow=True)
    model = PolicyIteration(
        objects=[mug],
        database_uri="sqlite:///grasps.sqlite",
        results=Path("my_mug_grasps"),
    ).run()

    grasps = mug.scene().graspable.grasp_candidates(
        model.model_registry(), require_lifting=True
    )

An annotation that calls for parts, such as a mug's handle, has them found by the part
finding server ``SEMANTIC_DIGITAL_TWIN_PART_FINDING_SERVER_URL`` names, unless the
object is given a part finder of its own.

Run it with ``python -m experiments.grasp_learning.policy_iteration``; see ``--help``.
"""

from __future__ import annotations

import argparse
import inspect
import json
import math
import os
import time
import zlib
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from typing_extensions import Any, Dict, List, Optional, Type

from experiments.grasp_learning.convergence import SamplingDensities, SamplingPhase
from experiments.grasp_learning.database import (
    GraspDatabase,
    GraspDatabaseEnvironmentVariable,
)
from experiments.grasp_learning.grasp_selection import (
    DrawnFromModel,
    GraspChoice,
    SelectedByLiftProbability,
)
from experiments.grasp_learning.models import GraspModel
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
)
from experiments.grasp_learning.stopping_rules import (
    DistancesWithinFloors,
    FixedRounds,
    PolicySettled,
    PolicyUnchanged,
    RoundState,
    StoppingRule,
)
from experiments.grasp_learning.workers import (
    AttemptBatch,
    AttemptOutcome,
    IterationObject,
    WorkerPool,
)
from experiments.physical_pick_up.objects import PickUpObject
from experiments.physical_pick_up.robots import PickUpRobot
from semantic_digital_twin.semantic_annotations import semantic_annotations
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% the iteration


@dataclass
class PolicyIteration:
    """
    Iterates the grasp policy of one robot for the models of one object until a stopping
    rule says it no longer changes.
    """

    objects: List[IterationObject]
    """
    The models of the object, the first of which the stopping rule asks about.
    """

    database_uri: str
    """
    The database the attempts and the models are stored in, such as
    ``sqlite:///grasps.sqlite`` or ``postgresql+psycopg://user:password@host/grasps``;
    many workers write to a PostgreSQL database more smoothly than to an SQLite file.
    """

    results: Path
    """
    Where the rounds and the phases are written.
    """

    workers: int = 8
    """
    How many worker processes try grasps at once.
    """

    choice: GraspChoice = field(default_factory=DrawnFromModel)
    """
    How the policy chooses grasps from a model.
    """

    stopping: StoppingRule = field(default_factory=PolicySettled)
    """
    Tells when the policy no longer changes.
    """

    initial_attempts: int = 192
    """
    How many grasps drawn from the stated regions are tried before the first model.
    """

    attempts_per_round: int = 96
    """
    How many grasps are tried in each later round.
    """

    exploring_share: float = 0.2
    """
    The share of each later round's grasps drawn from the stated regions instead of the
    model, so that regions the model gave up on are still tried.
    """

    attempts_per_batch: int = 2
    """
    How many grasps a worker tries per batch.
    """

    maximum_rounds: int = 15
    """
    The iteration stops after this many rounds with a model in any case.
    """

    maximum_yaw: float = math.pi / 6
    """
    How far the object may be turned either way about the vertical axis, in radians.
    """

    seed: int = 0
    """
    Seeds the batches.
    """

    models: List[GraspModel] = field(default_factory=list, init=False)
    """
    The models of the rounds so far.
    """

    phases: List[SamplingPhase] = field(default_factory=list, init=False)
    """
    The phases the attempts were drawn in so far.
    """

    database: Optional[GraspDatabase] = field(default=None, init=False)
    """
    The connection to :attr:`database_uri`; open while the iteration runs.
    """

    pool: Optional[WorkerPool] = field(default=None, init=False)
    """
    The workers that try grasps; running while the iteration runs.
    """

    @property
    def rounds_file(self) -> Path:
        """
        The JSON lines file every round is appended to.
        """
        return self.results / "rounds.jsonl"

    @property
    def phases_file(self) -> Path:
        """
        The JSON lines file every phase is appended to.
        """
        return self.results / "phases.jsonl"

    def run(self) -> Optional[GraspModel]:
        """
        Start the workers, iterate the policy, and stop the workers.

        :return: The last model, stored with the lift rate of its own grasps; ``None`` if
            no round had a model.
        """
        self.database = GraspDatabase.connect(self.database_uri)
        with WorkerPool(
            objects=self.objects, size=self.workers, database_uri=self.database_uri
        ) as pool:
            self.pool = pool
            return self.iterate()

    def iterate(self) -> Optional[GraspModel]:
        """
        Iterate the policy with :attr:`pool`.

        :return: The last model; ``None`` if no round had a model.
        """
        self.results.mkdir(parents=True, exist_ok=True)
        scenes = {
            iteration_object.name: iteration_object.scene()
            for iteration_object in self.objects
        }
        main = scenes[self.objects[0].name]
        task = GraspLearningTask.of_graspable(
            main.graspable, type(main.arm.end_effector)
        )
        densities = SamplingDensities(
            parts={
                name: scene.graspable.grasped_part() for name, scene in scenes.items()
            },
            models=self.models,
        )
        learner = self.choice.learner()

        start = time.monotonic()
        exploring = self.attempt(
            GraspSource.STATED_REGIONS, None, self.initial_attempts, 0
        )
        self.report({"round": 0, **self.summary("exploring", exploring)}, start)

        model = None
        for round_number in range(1, self.maximum_rounds + 1):
            start = time.monotonic()
            attempts = self.own_attempts(task)
            model = learner.learn(task, list(attempts.values()))
            stopping = self.stopping.observe(
                RoundState(
                    number=round_number,
                    model=model,
                    attempts=attempts,
                    phases=self.phases,
                    densities=densities,
                    main_object=self.objects[0].name,
                    lift_rates=[earlier.verified_lift_rate for earlier in self.models],
                )
            )
            exploring_count = round(self.attempts_per_round * self.exploring_share)
            on_policy = self.attempt(
                GraspSource.LEARNED_MODEL,
                model,
                self.attempts_per_round - exploring_count,
                round_number,
            )
            exploring = self.attempt(
                GraspSource.STATED_REGIONS, None, exploring_count, round_number
            )
            tried = [outcome for outcome in on_policy if outcome.tried]
            model.verified_lift_rate = (
                sum(outcome.lifted for outcome in tried) / len(tried) if tried else None
            )
            self.database.add_model(model)
            self.models.append(model)
            self.report(
                {
                    "round": round_number,
                    "model_attempts": model.number_of_attempts,
                    "model_lift_rate_of_attempts": model.lift_rate_of_attempts,
                    **self.summary("on_policy", on_policy),
                    **self.summary("exploring", exploring),
                    **stopping,
                    "stops": self.stopping.stops,
                },
                start,
            )
            if self.stopping.stops:
                break
        return model

    def own_attempts(self, task: GraspLearningTask) -> Dict[int, GraspAttempt]:
        """
        :param task: The task of the iteration's objects.
        :return: The stored attempts of the task on the iteration's objects, by the
            number they were stored under; other iterations of the same task on other
            objects store theirs alongside.
        """
        names = {iteration_object.name for iteration_object in self.objects}
        return {
            number: attempt
            for number, attempt in self.database.numbered_attempts(task).items()
            if attempt.object_name in names
        }

    def attempt(
        self,
        source: GraspSource,
        model: Optional[GraspModel],
        number_of_attempts: int,
        round_number: int,
    ) -> List[AttemptOutcome]:
        """
        Have the workers try grasps on the objects in turn, and record the phase they
        were drawn in.

        :param source: Where the grasps are drawn from.
        :param model: The model the policy chooses them from, for
            :attr:`GraspSource.LEARNED_MODEL`.
        :param number_of_attempts: How many grasps are tried.
        :param round_number: The round, which seeds the batches.
        :return: What happened in every attempt that was not lost.
        """
        first_number = self.database.last_attempt_number() + 1
        sizes = [self.attempts_per_batch] * (
            number_of_attempts // self.attempts_per_batch
        )
        if number_of_attempts % self.attempts_per_batch:
            sizes.append(number_of_attempts % self.attempts_per_batch)
        batches = [
            AttemptBatch(
                iteration_object=self.objects[index % len(self.objects)],
                source=source,
                model=model,
                choice=self.choice,
                number_of_attempts=size,
                maximum_yaw=self.maximum_yaw,
                seed=zlib.crc32(
                    f"{self.seed}/{round_number}/{source.value}/{index}".encode()
                ),
            )
            for index, size in enumerate(sizes)
        ]
        outcomes = self.pool.run(batches)
        last_number = self.database.last_attempt_number()
        if last_number < first_number:
            return outcomes
        model_index = None if model is None else len(self.models)
        for name in sorted({outcome.object_name for outcome in outcomes}):
            phase = SamplingPhase(name, source, model_index, first_number, last_number)
            phase.append_to(self.phases_file)
            self.phases.append(phase)
        return outcomes

    @staticmethod
    def summary(kind: str, outcomes: List[AttemptOutcome]) -> Dict[str, Any]:
        """
        :param kind: What the outcomes are, as their keys begin.
        :param outcomes: The outcomes of some attempts.
        :return: How many grasps were tried, how many were not, and the share of those
            tried that lifted the object.
        """
        tried = [outcome for outcome in outcomes if outcome.tried]
        return {
            f"{kind}_attempts": len(tried),
            f"{kind}_untried": len(outcomes) - len(tried),
            f"{kind}_lift_rate": (
                sum(outcome.lifted for outcome in tried) / len(tried) if tried else None
            ),
        }

    def report(self, values: Dict[str, Any], start: float) -> None:
        """
        Append a round to :attr:`rounds_file` and print it.

        :param values: What the round did.
        :param start: When the round started, on the monotonic clock.
        """
        values = {
            **values,
            "lost_batches": self.pool.lost_batches,
            "seconds": time.monotonic() - start,
        }
        line = json.dumps(values)
        with self.rounds_file.open("a") as rounds:
            rounds.write(line + "\n")
        print(line, flush=True)


# %% the command line


class GraspChoiceName(Enum):
    """
    The ways of choosing grasps from a model the command line offers.
    """

    DRAWN = DrawnFromModel
    """
    Draw grasps from the model.
    """

    SELECTED = SelectedByLiftProbability
    """
    Choose grasps the model expects to lift the object.
    """


class StoppingRuleName(Enum):
    """
    The stopping rules the command line offers.
    """

    SETTLED = PolicySettled
    """
    Both L1 distances within their floors and a lift rate that no longer rises.
    """

    L1 = DistancesWithinFloors
    """
    Both L1 distances within their floors.
    """

    MMD = PolicyUnchanged
    """
    Consecutive policies that cannot be told apart.
    """

    ROUNDS = FixedRounds
    """
    The maximum number of rounds.
    """


def graspable_annotation_types() -> Dict[str, Type[HasGraspCandidates]]:
    """
    :return: Every annotation an object can be given that states where it may be
        grasped, by its name.
    """
    return {
        name: annotation_type
        for name, annotation_type in inspect.getmembers(
            semantic_annotations, inspect.isclass
        )
        if issubclass(annotation_type, HasGraspCandidates)
        and not inspect.isabstract(annotation_type)
    }


def iteration_object(arguments: argparse.Namespace) -> IterationObject:
    """
    :param arguments: The parsed command line.
    :return: The chosen object.
    """
    robot = PickUpRobot[arguments.robot.upper()]
    if arguments.mesh is None:
        return IterationObject(
            description=PickUpObject[arguments.object.upper()].value, robot=robot
        )
    return IterationObject.from_mesh(
        mesh_file=arguments.mesh,
        semantic_annotation_type=graspable_annotation_types()[arguments.annotation],
        mass=arguments.mass,
        hollow=arguments.hollow,
        meters_per_mesh_unit=arguments.meters_per_mesh_unit,
        robot=robot,
    )


def main() -> None:
    """
    Iterate the grasp policy for one object and print every round.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    objects = parser.add_mutually_exclusive_group(required=True)
    objects.add_argument(
        "--object",
        choices=[member.name.lower() for member in PickUpObject],
        help="one of the objects the pick-up experiment knows",
    )
    objects.add_argument("--mesh", type=Path, help="a new object, by its mesh file")
    parser.add_argument(
        "--annotation",
        choices=sorted(graspable_annotation_types()),
        help="what the new object is",
    )
    parser.add_argument("--mass", type=float, help="the new object's mass in kilograms")
    parser.add_argument(
        "--hollow",
        action="store_true",
        help="the new object is hollow, like a bowl or a mug",
    )
    parser.add_argument("--meters-per-mesh-unit", type=float, default=1.0)
    parser.add_argument(
        "--robot",
        choices=[robot.name.lower() for robot in PickUpRobot],
        default=PickUpRobot.PR2.name.lower(),
    )
    parser.add_argument(
        "--database",
        default=os.environ.get(GraspDatabaseEnvironmentVariable.URI),
        help="the database the attempts and models are stored in; "
        f"{GraspDatabaseEnvironmentVariable.URI.value} by default",
    )
    parser.add_argument(
        "--choice",
        choices=[member.name.lower() for member in GraspChoiceName],
        default=GraspChoiceName.DRAWN.name.lower(),
    )
    parser.add_argument(
        "--stopping",
        choices=[member.name.lower() for member in StoppingRuleName],
        default=StoppingRuleName.SETTLED.name.lower(),
    )
    parser.add_argument("--workers", type=int, default=PolicyIteration.workers)
    parser.add_argument(
        "--initial-attempts", type=int, default=PolicyIteration.initial_attempts
    )
    parser.add_argument(
        "--attempts-per-round", type=int, default=PolicyIteration.attempts_per_round
    )
    parser.add_argument(
        "--exploring-share", type=float, default=PolicyIteration.exploring_share
    )
    parser.add_argument(
        "--maximum-rounds", type=int, default=PolicyIteration.maximum_rounds
    )
    parser.add_argument("--seed", type=int, default=PolicyIteration.seed)
    parser.add_argument("--results", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.mesh is not None and (
        arguments.annotation is None or arguments.mass is None
    ):
        parser.error("--mesh needs --annotation and --mass")
    if arguments.database is None:
        parser.error(
            f"give --database or set {GraspDatabaseEnvironmentVariable.URI.value}"
        )
    PolicyIteration(
        objects=[iteration_object(arguments)],
        database_uri=arguments.database,
        results=arguments.results,
        workers=arguments.workers,
        choice=GraspChoiceName[arguments.choice.upper()].value(),
        stopping=StoppingRuleName[arguments.stopping.upper()].value(),
        initial_attempts=arguments.initial_attempts,
        attempts_per_round=arguments.attempts_per_round,
        exploring_share=arguments.exploring_share,
        maximum_rounds=arguments.maximum_rounds,
        seed=arguments.seed,
    ).run()


if __name__ == "__main__":
    main()
