"""
Iterating a grasp policy until it stops changing: try grasps drawn uniformly from where
an object's annotation says it may be grasped, learn a model from every attempt so far,
try the grasps the model chooses together with a share drawn uniformly again, learn
again, and stop once a stopping rule says the model no longer changes.

The grasps are tried on every model of an object, by a pool of worker processes that
each keep the simulations of the objects they try grasps on. Every attempt and every
model is stored in the database
:attr:`~experiments.grasp_learning.database.GraspDatabaseEnvironmentVariable.URI` names,
and each round is written to a JSON lines file.

Run it with ``python -m experiments.grasp_learning.policy_iteration``; see ``--help``.
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import multiprocessing.connection
import time
import zlib
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from datetime import timedelta
from enum import Enum
from functools import cached_property
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Any, Dict, List, Optional, Tuple

from experiments.grasp_learning.convergence import (
    ConvergenceMeter,
    SamplingDensities,
    SamplingPhase,
)
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.grasp_selection import (
    GraspSelection,
    LiftProbability,
    WorldFrameLearner,
    turned,
)
from experiments.grasp_learning.models import GraspModel, GraspModelLearner
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
)
from experiments.physical_pick_up.objects import (
    GraspableRoboCasaObject,
    ObjectDescription,
    PickUpObject,
)
from experiments.physical_pick_up.pick_up_experiment import PickUpExperiment
from experiments.physical_pick_up.robots import ObjectPlacement, PickUpRobot
from experiments.physical_pick_up.scene import PickUpScene
from experiments.physical_pick_up.stored_parts import StoredPartLocator
from krrood.parametrization.parameterizer import UnderspecifiedParameters
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
)
from semantic_digital_twin.grasping.surface_grasp import (
    SurfaceGrasp,
    draw_surface_grasps,
)
from semantic_digital_twin.part_finding.annotation_part_finder import (
    AnnotationPartFinder,
)
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% the objects grasps are tried on


@dataclass
class IterationObject:
    """
    One object the iteration tries grasps on, and the robot that tries them.
    """

    description: ObjectDescription
    """
    The object.
    """

    robot: PickUpRobot
    """
    The robot that picks it up.
    """

    stored_parts: Optional[Path] = None
    """
    Parts located before for the object, which it is split by instead of asking the part
    finding server; an object without stored parts then has none.
    """

    @property
    def name(self) -> str:
        """
        The name of the object's body, which tells the object apart from the others.
        """
        return self.description.body_name

    def scene(self) -> PickUpScene:
        """
        :return: The scene of the robot picking the object up.
        """
        part_finder = (
            AnnotationPartFinder()
            if self.stored_parts is None
            else AnnotationPartFinder(
                locator=StoredPartLocator(
                    directory=self.stored_parts, require_stored_answer=False
                )
            )
        )
        return PickUpScene(
            object_description=self.description,
            robot_setup=self.robot.value,
            part_finder=part_finder,
        )


# %% grasps as points


def grasp_features(grasp: SurfaceGrasp) -> List[float]:
    """
    :param grasp: A surface grasp.
    :return: The grasp as a point, its azimuth on the unit circle so that grasps from
        either side of zero lie close.
    """
    return [
        math.cos(grasp.azimuth),
        math.sin(grasp.azimuth),
        grasp.height,
        grasp.depth,
        grasp.pitch,
        grasp.roll,
    ]


# %% a model, prepared for drawing grasps for one part


@dataclass
class PreparedModel:
    """
    A learned model, asked about the grasps of one part.
    """

    model: GraspModel
    """
    The model.
    """

    part: HasGraspCandidates
    """
    The grasped part.
    """

    @cached_property
    def answers(self) -> bool:
        """
        Whether the model gives any probability to lifting grasps where the part's
        statement allows them.
        """
        parameters = UnderspecifiedParameters(
            self.part.surface_grasp_statement(require_lifting=True)
        )
        model = self.model.model_registry().get_model(parameters)
        return parameters.resolve_conditioned_and_truncated_model(model) is not None

    @cached_property
    def lift_probability(self) -> LiftProbability:
        """
        How likely the model expects a grasp to lift the object.
        """
        return LiftProbability(model=self.model, part=self.part)


# %% choosing the grasps of the policy


class GraspChoice(ABC):
    """
    How the policy chooses a grasp from a learned model.
    """

    @abstractmethod
    def grasp(
        self,
        prepared: PreparedModel,
        placement: ObjectPlacement,
        generator: np.random.Generator,
    ) -> Optional[SurfaceGrasp]:
        """
        :param prepared: The model, asked about the grasps of the grasped part.
        :param placement: Where the object stands for the attempt.
        :param generator: The source of randomness.
        :return: The grasp to try, seen from the object, without a result; ``None`` if
            the model has none to offer.
        """

    def learner(self) -> GraspModelLearner:
        """
        :return: What learns the models this choice draws from.
        """
        return GraspModelLearner()


@dataclass
class DrawnFromModel(GraspChoice):
    """
    Draws one grasp from the model, asked for grasps that lift the object.
    """

    def grasp(
        self,
        prepared: PreparedModel,
        placement: ObjectPlacement,
        generator: np.random.Generator,
    ) -> Optional[SurfaceGrasp]:
        if not prepared.answers:
            return None
        [drawn] = draw_surface_grasps(
            prepared.part.surface_grasp_statement(require_lifting=True),
            1,
            prepared.model.model_registry(),
        )
        return drawn


@dataclass
class SelectedByLiftProbability(GraspChoice):
    """
    Chooses a grasp the model expects to lift the object, see
    :mod:`experiments.grasp_learning.grasp_selection`.
    """

    selection: GraspSelection = field(default_factory=GraspSelection)
    """
    How the grasp is chosen among those drawn from the model.
    """

    world_frame: bool = True
    """
    Whether the models are learned over grasps seen from the world, so that a chosen
    grasp is turned back by the yaw of the object it is tried on.
    """

    def grasp(
        self,
        prepared: PreparedModel,
        placement: ObjectPlacement,
        generator: np.random.Generator,
    ) -> Optional[SurfaceGrasp]:
        if not prepared.answers:
            return None
        chosen = self.selection.choose(
            prepared.part, prepared.model, prepared.lift_probability, generator
        )
        if chosen is None or not self.world_frame:
            return chosen
        return turned(chosen, -placement.yaw)

    def learner(self) -> GraspModelLearner:
        """
        :return: A learner over grasps seen from the world if :attr:`world_frame`,
            otherwise as seen from the object.
        """
        return WorldFrameLearner() if self.world_frame else GraspModelLearner()


# %% the workers


@dataclass
class AttemptBatch:
    """
    A few grasps for one worker to try on one object.
    """

    iteration_object: IterationObject
    """
    The object the grasps are tried on.
    """

    source: GraspSource
    """
    Where the grasps are drawn from.
    """

    model: Optional[GraspModel]
    """
    The model the policy chooses the grasps from; ``None`` for the stated regions.
    """

    choice: GraspChoice
    """
    How the policy chooses a grasp from the model.
    """

    number_of_attempts: int
    """
    How many grasps are tried.
    """

    maximum_yaw: float
    """
    How far the object may be turned either way about the vertical axis, in radians.
    """

    seed: int
    """
    Seeds where the object stands and which grasps are drawn.
    """


@dataclass
class AttemptOutcome:
    """
    What a worker reports of one attempt.
    """

    object_name: str
    """
    The object the grasp was tried on.
    """

    source: GraspSource
    """
    Where the grasp was drawn from.
    """

    features: Optional[List[float]]
    """
    The grasp as a point; ``None`` if no grasp was tried.
    """

    lifted: bool = False
    """
    Whether the grasp lifted the object.
    """

    @property
    def tried(self) -> bool:
        """
        Whether a grasp was tried; the policy had none to offer otherwise, or offered
        one beyond the part's surface.
        """
        return self.features is not None


@dataclass
class IterationWorker:
    """
    Tries the grasps of the batches it is given, each object in a simulation of its own
    that is built once and reset between attempts.
    """

    database: GraspDatabase
    """
    Where the attempts are stored.
    """

    experiments: Dict[str, PickUpExperiment] = field(default_factory=dict)
    """
    The experiment of each object tried so far, by the object's name.
    """

    prepared_models: Dict[Tuple[str, int], PreparedModel] = field(default_factory=dict)
    """
    The models asked about each object's part so far, by the object's name and a
    checksum of the model.
    """

    time_limit: timedelta = timedelta(seconds=20)
    """
    The simulated time after which an attempt that is still moving is given up on.
    """

    @classmethod
    def serve(
        cls,
        jobs: multiprocessing.connection.Connection,
        results: multiprocessing.connection.Connection,
    ) -> None:
        """
        Try the batches arriving on ``jobs`` until ``None`` arrives, answering each with
        its outcomes on ``results``.

        :param jobs: Where the batches arrive.
        :param results: Where the outcomes of each batch are sent, in the order the
            batches arrive.
        """
        worker = cls(database=GraspDatabase.from_environment())
        for batch in iter(jobs.recv, None):
            results.send(worker.try_batch(batch))

    def experiment(self, iteration_object: IterationObject) -> PickUpExperiment:
        """
        :param iteration_object: An object.
        :return: The object's experiment, built the first time it is asked for.
        """
        name = iteration_object.name
        if name not in self.experiments:
            self.experiments[name] = PickUpExperiment(
                scene=iteration_object.scene(), time_limit=self.time_limit
            )
        return self.experiments[name]

    def prepared(
        self, model: GraspModel, part: HasGraspCandidates, name: str
    ) -> PreparedModel:
        """
        :param model: A model.
        :param part: The grasped part of the object named ``name``.
        :param name: The object's name.
        :return: The model asked about the part, prepared the first time it is asked for.
        """
        key = (name, zlib.crc32(model.circuit.encode()))
        if key not in self.prepared_models:
            self.prepared_models.clear()
            self.prepared_models[key] = PreparedModel(model=model, part=part)
        return self.prepared_models[key]

    def try_batch(self, batch: AttemptBatch) -> List[AttemptOutcome]:
        """
        Try the grasps of a batch and store every attempt as soon as it is tried.

        :param batch: The grasps to try.
        :return: What happened in each attempt.
        """
        experiment = self.experiment(batch.iteration_object)
        scene = experiment.scene
        part = scene.graspable.grasped_part()
        task = GraspLearningTask.of_graspable(
            scene.graspable, type(scene.arm.end_effector)
        )
        generator = np.random.default_rng(batch.seed)
        np.random.seed(batch.seed % 2**32)
        outcomes = []
        for _ in range(batch.number_of_attempts):
            placement = scene.pick_up_area.random_placement(
                generator, batch.maximum_yaw
            )
            grasp = self.drawn_grasp(batch, part, placement, generator)
            if grasp is None or not grasp.reaches_surface_of(part):
                outcomes.append(
                    AttemptOutcome(
                        object_name=batch.iteration_object.name,
                        source=batch.source,
                        features=None,
                    )
                )
                continue
            grasp.result = experiment.run(
                grasp.grasp_candidate(part), placement=placement
            )
            self.database.add_attempts(
                [
                    GraspAttempt(
                        task=task,
                        robot=type(scene.robot),
                        object_name=batch.iteration_object.name,
                        source=batch.source,
                        placement=placement,
                        grasp=grasp,
                    )
                ]
            )
            outcomes.append(
                AttemptOutcome(
                    object_name=batch.iteration_object.name,
                    source=batch.source,
                    features=grasp_features(grasp),
                    lifted=grasp.result.lifted,
                )
            )
        return outcomes

    def drawn_grasp(
        self,
        batch: AttemptBatch,
        part: HasGraspCandidates,
        placement: ObjectPlacement,
        generator: np.random.Generator,
    ) -> Optional[SurfaceGrasp]:
        """
        :param batch: The batch the grasp belongs to.
        :param part: The grasped part.
        :param placement: Where the object stands for the attempt.
        :param generator: The source of randomness.
        :return: A grasp drawn uniformly from the part's statement for the stated
            regions, otherwise the one the policy chooses; ``None`` if it has none.
        """
        if batch.model is None:
            [drawn] = draw_surface_grasps(part.surface_grasp_statement(), 1)
            return drawn
        prepared = self.prepared(batch.model, part, batch.iteration_object.name)
        return batch.choice.grasp(prepared, placement, generator)


@dataclass
class WorkerPool:
    """
    Worker processes that try the grasps of batches, each object always by the same
    workers so that each worker keeps the simulations of only a few objects.

    A worker whose process ends while it tries a batch is replaced, and that batch is
    lost.
    """

    objects: List[IterationObject]
    """
    The objects the workers try grasps on.
    """

    size: int
    """
    How many workers there are.
    """

    context: Any = field(
        default_factory=lambda: multiprocessing.get_context("spawn"), repr=False
    )
    """
    How the worker processes are started.
    """

    processes: List[Any] = field(default_factory=list, init=False, repr=False)
    """
    The process of each worker.
    """

    jobs: List[multiprocessing.connection.Connection] = field(
        default_factory=list, init=False, repr=False
    )
    """
    Where the batches are sent to each worker.
    """

    results: List[multiprocessing.connection.Connection] = field(
        default_factory=list, init=False, repr=False
    )
    """
    Where each worker's outcomes arrive.
    """

    lost_batches: int = field(default=0, init=False)
    """
    How many batches were lost with a worker so far.
    """

    def __enter__(self) -> WorkerPool:
        for index in range(self.size):
            self.jobs.append(None)
            self.results.append(None)
            self.processes.append(None)
            self._start(index)
        return self

    def __exit__(self, *exception: Any) -> None:
        for jobs in self.jobs:
            jobs.send(None)
        for process in self.processes:
            process.join(timeout=30)
            process.kill()

    def _start(self, index: int) -> None:
        """
        Start the process of a worker, with new connections to it.

        :param index: The worker.
        """
        jobs_end, self.jobs[index] = self.context.Pipe(duplex=False)
        self.results[index], results_end = self.context.Pipe(duplex=False)
        process = self.context.Process(
            target=IterationWorker.serve, args=(jobs_end, results_end), daemon=True
        )
        process.start()
        self.processes[index] = process

    def workers_of(self, iteration_object: IterationObject) -> List[int]:
        """
        :param iteration_object: One of :attr:`objects`.
        :return: The workers that try grasps on it.
        """
        names = [candidate.name for candidate in self.objects]
        position = names.index(iteration_object.name)
        if self.size >= len(names):
            return [
                worker for worker in range(self.size) if worker % len(names) == position
            ]
        return [position % self.size]

    def run(self, batches: List[AttemptBatch]) -> List[AttemptOutcome]:
        """
        :param batches: The batches to try.
        :return: The outcomes of every batch that was not lost.
        """
        waiting = list(batches)
        busy: Dict[int, AttemptBatch] = {}
        outcomes = []
        while waiting or busy:
            for batch in list(waiting):
                idle = [
                    worker
                    for worker in self.workers_of(batch.iteration_object)
                    if worker not in busy
                ]
                if not idle:
                    continue
                busy[idle[0]] = batch
                self.jobs[idle[0]].send(batch)
                waiting.remove(batch)
            multiprocessing.connection.wait(
                [self.results[worker] for worker in busy]
                + [self.processes[worker].sentinel for worker in busy]
            )
            for worker in list(busy):
                if self.results[worker].poll():
                    outcomes.extend(self.results[worker].recv())
                    del busy[worker]
                    continue
                if self.processes[worker].is_alive():
                    continue
                self.lost_batches += 1
                self._start(worker)
                del busy[worker]
        return outcomes


# %% telling when to stop


@dataclass
class RoundState:
    """
    What a stopping rule may look at in one round.
    """

    number: int
    """
    The round, from 1.
    """

    model: GraspModel
    """
    The round's model, learned from :attr:`attempts`.
    """

    attempts: Dict[int, GraspAttempt]
    """
    Every attempt the model was learned from, by the number it was stored under.
    """

    phases: List[SamplingPhase]
    """
    The phases the attempts were drawn in.
    """

    densities: SamplingDensities
    """
    The distributions the attempts were drawn from.
    """

    main_object: str
    """
    The object whose part the rule asks the model about.
    """


class StoppingRule(ABC):
    """
    Tells from each round whether the policy no longer changes.
    """

    @abstractmethod
    def observe(self, state: RoundState) -> Dict[str, Any]:
        """
        Look at a round.

        :param state: The round.
        :return: What the rule measured, for the round's report.
        """

    @property
    @abstractmethod
    def stops(self) -> bool:
        """
        Whether the rounds observed so far say the policy no longer changes.
        """


@dataclass
class FixedRounds(StoppingRule):
    """
    Never stops early: the iteration runs for its maximum number of rounds.
    """

    def observe(self, state: RoundState) -> Dict[str, Any]:
        return {}

    @property
    def stops(self) -> bool:
        return False


@dataclass
class PolicyUnchanged(StoppingRule):
    """
    Stops once grasps drawn from consecutive models cannot be told apart for some rounds
    in a row, by a permutation test of their maximum mean discrepancy.
    """

    policy_samples: int = 400
    """
    How many grasps are drawn from each model to compare it with the previous one.
    """

    significance: float = 0.05
    """
    Below this p-value two consecutive models count as different.
    """

    required_stable_rounds: int = 2
    """
    How many rounds in a row the model has to stay the same.
    """

    number_of_permutations: int = 500
    """
    How many permutations the test draws.
    """

    seed: int = 0
    """
    Seeds the permutations.
    """

    stable_rounds: int = field(default=0, init=False)
    """
    How many rounds in a row the model stayed the same so far.
    """

    previous_samples: Optional[NDArray[np.float64]] = field(default=None, init=False)
    """
    The grasps drawn from the previous round's model.
    """

    def observe(self, state: RoundState) -> Dict[str, Any]:
        part = state.densities.parts[state.main_object]
        grasps = draw_surface_grasps(
            part.surface_grasp_statement(require_lifting=True),
            self.policy_samples,
            state.model.model_registry(),
        )
        samples = np.array([grasp_features(grasp) for grasp in grasps])
        p_value = None
        if (
            self.previous_samples is not None
            and len(samples)
            and len(self.previous_samples)
        ):
            p_value = self.p_value(
                self.previous_samples,
                samples,
                np.random.default_rng((self.seed, state.number)),
            )
            self.stable_rounds = (
                self.stable_rounds + 1 if p_value >= self.significance else 0
            )
        self.previous_samples = samples
        return {"policy_change_p_value": p_value, "stable_rounds": self.stable_rounds}

    @property
    def stops(self) -> bool:
        return self.stable_rounds >= self.required_stable_rounds

    def p_value(
        self,
        previous: NDArray[np.float64],
        current: NDArray[np.float64],
        generator: np.random.Generator,
    ) -> float:
        """
        :param previous: Grasps drawn from one policy, one row per grasp.
        :param current: Grasps drawn from another policy.
        :param generator: Draws the permutations.
        :return: The share of permutations whose maximum mean discrepancy under a
            Gaussian kernel is at least the observed one; small when the policies
            differ.
        """
        pooled = np.vstack([previous, current])
        pooled = (pooled - pooled.mean(axis=0)) / np.maximum(pooled.std(axis=0), 1e-9)
        squared_distances = np.sum(
            (pooled[:, None, :] - pooled[None, :, :]) ** 2, axis=-1
        )
        bandwidth = np.median(squared_distances[squared_distances > 0])
        kernel = np.exp(-squared_distances / bandwidth)
        labels = np.r_[np.zeros(len(previous), bool), np.ones(len(current), bool)]

        def discrepancy(in_current: NDArray[np.bool_]) -> float:
            in_previous = ~in_current
            return (
                kernel[np.ix_(in_previous, in_previous)].mean()
                + kernel[np.ix_(in_current, in_current)].mean()
                - 2 * kernel[np.ix_(in_previous, in_current)].mean()
            )

        observed = discrepancy(labels)
        exceeding = sum(
            discrepancy(generator.permutation(labels)) >= observed
            for _ in range(self.number_of_permutations)
        )
        return (exceeding + 1) / (self.number_of_permutations + 1)


@dataclass
class DistancesWithinFloors(StoppingRule):
    """
    Stops once both L1 distances of :mod:`experiments.grasp_learning.convergence` stay
    within their floors for some rounds in a row.
    """

    required_converged_rounds: int = 3
    """
    How many rounds in a row both distances have to stay within their floors.
    """

    sample_size: int = 5000
    """
    How many samples each distance is estimated from.
    """

    seed: int = 0
    """
    Seeds the resampling.
    """

    meter: Optional[ConvergenceMeter] = field(default=None, init=False)
    """
    Measures the distances; made in the first round.
    """

    converged_rounds: int = field(default=0, init=False)
    """
    How many rounds in a row both distances stayed within their floors so far.
    """

    def observe(self, state: RoundState) -> Dict[str, Any]:
        if self.meter is None:
            self.meter = ConvergenceMeter(
                task=state.model.task,
                main_object=state.main_object,
                densities=state.densities,
                sample_size=self.sample_size,
                seed=self.seed,
            )
        measurements = self.meter.measure(state.number, state.attempts, state.phases)
        self.converged_rounds = (
            self.converged_rounds + 1 if measurements.converged else 0
        )
        return {
            "measurements": asdict(measurements),
            "converged_rounds": self.converged_rounds,
        }

    @property
    def stops(self) -> bool:
        return self.converged_rounds >= self.required_converged_rounds


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

    pool: WorkerPool
    """
    The workers that try grasps.
    """

    database: GraspDatabase
    """
    Where the workers store the attempts, read back to learn.
    """

    results: Path
    """
    Where the rounds and the phases are written.
    """

    choice: GraspChoice = field(default_factory=DrawnFromModel)
    """
    How the policy chooses grasps from a model.
    """

    stopping: StoppingRule = field(default_factory=DistancesWithinFloors)
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
        :return: The last model, stored with the lift rate of its own grasps; ``None`` if
            no round had a model.
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
            attempts = self.database.numbered_attempts(task)
            model = learner.learn(task, list(attempts.values()))
            stopping = self.stopping.observe(
                RoundState(
                    number=round_number,
                    model=model,
                    attempts=attempts,
                    phases=self.phases,
                    densities=densities,
                    main_object=self.objects[0].name,
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


def iteration_objects(arguments: argparse.Namespace) -> List[IterationObject]:
    """
    :param arguments: The parsed command line.
    :return: The models of the chosen object.
    """
    robot = PickUpRobot[arguments.robot.upper()]
    if arguments.robocasa_object is None:
        descriptions = [PickUpObject[arguments.object.upper()].value]
    else:
        category = GraspableRoboCasaObject[arguments.robocasa_object.upper()].value
        count = len(RoboCasaDatasetLoader().object_model_files(category.category))
        if arguments.models is not None:
            count = min(count, arguments.models)
        descriptions = [
            category.description(instance_index=index) for index in range(count)
        ]
    return [
        IterationObject(
            description=description, robot=robot, stored_parts=arguments.stored_parts
        )
        for description in descriptions
    ]


def main() -> None:
    """
    Iterate the policy for the models of one object and print every round.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    objects = parser.add_mutually_exclusive_group(required=True)
    objects.add_argument(
        "--object", choices=[member.name.lower() for member in PickUpObject]
    )
    objects.add_argument(
        "--robocasa-object",
        choices=[member.name.lower() for member in GraspableRoboCasaObject],
        help="every downloaded model of a graspable RoboCasa object",
    )
    parser.add_argument("--models", type=int, help="at most this many RoboCasa models")
    parser.add_argument(
        "--robot",
        choices=[robot.name.lower() for robot in PickUpRobot],
        default=PickUpRobot.PR2.name.lower(),
    )
    parser.add_argument("--stored-parts", type=Path)
    parser.add_argument(
        "--choice",
        choices=[member.name.lower() for member in GraspChoiceName],
        default=GraspChoiceName.DRAWN.name.lower(),
    )
    parser.add_argument(
        "--stopping",
        choices=[member.name.lower() for member in StoppingRuleName],
        default=StoppingRuleName.L1.name.lower(),
    )
    parser.add_argument("--workers", type=int, default=16)
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
    chosen_objects = iteration_objects(arguments)
    with WorkerPool(objects=chosen_objects, size=arguments.workers) as pool:
        PolicyIteration(
            objects=chosen_objects,
            pool=pool,
            database=GraspDatabase.from_environment(),
            results=arguments.results,
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
