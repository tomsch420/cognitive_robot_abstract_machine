"""
Trying grasps in many worker processes at once, each keeping the simulations of the
objects it tries grasps on and resetting them between attempts.
"""

from __future__ import annotations

import multiprocessing
import multiprocessing.connection
import zlib
from dataclasses import dataclass, field
from datetime import timedelta
from pathlib import Path

import numpy as np
from typing_extensions import Any, Dict, List, Optional, Tuple, Type

from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.grasp_selection import GraspChoice, PreparedModel
from experiments.grasp_learning.grasp_variables import grasp_features
from experiments.grasp_learning.models import GraspModel
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
)
from experiments.physical_pick_up.objects import (
    MeshFileObjectDescription,
    ObjectDescription,
    hollow_object_decomposition,
)
from experiments.physical_pick_up.pick_up_experiment import PickUpExperiment
from experiments.physical_pick_up.robots import ObjectPlacement, PickUpRobot
from experiments.physical_pick_up.scene import PickUpScene
from semantic_digital_twin.grasping.surface_grasp import (
    SurfaceGrasp,
    draw_surface_grasps,
)
from semantic_digital_twin.part_finding.annotation_part_finder import (
    AnnotationPartFinder,
)
from semantic_digital_twin.part_finding.locating import KnownPartLocator
from semantic_digital_twin.semantic_annotations.mixins import (
    HasGraspCandidates,
    PartWholeRelationship,
)

# %% the objects grasps are tried on


@dataclass
class IterationObject:
    """
    One object grasps are tried on, and the robot that tries them.
    """

    description: ObjectDescription
    """
    The object.
    """

    robot: PickUpRobot = PickUpRobot.PR2
    """
    The robot that picks it up.
    """

    part_finder: Optional[AnnotationPartFinder] = None
    """
    Finds the parts the object's annotation calls for, such as its handle; ``None`` asks
    the part finding server for an annotation that calls for parts.
    """

    @classmethod
    def from_mesh(
        cls,
        mesh_file: Path,
        semantic_annotation_type: Type[HasGraspCandidates],
        mass: float,
        hollow: bool = False,
        meters_per_mesh_unit: float = 1.0,
        robot: PickUpRobot = PickUpRobot.PR2,
    ) -> IterationObject:
        """
        :param mesh_file: The mesh the object looks like.
        :param semantic_annotation_type: What the object is.
        :param mass: The object's mass in kilograms.
        :param hollow: Whether the object is hollow, like a bowl or a mug, so that it
            collides as convex parts that leave it open instead of as its convex hull.
        :param meters_per_mesh_unit: The length of one unit of the mesh file, in meters.
        :param robot: The robot that picks it up.
        :return: The object, named after its mesh file.
        """
        return cls(
            description=MeshFileObjectDescription(
                body_name=mesh_file.stem,
                mesh_file=mesh_file,
                semantic_annotation_type=semantic_annotation_type,
                mass=mass,
                meters_per_mesh_unit=meters_per_mesh_unit,
                convex_decomposition=hollow_object_decomposition() if hollow else None,
            ),
            robot=robot,
        )

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
        return PickUpScene(
            object_description=self.description,
            robot_setup=self.robot.value,
            part_finder=self.parts_finder(),
        )

    def parts_finder(self) -> AnnotationPartFinder:
        """
        :return: :attr:`part_finder`; without one, a finder that asks the part finding
            server, or one that finds nothing for an annotation that calls for no parts.
        """
        if self.part_finder is not None:
            return self.part_finder
        if not issubclass(
            self.description.semantic_annotation_type, PartWholeRelationship
        ):
            return AnnotationPartFinder(locator=KnownPartLocator())
        return AnnotationPartFinder()


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
        database_uri: str,
        jobs: multiprocessing.connection.Connection,
        results: multiprocessing.connection.Connection,
    ) -> None:
        """
        Try the batches arriving on ``jobs`` until ``None`` arrives, answering each with
        its outcomes on ``results``.

        :param database_uri: The database the attempts are stored in.
        :param jobs: Where the batches arrive.
        :param results: Where the outcomes of each batch are sent, in the order the
            batches arrive.
        """
        worker = cls(database=GraspDatabase.connect(database_uri))
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

    database_uri: str
    """
    The database the workers store the attempts in.
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
            target=IterationWorker.serve,
            args=(self.database_uri, jobs_end, results_end),
            daemon=True,
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
