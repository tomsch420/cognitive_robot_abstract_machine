"""
Trying grasps on an object of a RoboCasa kitchen cut down to the fixture the object lies
in, in MuJoCo: every attempt starts with the object where RoboCasa put it, and the robot
walks to a stand it chooses and lifts the object by a grasp drawn from
:attr:`KitchenGraspTrials.grasp_strategy`. Every attempt is stored with its plans and
with the world's state as it moved, and can be replayed with
``experiments/scripts/grasp_learning/replay_kitchen_grasp_attempt.py``.

Run it with ``python -m experiments.grasp_learning.kitchen_grasp_trials``; see
``--help``.
"""

from __future__ import annotations

import argparse
import logging
import math
from dataclasses import dataclass, field
from datetime import timedelta

import numpy as np
from typing_extensions import TYPE_CHECKING, Iterator

from coraplex.plans.executors import PlanOutcome
from coraplex.robot_plans.actions.core.pick_up import GraspingAction
from coraplex.robot_plans.plan_transformations import CloseFingersToObjectWidth
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.exceptions import ObjectInNoFixture
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
    RecordedGraspAttempt,
)
from experiments.physical_pick_up.robots import ObjectPlacement
from experiments.robocasa_atomic_tasks.catalogue import TaskModule
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.skills import StandAndPickUpAction
from experiments.robocasa_atomic_tasks.stored_runs import (
    PlanRecorder,
    SimulatedMomentRecorder,
    StoredTaskRun,
    TaskRunReplay,
)
from experiments.robocasa_atomic_tasks.tasks import PickAndPlaceTask
from krrood.bounds import Bounds
from krrood.parametrization.model_registries import UniformPriorRegistry
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Twist
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.grasp_strategies import (
    GraspFromAnySide,
    GraspStrategy,
)
from semantic_digital_twin.grasping.grasp_trials import GraspPerformer, GraspTrials
from semantic_digital_twin.grasping.object_relative_grasp import GraspResult
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable
from semantic_digital_twin.world_description.geometry import CoordinateBounds
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

if TYPE_CHECKING:
    from robocasa.models.fixtures import Fixture

logger = logging.getLogger(__name__)

# %% the kitchen cut down to one fixture


@dataclass(kw_only=True)
class KitchenAroundTheObject:
    """
    An episode of a pick and place task whose kitchen is cut down to the fixture the
    picked object lies in and to what lies within :attr:`neighbour_distance` of that
    fixture's footprint below its top, such as the counter it is set into, the cabinet
    below it and the wall behind it.

    The robot and the picked object stay.
    """

    task: PickAndPlaceTask
    """
    The task whose episode is composed.
    """

    neighbour_distance: float = 0.05
    """
    How far apart, horizontally, the footprints of the fixture and of a body that stays
    may lie at most, in metres.
    """

    def compose(self) -> RoboCasaEpisode:
        """
        :return: A new episode of :attr:`task`, cut down.
        """
        episode = self.task.compose()
        fixture = self.holding_fixture(episode)
        fixture_box = episode.bounds.of_branch(episode.fixture_body(fixture))
        world = episode.world
        kept = {episode.robot.root, episode.object_body(self.task.picked_object)}
        removed = [
            entity
            for entity in world.root.child_kinematic_structure_entities
            if entity not in kept and not self.stays_with(episode, entity, fixture_box)
        ]
        removed_entities = {
            part
            for entity in removed
            for part in world.get_kinematic_structure_entities_of_branch(entity)
        }
        with world.modify_world():
            for annotation in list(world.semantic_annotations):
                if annotation.root in removed_entities:
                    world.remove_semantic_annotation(annotation)
            for entity in removed:
                world.remove_branch_from_world(entity)
        logger.info(
            "Kept %d of the kitchen's bodies for: %s",
            len(world.bodies),
            episode.task.instruction,
        )
        return RoboCasaEpisode(
            environment=episode.environment,
            task=episode.task,
            robot=episode.robot,
            task_robot=episode.task_robot,
        )

    def holding_fixture(self, episode: RoboCasaEpisode) -> Fixture:
        """
        :param episode: An episode of :attr:`task`, whole or cut down.
        :return: The smallest fixture still in the episode's world whose bounding box
            holds the middle of the picked object.
        :raises ObjectInNoFixture: If no fixture's box holds it.
        """
        middle = episode.object_body(self.task.picked_object).global_pose.to_np()[:3, 3]
        body_names = {body.name.name for body in episode.world.bodies}
        volumes = {}
        for fixture in episode.environment.fixtures.values():
            if fixture.root_body not in body_names:
                continue
            body = episode.fixture_body(fixture)
            if not self.collides(episode, body):
                continue
            box = episode.bounds.of_branch(body)
            if np.all(box.lower <= middle) and np.all(middle <= box.upper):
                volumes[fixture] = float(np.prod(box.upper - box.lower))
        if not volumes:
            raise ObjectInNoFixture(object_name=self.task.picked_object)
        return min(volumes, key=volumes.get)

    @staticmethod
    def collides(episode: RoboCasaEpisode, entity: KinematicStructureEntity) -> bool:
        """
        :param episode: An episode.
        :param entity: An entity of its world.
        :return: Whether a body of the branch the entity is the root of has collision
            geometry.
        """
        return any(
            isinstance(part, Body) and part.has_collision()
            for part in episode.world.get_kinematic_structure_entities_of_branch(entity)
        )

    def stays_with(
        self,
        episode: RoboCasaEpisode,
        entity: KinematicStructureEntity,
        fixture_box: CoordinateBounds,
    ) -> bool:
        """
        :param episode: The episode being cut down.
        :param entity: A body fixed to the world's root, with what hangs from it.
        :param fixture_box: The bounding box of the fixture the picked object lies in.
        :return: Whether the body collides with something, lies within
            :attr:`neighbour_distance` of the fixture's footprint and reaches below the
            fixture's top.
        """
        if not self.collides(episode, entity):
            return False
        box = episode.bounds.of_branch(entity)
        gap = np.maximum(
            np.maximum(fixture_box.lower[:2] - box.upper[:2], 0.0),
            np.maximum(box.lower[:2] - fixture_box.upper[:2], 0.0),
        )
        return bool(
            np.linalg.norm(gap) <= self.neighbour_distance
            and box.lower[2] < fixture_box.upper[2]
        )


# %% closing on an object wider than the gripper


@dataclass
class CloseFingersToObjectWidthWhereItFits(CloseFingersToObjectWidth):
    """
    Differs from :class:`CloseFingersToObjectWidth` in leaving a grasp on an object
    wider than the gripper opens to shut the fingers onto whatever part of the object
    lies between them.
    """

    def is_applicable(self, plan_node: GraspingAction) -> bool:
        return (
            super().is_applicable(plan_node)
            and plan_node.grasp.width_along_closing_axis
            <= plan_node.arm.end_effector.separation_when_open
        )


# %% performing a grasp in the kitchen


@dataclass(kw_only=True)
class GraspOutcomeMeasurement:
    """
    Measures how an object moved from where it rested when the measurement was made, and
    how it slipped in the gripper from when the gripper took hold of it.
    """

    episode: RoboCasaEpisode
    """
    The episode the object is grasped in.
    """

    graspable: IsGraspable
    """
    The object grasped.
    """

    grasp: GraspCandidate
    """
    The grasp the gripper takes the object by.
    """

    minimum_rise: float = 0.05
    """
    How far the object has to have risen to count as raised, in metres.
    """

    world_T_resting_object: np.ndarray = field(init=False)
    """
    The pose of the object when the measurement was made.
    """

    gripper_T_grasp_when_taken: np.ndarray = field(init=False)
    """
    The pose of the grasp frame, fixed to the object, in the tool frame, when the
    gripper took hold; as when the measurement was made until then.
    """

    def __post_init__(self) -> None:
        self.world_T_resting_object = self.world_T_object()
        self.gripper_T_grasp_when_taken = self.gripper_T_grasp()

    def note_hold_taken(self) -> None:
        """
        Remember where the object lies in the gripper now, which slip is measured from.
        """
        self.gripper_T_grasp_when_taken = self.gripper_T_grasp()

    def result(self, *, motion_completed: bool) -> GraspResult:
        """
        :param motion_completed: Whether the robot went through every step.
        :return: What happened to the object, as the simulation last left it.
        """
        world_T_held_object = self.world_T_object()
        displacement = np.eye(4)
        displacement[:3, 3] = (
            world_T_held_object[:3, 3] - self.world_T_resting_object[:3, 3]
        )
        displacement[:3, :3] = (
            world_T_held_object[:3, :3] @ self.world_T_resting_object[:3, :3].T
        )
        slip = np.linalg.inv(self.gripper_T_grasp_when_taken) @ self.gripper_T_grasp()
        return GraspResult(
            object_raised=bool(displacement[2, 3] >= self.minimum_rise),
            motion_completed=motion_completed,
            object_displacement=Twist.from_transformation(
                HomogeneousTransformationMatrix(displacement)
            ),
            slip=Twist.from_transformation(HomogeneousTransformationMatrix(slip)),
        )

    def world_T_object(self) -> np.ndarray:
        """
        :return: The pose of the object, as the simulation last left it.
        """
        return self.graspable.root.global_pose.to_np()

    def gripper_T_grasp(self) -> np.ndarray:
        """
        :return: The pose of the grasp frame, fixed to the object, in the tool frame.
        """
        world_T_tool = self.episode.arm.end_effector.tool_frame.global_pose.to_np()
        return (
            np.linalg.inv(world_T_tool)
            @ self.world_T_object()
            @ self.grasp.grasp_pose.to_np()
        )


@dataclass(kw_only=True)
class AttemptRecording:
    """
    The plans of one attempt and the world's state as it moved through it.
    """

    plan_recorder: PlanRecorder
    """
    The plans of the attempt.
    """

    moment_recorder: SimulatedMomentRecorder
    """
    The world's state through the attempt.
    """


@dataclass(kw_only=True)
class KitchenGraspPerformer(GraspPerformer):
    """
    Performs every grasp in a physically simulated episode: the simulation is put back
    to :attr:`start`, the robot walks to a stand it chooses facing the object, picks the
    object up by the grasp and parks its arms, and holds still for
    :attr:`hold_duration`.
    """

    episode: RoboCasaEpisode
    """
    The episode, simulated physically.
    """

    graspable: IsGraspable
    """
    The object picked up.
    """

    start: MujocoSimulationSnapshot
    """
    The moment of the simulation every attempt starts from, with the object at rest.
    """

    hold_duration: timedelta = timedelta(seconds=2)
    """
    How long the parked arm holds the object before the outcome is measured, so that an
    object slipping out of the fingers has time to fall.
    """

    minimum_rise: float = 0.05
    """
    How far the object has to have risen to count as raised, in metres.
    """

    recordings: list[AttemptRecording] = field(default_factory=list, init=False)
    """
    What each attempt so far recorded, in order.
    """

    def perform(self, grasp: GraspCandidate) -> GraspResult:
        episode = self.episode
        simulation = episode.simulation
        simulation.restore_snapshot(snapshot=self.start)
        recording = AttemptRecording(
            plan_recorder=PlanRecorder(simulation=simulation),
            moment_recorder=SimulatedMomentRecorder(
                _world=episode.world, simulation=simulation
            ),
        )
        self.recordings.append(recording)
        episode.executor_extensions.append(recording.plan_recorder)
        measurement = GraspOutcomeMeasurement(
            episode=episode,
            graspable=self.graspable,
            grasp=grasp,
            minimum_rise=self.minimum_rise,
        )
        picked_up = self.pick_up(grasp)
        measurement.note_hold_taken()
        parked = (
            picked_up.succeeded
            and episode.run([episode.create_parking_step()]).succeeded
        )
        episode.let_physics_run(self.hold_duration)
        episode.executor_extensions.remove(recording.plan_recorder)
        recording.moment_recorder.finish()
        result = measurement.result(motion_completed=parked)
        logger.info(
            "Attempt %d: picked up %s, lifted %s, rose %.3f m, slipped %.3f m",
            len(self.recordings),
            picked_up.succeeded,
            result.lifted,
            float(result.object_displacement.translation.to_np()[2]),
            float(np.linalg.norm(result.slip.translation.to_np()[:3])),
        )
        return result

    def pick_up(self, grasp: GraspCandidate) -> PlanOutcome:
        """
        :param grasp: The grasp to take the object by.
        :return: How the plan ended that walks the robot to a stand facing the object,
            leaving the stand to the plan, and picks the object up by the grasp.
        """
        episode = self.episode
        position = self.graspable.root.global_pose.to_np()[:3, 3]
        stands = episode.stands.stands_facing(position)
        with episode.allowances.gripper_may_touch(
            episode.arm,
            episode.fixtures.bodies_near(
                episode.bounds.of_branch(self.graspable.root),
                margin=episode.handling_margin,
            ),
        ):
            return episode.perform_reaching(
                lambda through_free_space: [
                    episode.create_parking_step(),
                    StandAndPickUpAction.from_stands(
                        self.graspable,
                        episode.arm,
                        stands,
                        grasps=[grasp],
                        through_free_space=through_free_space,
                    ),
                ],
                retries=0,
            )


# %% the trials


@dataclass(kw_only=True)
class KitchenGraspTrials:
    """
    Tries grasps drawn from :attr:`grasp_strategy` on the object of a pick and place
    task, in its kitchen cut down to the fixture the object lies in, and stores every
    attempt with its run.
    """

    kitchen: KitchenAroundTheObject
    """
    The episode the grasps are tried in.
    """

    database: GraspDatabase
    """
    Where the attempts are stored.
    """

    number_of_attempts: int = 20
    """
    How many grasps are drawn.
    """

    grasp_strategy: GraspStrategy = field(
        default_factory=lambda: GraspFromAnySide(
            pitch=Bounds(lower=0.0, upper=math.pi / 12),
            height=Bounds(lower=0.4, upper=0.95),
        )
    )
    """
    Where the grasps are drawn from: by default from above, the approach at most 15
    degrees off straight down, on the upper part of the object.
    """

    settling_duration: timedelta = timedelta(seconds=1)
    """
    How long the object is given to come to rest where RoboCasa put it before the first
    attempt.
    """

    def run(self) -> Iterator[RecordedGraspAttempt]:
        """
        Try the grasps and store each attempt as soon as it is tried.

        :return: The stored attempts, as they are tried.
        """
        episode = self.kitchen.compose()
        task = self.kitchen.task
        graspable = episode.graspable(task.picked_object)
        episode.plan_adaptations.insert(
            0,
            CloseFingersToObjectWidthWhereItFits(
                squeeze=episode.arm.end_effector.squeeze
            ),
        )
        with episode.executed(physics=True):
            episode.run([episode.create_parking_step()]).raise_if_failed()
            episode.let_physics_run(self.settling_duration)
            world_T_object = graspable.root.global_pose.to_np()
            placement = ObjectPlacement(
                x=float(world_T_object[0, 3]),
                y=float(world_T_object[1, 3]),
                yaw=math.atan2(world_T_object[1, 0], world_T_object[0, 0]),
            )
            performer = KitchenGraspPerformer(
                episode=episode,
                graspable=graspable,
                start=episode.simulation.take_snapshot(),
            )
            trials = GraspTrials(
                graspable=graspable,
                performer=performer,
                number_of_trials=self.number_of_attempts,
                model_registry=UniformPriorRegistry(),
                grasp_strategy=self.grasp_strategy,
            )
            learning_task = GraspLearningTask.of_graspable(
                graspable, type(episode.arm.end_effector)
            )
            for record in trials.run():
                recording = performer.recordings[-1]
                recorded = RecordedGraspAttempt(
                    attempt=GraspAttempt(
                        task=learning_task,
                        robot=type(episode.robot),
                        object_name=graspable.root.name.name,
                        source=GraspSource.STATED_REGIONS,
                        placement=placement,
                        grasp=record.grasp,
                    ),
                    run=StoredTaskRun(
                        task=task.robocasa_task_name,
                        layout=task.layout,
                        style=task.style,
                        seed=task.seed,
                        robot=task.robot,
                        degree_of_freedom_names=recording.moment_recorder.degree_of_freedom_names,
                        moments=recording.moment_recorder.moments,
                        judgement=None,
                        plans=recording.plan_recorder.plans,
                    ),
                )
                self.database.add_attempts([recorded])
                yield recorded


# %% replaying a run in the kitchen cut down


def replay_run(*, run: StoredTaskRun, speed: float = 1.0) -> None:
    """
    Replay a stored run, with its plans, in its kitchen cut down anew, as fast as it
    passed in the simulation times ``speed``.

    :param run: The run, such as the run of a stored attempt or pick-up trial.
    :param speed: How many times faster than it passed the run is replayed.
    """
    task = TaskModule.tasks_by_name[run.task](
        layout=run.layout, style=run.style, seed=run.seed, robot=run.robot
    )
    episode = KitchenAroundTheObject(task=task).compose()
    # a viewer that shows the worlds plans run in, as cramera-live does, starts showing
    # this one once a plan executor is created for it
    episode.create_executor()
    TaskRunReplay(world=episode.world, run=run, robot=episode.robot).play(speed)


# %% the command line


def main() -> None:
    """
    Try grasps on the picked object of a pick and place task and report how many lifted
    it.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", required=True, help="the database URI")
    tasks = {
        name: task
        for name, task in TaskModule.tasks_by_name.items()
        if issubclass(task, PickAndPlaceTask)
    }
    parser.add_argument(
        "--task", choices=sorted(tasks), default="PickPlaceSinkToCounter"
    )
    parser.add_argument(
        "--robot",
        type=RobotModel,
        choices=list(RobotModel),
        default=RobotModel.PANDA_OMRON,
    )
    parser.add_argument(
        "--seed", type=int, default=0, help="seeds RoboCasa's choice of the objects"
    )
    parser.add_argument(
        "--draw-seed", type=int, default=0, help="seeds the drawing of the grasps"
    )
    parser.add_argument(
        "--attempts", type=int, default=KitchenGraspTrials.number_of_attempts
    )
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.INFO, force=True)
    database = GraspDatabase.connect(arguments.database)
    # the probabilistic models draw from numpy's global random state
    np.random.seed(arguments.draw_seed)
    trials = KitchenGraspTrials(
        kitchen=KitchenAroundTheObject(
            task=tasks[arguments.task](
                robot=arguments.robot, seed=arguments.seed, physics=True
            )
        ),
        database=database,
        number_of_attempts=arguments.attempts,
    )
    attempts = list(trials.run())
    lifted = sum(recorded.attempt.grasp.result.lifted for recorded in attempts)
    print(f"{lifted} of {len(attempts)} attempts lifted the object")


if __name__ == "__main__":
    main()
