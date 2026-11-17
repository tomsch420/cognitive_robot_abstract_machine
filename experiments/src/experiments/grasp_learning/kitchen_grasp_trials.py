"""
Trying grasps on an object of a RoboCasa kitchen cut down to the fixture the object lies
in, in MuJoCo: before every attempt the object is dropped somewhere inside the fixture,
and the robot walks to a stand it chooses and lifts the object by a grasp drawn from
where the object's annotation allows. Every attempt is stored with its plans and with
the world's state as it moved.

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
from random_events.interval import closed
from typing_extensions import TYPE_CHECKING, Iterator

from coraplex.plans.executors import PlanOutcome
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.exceptions import ObjectInNoFixture
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
    RecordedGraspAttempt,
)
from experiments.physical_pick_up.robots import ObjectPlacement, PickUpArea
from experiments.robocasa_atomic_tasks.catalogue import TaskModule
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.placement import FixtureRegion
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.skills import StandAndPickUpAction
from experiments.robocasa_atomic_tasks.stored_runs import (
    PlanRecorder,
    SimulatedMomentRecorder,
)
from experiments.robocasa_atomic_tasks.tasks import PickAndPlaceTask
from krrood.parametrization.model_registries import UniformPriorRegistry
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Twist
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.grasp_trials import GraspPerformer, GraspTrials
from semantic_digital_twin.grasping.object_relative_grasp import GraspResult
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
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

    def compose(self) -> EpisodeAroundTheObject:
        """
        :return: A new episode of :attr:`task`, cut down, with the fixture the picked
            object lies in.
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
        return EpisodeAroundTheObject(
            episode=RoboCasaEpisode(
                environment=episode.environment,
                task=episode.task,
                robot=episode.robot,
                task_robot=episode.task_robot,
            ),
            fixture=fixture,
        )

    def holding_fixture(self, episode: RoboCasaEpisode) -> Fixture:
        """
        :param episode: An episode of :attr:`task` whose kitchen is whole.
        :return: The smallest fixture whose bounding box holds the middle of the picked
            object.
        :raises ObjectInNoFixture: If no fixture's box holds it.
        """
        middle = episode.object_body(self.task.picked_object).global_pose.to_np()[:3, 3]
        volumes = {}
        for fixture in episode.environment.fixtures.values():
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


@dataclass(kw_only=True)
class EpisodeAroundTheObject:
    """
    An episode whose kitchen is cut down around the fixture the picked object lies in.
    """

    episode: RoboCasaEpisode
    """
    The cut down episode.
    """

    fixture: Fixture
    """
    The fixture the picked object lies in.
    """

    def inside_of_the_fixture(
        self, graspable: HasGraspCandidates, margin: float = 0.03
    ) -> PickUpArea:
        """
        :param graspable: The object picked up.
        :param margin: How far the object's footprint keeps from the sides of the
            inside, in metres, beyond half the diagonal of its bounding box.
        :return: Where in the lowest inside region of :attr:`fixture` the middle of the
            object's footprint may lie however the object is turned.
        """
        regions = self.fixture.get_int_sites(relative=False)
        region = FixtureRegion.from_corners(
            min(regions.values(), key=lambda corners: corners[0][2])
        )
        corners = np.array([region.corner, region.along_width, region.along_depth])
        object_box = self.episode.bounds.of_body(graspable.root)
        reach = (
            float(np.linalg.norm(object_box.upper[:2] - object_box.lower[:2])) / 2
            + margin
        )
        lower = corners[:, :2].min(axis=0) + reach
        upper = corners[:, :2].max(axis=0) - reach
        return PickUpArea(
            height=float(region.corner[2]),
            x=closed(lower[0], upper[0]).simple_sets[0],
            y=closed(lower[1], upper[1]).simple_sets[0],
        )


# %% performing a grasp in the kitchen


@dataclass(kw_only=True)
class AttemptRecording:
    """
    What one attempt left behind besides its result: where the object lay, the plans and
    the world's state as it moved.
    """

    placement: ObjectPlacement
    """
    Where the object was dropped.
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
    to :attr:`start` and the object dropped at a placement drawn inside the fixture; the
    robot then walks to a stand it chooses facing the object, picks it up by the grasp
    and parks its arms, and holds still for :attr:`hold_duration`.
    """

    episode: RoboCasaEpisode
    """
    The episode, simulated physically.
    """

    graspable: HasGraspCandidates
    """
    The object picked up.
    """

    area: PickUpArea
    """
    Where the middle of the object's footprint may come to lie.
    """

    generator: np.random.Generator
    """
    Draws the placements.
    """

    start: MujocoSimulationSnapshot
    """
    The moment of the simulation every attempt starts from.
    """

    maximum_yaw: float = math.pi
    """
    How far the object may be turned either way about the vertical axis, in radians.
    """

    drop_height: float = 0.02
    """
    How far above the floor of the area the object's lowest point starts, in metres.
    """

    settling_duration: timedelta = timedelta(seconds=1)
    """
    How long the dropped object is given to come to rest before the robot moves.
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
    What each attempt so far left behind, in order.
    """

    def perform(self, grasp: GraspCandidate) -> GraspResult:
        episode = self.episode
        simulation = episode.simulation
        simulation.restore_snapshot(snapshot=self.start)
        placement = self.area.random_placement(self.generator, self.maximum_yaw)
        self.drop_object(placement)
        simulation.step_simulation(self.settling_duration)
        recording = AttemptRecording(
            placement=placement,
            plan_recorder=PlanRecorder(simulation=simulation),
            moment_recorder=SimulatedMomentRecorder(
                _world=episode.world, simulation=simulation
            ),
        )
        self.recordings.append(recording)
        episode.executor_extensions.append(recording.plan_recorder)
        world_T_resting_object = self.world_T_object()
        picked_up = self.pick_up(grasp)
        gripper_T_grasp_when_picked_up = self.gripper_T_grasp(grasp)
        parked = (
            picked_up.succeeded
            and episode.run([episode.create_parking_step()]).succeeded
        )
        episode.let_physics_run(self.hold_duration)
        episode.executor_extensions.remove(recording.plan_recorder)
        recording.moment_recorder.finish()
        world_T_held_object = self.world_T_object()
        displacement = np.eye(4)
        displacement[:3, 3] = world_T_held_object[:3, 3] - world_T_resting_object[:3, 3]
        displacement[:3, :3] = (
            world_T_held_object[:3, :3] @ world_T_resting_object[:3, :3].T
        )
        slip = np.linalg.inv(gripper_T_grasp_when_picked_up) @ self.gripper_T_grasp(
            grasp
        )
        result = GraspResult(
            object_raised=bool(displacement[2, 3] >= self.minimum_rise),
            motion_completed=parked,
            object_displacement=Twist.from_transformation(
                HomogeneousTransformationMatrix(displacement)
            ),
            slip=Twist.from_transformation(HomogeneousTransformationMatrix(slip)),
        )
        logger.info(
            "Attempt %d at %s: lifted %s, rose %.3f m, slipped %.3f m",
            len(self.recordings),
            placement,
            result.lifted,
            displacement[2, 3],
            float(np.linalg.norm(slip[:3, 3])),
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

    def drop_object(self, placement: ObjectPlacement) -> None:
        """
        Put the object, at rest, where the middle of its footprint lies at the
        placement, its lowest point :attr:`drop_height` above the floor of the area.

        :param placement: Where the object is dropped.
        """
        body = self.graspable.root
        lower, upper = body.collision.combined_mesh.bounds
        world_T_object = (
            HomogeneousTransformationMatrix.from_xyz_rpy(
                x=placement.x,
                y=placement.y,
                z=self.area.height - lower[2] + self.drop_height,
                yaw=placement.yaw,
            ).to_np()
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=-(lower[0] + upper[0]) / 2, y=-(lower[1] + upper[1]) / 2
            ).to_np()
        )
        world = self.episode.world
        body.parent_connection.origin = HomogeneousTransformationMatrix(
            world_T_object, reference_frame=world.root
        )

    def world_T_object(self) -> np.ndarray:
        """
        :return: The pose of the object, as the simulation last left it.
        """
        return self.graspable.root.global_pose.to_np()

    def gripper_T_grasp(self, grasp: GraspCandidate) -> np.ndarray:
        """
        :param grasp: A grasp on the object.
        :return: The pose of the grasp frame, fixed to the object, in the tool frame.
        """
        world_T_tool = self.episode.arm.end_effector.tool_frame.global_pose.to_np()
        return (
            np.linalg.inv(world_T_tool)
            @ self.world_T_object()
            @ grasp.grasp_pose.to_np()
        )


# %% the trials


@dataclass(kw_only=True)
class KitchenGraspTrials:
    """
    Tries grasps drawn from where the picked object's annotation allows on the object of
    a pick and place task, in its kitchen cut down to the fixture the object lies in,
    and stores every attempt with what it left behind.
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

    seed: int = 0
    """
    Seeds the placements of the object.
    """

    def run(self) -> Iterator[RecordedGraspAttempt]:
        """
        Try the grasps and store each attempt as soon as it is tried.

        :return: The stored attempts, as they are tried.
        """
        around_the_object = self.kitchen.compose()
        episode = around_the_object.episode
        task = self.kitchen.task
        graspable = episode.graspable(task.picked_object)
        with episode.executed(physics=True):
            episode.run([episode.create_parking_step()]).raise_if_failed()
            performer = KitchenGraspPerformer(
                episode=episode,
                graspable=graspable,
                area=around_the_object.inside_of_the_fixture(graspable),
                generator=np.random.default_rng(self.seed),
                start=episode.simulation.take_snapshot(),
            )
            trials = GraspTrials(
                graspable=graspable,
                performer=performer,
                number_of_trials=self.number_of_attempts,
                model_registry=UniformPriorRegistry(),
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
                        placement=recording.placement,
                        grasp=record.grasp,
                    ),
                    degree_of_freedom_names=recording.moment_recorder.degree_of_freedom_names,
                    moments=recording.moment_recorder.moments,
                    plans=recording.plan_recorder.plans,
                )
                self.database.add_attempts([recorded])
                yield recorded


# %% the command line


def main() -> None:
    """
    Try grasps on the picked object of a pick and place task from the command line and
    report how many lifted it.
    """
    parser = argparse.ArgumentParser(description=__doc__)
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
        "--placement-seed",
        type=int,
        default=KitchenGraspTrials.seed,
        help="seeds where the object is dropped",
    )
    parser.add_argument(
        "--attempts", type=int, default=KitchenGraspTrials.number_of_attempts
    )
    parser.add_argument("--database", required=True, help="the database URI")
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.INFO, force=True)
    trials = KitchenGraspTrials(
        kitchen=KitchenAroundTheObject(
            task=tasks[arguments.task](
                robot=arguments.robot, seed=arguments.seed, physics=True
            )
        ),
        database=GraspDatabase.connect(arguments.database),
        number_of_attempts=arguments.attempts,
        seed=arguments.placement_seed,
    )
    attempts = list(trials.run())
    lifted = sum(recorded.attempt.grasp.result.lifted for recorded in attempts)
    print(f"{lifted} of {len(attempts)} attempts lifted the object")


if __name__ == "__main__":
    main()
