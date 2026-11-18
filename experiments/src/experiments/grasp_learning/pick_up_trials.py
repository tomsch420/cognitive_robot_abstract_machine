"""
Trying whole pick-ups of an object from a starting world stored in the database: the
base moves, the gripper opens, comes down onto a grasp in straight lines, closes with a
force, waits and leaves the way it came. What a trial varies by is a
:class:`PickUpParameters`, drawn from a :class:`PickUpParameterSpace` by a model.

Run it with ``python -m experiments.grasp_learning.pick_up_trials``; see ``--help``.
"""

from __future__ import annotations

import argparse
import logging
import math
from dataclasses import dataclass, field
from datetime import timedelta

import numpy as np
from typing_extensions import Iterator

from coraplex.robot_plans.actions.core.robot_body import MoveFingersToWidthAction
from coraplex.robot_plans.mixins import HasApproachesGraspPoses
from cramph.node import StatechartNode
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.exceptions import (
    ParameterWithoutCentre,
    StartingWorldDoesNotMatch,
)
from experiments.grasp_learning.records import (
    PickUpParameters,
    PickUpStage,
    PickUpTrial,
    StandOffset,
)
from experiments.grasp_learning.kitchen_grasp_trials import (
    AttemptRecording,
    GraspOutcomeMeasurement,
    KitchenAroundTheObject,
)
from experiments.robocasa_atomic_tasks.catalogue import TaskModule
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.skills import (
    MoveTheToolInAStraightLineAction,
    StandAction,
)
from experiments.robocasa_atomic_tasks.stored_runs import (
    PlanRecorder,
    SimulatedMomentRecorder,
    StoredTaskRun,
)
from experiments.robocasa_atomic_tasks.tasks import PickAndPlaceTask
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from krrood.bounds import Bounds
from krrood.entity_query_language.backends import ProbabilisticBackend
from krrood.entity_query_language.factories import ConditionType, a
from krrood.entity_query_language.query.match import Match
from krrood.parametrization.exceptions import UnboundedParameterError
from krrood.parametrization.model_registries import (
    ModelRegistry,
    UniformPriorRegistry,
)
from krrood.parametrization.parameterizer import ModelQueryParameters
from probabilistic_model.distributions.gaussian import GaussianDistribution
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
    ProductUnit,
    SumUnit,
    leaf,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidCollisionBetweenGroups,
)
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.grasp_strategies import GraspFromAnySide
from semantic_digital_twin.grasping.object_relative_grasp import GraspResult
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

# %% the values a pick-up may take


@dataclass(kw_only=True)
class PickUpParameterSpace:
    """
    The values every parameter of a pick-up may take; each upper bound is excluded.
    """

    forward: Bounds[float] = field(default_factory=lambda: Bounds(lower=0.0, upper=0.3))
    """
    The values of :attr:`StandOffset.forward`.
    """

    sideways: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=-0.15, upper=0.1)
    )
    """
    The values of :attr:`StandOffset.sideways`.
    """

    turn: Bounds[float] = field(default_factory=lambda: Bounds(lower=-0.3, upper=0.3))
    """
    The values of :attr:`StandOffset.turn`.
    """

    torso_height: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.0, upper=0.34)
    )
    """
    The values of :attr:`PickUpParameters.torso_height`.
    """

    pitch: Bounds[float] = field(default_factory=lambda: Bounds(lower=0.0, upper=1.2))
    """
    How far the approach tilts off straight down, in radians, towards the side the
    gripper comes from.
    """

    height: Bounds[float] = field(default_factory=lambda: Bounds(lower=0.3, upper=0.95))
    """
    The heights to grasp at, as fractions of the object's grasp surface from its bottom
    to its top.
    """

    side: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=-0.75 * math.pi, upper=0.25 * math.pi)
    )
    """
    The directions the gripper comes from, as the yaw of
    :class:`~semantic_digital_twin.grasping.surface_grasp.SurfaceGrasp`'s
    ``root_V_outward`` about the world's z-axis, in radians: by default the half of the
    object facing the robot's start, from its front to its right.
    """

    opening: Bounds[float] = field(default_factory=lambda: Bounds(lower=0.6, upper=1.0))
    """
    The values of :attr:`PickUpParameters.opening`.
    """

    closing_force: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=5.0, upper=100.0)
    )
    """
    The values of :attr:`PickUpParameters.closing_force`.
    """

    approach_clearance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.02, upper=0.12)
    )
    """
    The values of :attr:`PickUpParameters.approach_clearance`.
    """

    approach_velocity: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.02, upper=0.15)
    )
    """
    The values of :attr:`PickUpParameters.approach_velocity`.
    """

    settling_duration: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.0, upper=1.0)
    )
    """
    The values of :attr:`PickUpParameters.settling_duration`.
    """

    lift_velocity: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.02, upper=0.2)
    )
    """
    The values of :attr:`PickUpParameters.lift_velocity`.
    """

    def statement(self, *, graspable: HasGraspCandidates) -> Match[PickUpParameters]:
        """
        :param graspable: The object picked up.
        :return: The statement of every pick-up this space allows on the object.
        """
        parameters = a(PickUpParameters)(
            stand_offset=a(StandOffset)(forward=..., sideways=..., turn=...),
            torso_height=...,
            grasp=SurfaceGrasp.any_pattern(),
            opening=...,
            closing_force=...,
            approach_clearance=...,
            approach_velocity=...,
            settling_duration=...,
            lift_velocity=...,
        )
        # the conditions reach every parameter from the outermost pattern, which the
        # probabilistic backend needs to bound the nested ones
        bounded = [
            (parameters.stand_offset.forward, self.forward),
            (parameters.stand_offset.sideways, self.sideways),
            (parameters.stand_offset.turn, self.turn),
            (parameters.torso_height, self.torso_height),
            (parameters.opening, self.opening),
            (parameters.closing_force, self.closing_force),
            (parameters.approach_clearance, self.approach_clearance),
            (parameters.approach_velocity, self.approach_velocity),
            (parameters.settling_duration, self.settling_duration),
            (parameters.lift_velocity, self.lift_velocity),
        ]
        conditions: list[ConditionType] = [
            condition
            for variable, bounds in bounded
            for condition in (variable >= bounds.lower, variable < bounds.upper)
        ]
        world_T_object = graspable.root.global_pose.to_np()
        object_yaw = math.atan2(world_T_object[1, 0], world_T_object[0, 0])
        grasp_strategy = GraspFromAnySide(
            pitch=self.pitch,
            height=self.height,
            yaw=Bounds(
                lower=self.side.lower - object_yaw, upper=self.side.upper - object_yaw
            ),
        )
        conditions.extend(
            grasp_strategy.conditions(parameters.grasp, graspable.grasped_part())
        )
        return parameters.where(*conditions)

    def drawn_parameters(
        self,
        *,
        graspable: HasGraspCandidates,
        number_of_pick_ups: int,
        model_registry: ModelRegistry,
    ) -> list[PickUpParameters]:
        """
        :param graspable: The object picked up.
        :param number_of_pick_ups: How many pick-ups to draw.
        :param model_registry: Answers the statement of :meth:`statement`.
        :return: Pick-ups answering the statement whose grasp reaches the object's
            surface; fewer than asked for where some do not.
        """
        drawn = (
            self.statement(graspable=graspable)
            .limit(n=number_of_pick_ups)
            .evaluate(backend=ProbabilisticBackend(model_registry=model_registry))
        )
        part = graspable.grasped_part()
        return [
            parameters
            for parameters in drawn
            if parameters.grasp.reaches_surface_of(part)
        ]


# %% drawing pick-ups near successful ones


@dataclass(kw_only=True)
class NearSuccessfulPickUps(ModelRegistry):
    """
    Answers a statement of pick-ups with an equal mixture of one Gaussian around each of
    :attr:`centres`, whose standard deviation along every parameter is :attr:`spread`
    times the range the statement allows that parameter.
    """

    centres: list[PickUpParameters]
    """
    The pick-ups the draws are biased towards, such as ones that lifted the object.
    """

    spread: float = 0.1
    """
    The standard deviation of each Gaussian along a parameter, as a fraction of the
    range the statement allows the parameter.
    """

    def get_model(self, *, parameters: ModelQueryParameters) -> ProbabilisticCircuit:
        """
        :raises UnboundedParameterError: If the statement leaves a parameter without a
            lower or an upper bound.
        :raises ParameterWithoutCentre: If a parameter of the statement is not one of
            :meth:`PickUpParameters.values_by_attribute_path`.
        """
        allowed = parameters.truncation_assignments_from_where_conditions
        if allowed is None:
            raise UnboundedParameterError(
                parameter_name=", ".join(parameters.variables)
            )
        [box] = allowed.simple_sets
        prefix = f"{parameters.queried_class.__name__}."
        circuit = ProbabilisticCircuit()
        mixture = SumUnit(probabilistic_circuit=circuit)
        for centre in self.centres:
            values = centre.values_by_attribute_path()
            component = ProductUnit(probabilistic_circuit=circuit)
            for name, variable in parameters.variables.items():
                path = name.removeprefix(prefix)
                if path not in values:
                    raise ParameterWithoutCentre(parameter_name=name)
                if variable not in box.keys():
                    raise UnboundedParameterError(parameter_name=name)
                [interval] = box[variable].simple_sets
                component.add_subcircuit(
                    leaf(
                        GaussianDistribution(
                            variable=variable,
                            location=values[path],
                            scale=self.spread * (interval.upper - interval.lower),
                        ),
                        circuit,
                    )
                )
            mixture.add_subcircuit(component, -math.log(len(self.centres)))
        return circuit


# %% the starting world


@dataclass(kw_only=True)
class StoredStartingWorld:
    """
    A world stored in the database that every pick-up starts from.
    """

    database: GraspDatabase
    """
    Where the world is stored.
    """

    world_id: int
    """
    The database id of the world.
    """

    def put_into(self, *, world: World) -> None:
        """
        Give every degree of freedom of a world the state the stored world has for it.

        :param world: A world built like the stored one, such as the episode the stored
            one was saved from.
        :raises StartingWorldDoesNotMatch: If the two worlds do not have degrees of
            freedom of the same names.
        """
        stored = self.database.starting_world(database_id=self.world_id)
        stored_state = {
            str(degree_of_freedom.name): stored.state[degree_of_freedom.id].data
            for degree_of_freedom in stored.degrees_of_freedom
        }
        degrees_of_freedom = {
            str(degree_of_freedom.name): degree_of_freedom
            for degree_of_freedom in world.degrees_of_freedom
        }
        if set(stored_state) != set(degrees_of_freedom):
            raise StartingWorldDoesNotMatch(
                world_id=self.world_id,
                missing=sorted(set(stored_state) - set(degrees_of_freedom)),
                extra=sorted(set(degrees_of_freedom) - set(stored_state)),
            )
        for name, data in stored_state.items():
            world.state[degrees_of_freedom[name].id].data[:] = data
        world.notify_state_change()


# %% performing a pick-up


@dataclass(kw_only=True)
class PickUpPerformer:
    """
    Performs pick-ups in a physically simulated episode, each from :attr:`start`, with
    the robot allowed to touch the object, the gripper allowed to touch the fixture the
    object lies in and the rest of the robot kept :attr:`fixture_margin` away from it.
    """

    episode: RoboCasaEpisode
    """
    The episode, simulated physically.
    """

    task: PickAndPlaceTask
    """
    The task the episode was composed for.
    """

    graspable: HasGraspCandidates
    """
    The object picked up.
    """

    holding_fixture_bodies: list[Body]
    """
    The bodies of the fixture the object lies in, which the robot keeps away from.
    """

    start: MujocoSimulationSnapshot
    """
    The moment of the simulation every pick-up starts from.
    """

    fixture_margin: float = 0.005
    """
    How far the robot, apart from its gripper, is kept from the fixture the object lies
    in, in metres.
    """

    hold_duration: timedelta = timedelta(seconds=2)
    """
    How long the gripper holds still at the end before the outcome is measured, so that
    an object slipping out of the fingers has time to fall.
    """

    def perform(self, *, parameters: PickUpParameters) -> PickUpTrial:
        """
        :param parameters: The pick-up to perform.
        :return: The trial, with its result; its :attr:`PickUpTrial.starting_world_id`
            is left at ``-1`` for the caller to fill in.
        """
        episode = self.episode
        simulation = episode.simulation
        simulation.restore_snapshot(snapshot=self.start)
        self.limit_finger_force(force=parameters.closing_force)
        recording = AttemptRecording(
            plan_recorder=PlanRecorder(simulation=simulation),
            moment_recorder=SimulatedMomentRecorder(
                _world=episode.world, simulation=simulation
            ),
        )
        episode.executor_extensions.append(recording.plan_recorder)
        grasp = parameters.grasp.grasp_candidate(self.graspable.grasped_part())
        measurement = GraspOutcomeMeasurement(
            episode=episode, graspable=self.graspable, grasp=grasp
        )
        robot_bodies = list(episode.robot.bodies_with_collision)
        gripper_bodies = set(episode.arm.end_effector.bodies)
        arm_bodies = [body for body in robot_bodies if body not in gripper_bodies]
        with episode.allowances.rules_applied(
            ignore_rules=[
                AllowCollisionBetweenGroups(
                    body_group_a=robot_bodies, body_group_b=[self.graspable.root]
                ),
                AllowCollisionBetweenGroups(
                    body_group_a=[
                        body for body in robot_bodies if body in gripper_bodies
                    ],
                    body_group_b=self.holding_fixture_bodies,
                ),
            ],
            default_rules=[
                AvoidCollisionBetweenGroups(
                    body_group_a=arm_bodies,
                    body_group_b=self.holding_fixture_bodies,
                    buffer_zone_distance=self.fixture_margin,
                    violated_distance=0.0,
                )
            ],
        ):
            failed_stage = self.go_through_the_stages(
                parameters=parameters, grasp=grasp, measurement=measurement
            )
        episode.let_physics_run(self.hold_duration)
        episode.executor_extensions.remove(recording.plan_recorder)
        recording.moment_recorder.finish()
        result = measurement.result(motion_completed=failed_stage is None)
        logger.info(
            "Pick-up %s: lifted %s, rose %.3f m, slipped %.3f m",
            "went through" if failed_stage is None else f"failed {failed_stage}",
            result.lifted,
            float(result.object_displacement.translation.to_np()[2]),
            float(np.linalg.norm(result.slip.translation.to_np()[:3])),
        )
        task = self.task
        return PickUpTrial(
            starting_world_id=-1,
            parameters=parameters,
            result=result,
            failed_stage=failed_stage,
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

    def go_through_the_stages(
        self,
        *,
        parameters: PickUpParameters,
        grasp: GraspCandidate,
        measurement: GraspOutcomeMeasurement,
    ) -> PickUpStage | None:
        """
        Run the plan of each stage of a pick-up in turn, until one fails.

        :param parameters: The pick-up.
        :param grasp: The grasp candidate of :attr:`PickUpParameters.grasp`.
        :param measurement: Is told when the fingers have closed.
        :return: The stage whose plan failed; ``None`` when every one went through.
        """
        episode = self.episode
        gripper = episode.arm.end_effector
        if not self.stage_succeeded(
            stage=PickUpStage.STANDING,
            steps=[
                self.torso_step(height=parameters.torso_height),
                self.stand_step(offset=parameters.stand_offset),
            ],
        ):
            return PickUpStage.STANDING
        world_T_start = self.world_pose(pose=gripper.tool_frame.global_pose)
        poses = HasApproachesGraspPoses(
            approach_clearance=parameters.approach_clearance
        ).grasp_pose_sequence(grasp.grasp_pose, gripper, grasp)
        way_in = [
            self.world_pose(pose=poses.pre_grasp),
            self.world_pose(pose=poses.grasp),
        ]
        approaching = [
            MoveFingersToWidthAction(
                gripper=gripper,
                width=parameters.opening * gripper.separation_when_open,
            ),
            *self.straight_moves(poses=way_in, velocity=parameters.approach_velocity),
        ]
        if not self.stage_succeeded(stage=PickUpStage.APPROACHING, steps=approaching):
            return PickUpStage.APPROACHING
        closing = MoveFingersToWidthAction(
            gripper=gripper, width=0.0, squeeze=gripper.squeeze
        )
        if not self.stage_succeeded(stage=PickUpStage.CLOSING, steps=[closing]):
            return PickUpStage.CLOSING
        episode.let_physics_run(timedelta(seconds=parameters.settling_duration))
        measurement.note_hold_taken()
        way_out = [way_in[0], world_T_start]
        if not self.stage_succeeded(
            stage=PickUpStage.LEAVING,
            steps=self.straight_moves(poses=way_out, velocity=parameters.lift_velocity),
        ):
            return PickUpStage.LEAVING
        return None

    def stage_succeeded(
        self, *, stage: PickUpStage, steps: list[StatechartNode]
    ) -> bool:
        """
        :param stage: The stage the steps make up.
        :param steps: The steps to run.
        :return: Whether the plan of the steps succeeded; a failure is logged.
        """
        outcome = self.episode.run(steps)
        if not outcome.succeeded:
            logger.info(
                "Stage %s failed with %s: %s",
                stage,
                type(outcome.failure).__name__,
                " | ".join(str(outcome.failure).splitlines())[:1500],
            )
        return outcome.succeeded

    def torso_step(self, *, height: float) -> JointPositionList:
        """
        :param height: The position the torso's lift joint is to reach, in metres.
        :return: A new step moving the torso there.
        """
        [lift] = [
            connection
            for connection in self.episode.robot.get_torso().connections
            if isinstance(connection, ActiveConnection1DOF)
        ]
        return JointPositionList(
            goal_state=JointState(connections=[lift], target_values=[height])
        )

    def stand_step(self, *, offset: StandOffset) -> StandAction:
        """
        :param offset: How far the base is to move from where it stands now.
        :return: A new step moving the base there.
        """
        world_T_base = self.episode.robot.root.global_pose.to_np()
        base_T_stand = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=offset.forward, y=offset.sideways, yaw=offset.turn
        ).to_np()
        return StandAction(
            target_location=HomogeneousTransformationMatrix(
                world_T_base @ base_T_stand, reference_frame=self.episode.world.root
            ).pose
        )

    def straight_moves(
        self, *, poses: list[Pose], velocity: float
    ) -> list[StatechartNode]:
        """
        :param poses: Where the tool frame goes, in turn.
        :param velocity: How fast it goes, in m/s.
        :return: New steps moving the tool frame in a straight line to each pose.
        """
        return [
            MoveTheToolInAStraightLineAction(
                arm=self.episode.arm, target_pose=pose, max_linear_velocity=velocity
            )
            for pose in poses
        ]

    def world_pose(self, *, pose: Pose) -> Pose:
        """
        :param pose: A pose in any frame.
        :return: The same pose in the world's root frame, so it stays where it is when
            the frame it was given in moves.
        """
        world = self.episode.world
        return world.transform(spatial_object=pose, target_frame=world.root)

    def limit_finger_force(self, *, force: float) -> None:
        """
        Let each finger of the arm's gripper press with at most a force.

        :param force: The force, in newtons.
        """
        gripper = self.episode.arm.end_effector
        finger_degrees_of_freedom = {
            degree_of_freedom.id
            for connection in self.episode.world.connections
            if connection.child in gripper.bodies
            for degree_of_freedom in connection.dofs
        }
        synchronizer = self.episode.simulation.synchronizer
        for actuator in self.episode.world.actuators:
            if not any(
                degree_of_freedom.id in finger_degrees_of_freedom
                for degree_of_freedom in actuator.dofs
            ):
                continue
            self.episode.simulation.simulator.set_actuator_force_range(
                actuator_name=synchronizer.element_names.of(actuator),
                force_range=np.array([-force, force]),
            )


# %% the trials


@dataclass(kw_only=True)
class PickUpTrials:
    """
    Tries pick-ups drawn from :attr:`parameter_space` from a stored starting world and
    stores every one as soon as it is tried.
    """

    kitchen: KitchenAroundTheObject
    """
    The episode the starting world was saved from.
    """

    starting_world: StoredStartingWorld
    """
    The world every pick-up starts from.
    """

    number_of_trials: int = 10
    """
    How many pick-ups are drawn.
    """

    parameter_space: PickUpParameterSpace = field(default_factory=PickUpParameterSpace)
    """
    What the pick-ups are drawn from.
    """

    model_registry: ModelRegistry = field(default_factory=UniformPriorRegistry)
    """
    Answers the statement of the pick-ups.
    """

    def run(self) -> Iterator[PickUpTrial]:
        """
        :return: The stored trials, as they are tried.
        """
        episode = self.kitchen.compose()
        graspable = episode.graspable(self.kitchen.task.picked_object)
        self.starting_world.put_into(world=episode.world)
        holding_fixture_bodies = episode.fixtures.bodies_of(
            self.kitchen.holding_fixture(episode)
        )
        drawn = self.parameter_space.drawn_parameters(
            graspable=graspable,
            number_of_pick_ups=self.number_of_trials,
            model_registry=self.model_registry,
        )
        with episode.executed(physics=True):
            performer = PickUpPerformer(
                episode=episode,
                task=self.kitchen.task,
                graspable=graspable,
                holding_fixture_bodies=holding_fixture_bodies,
                start=episode.simulation.take_snapshot(),
            )
            for parameters in drawn:
                trial = performer.perform(parameters=parameters)
                trial.starting_world_id = self.starting_world.world_id
                self.starting_world.database.add_trial(trial=trial)
                yield trial


# %% the command line


def main() -> None:
    """
    Try pick-ups from a stored starting world and report how many lifted the object.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", required=True, help="the database URI")
    parser.add_argument(
        "--starting-world",
        type=int,
        required=True,
        help="the database id of the world every pick-up starts from",
    )
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
        "--draw-seed", type=int, default=0, help="seeds the drawing of the pick-ups"
    )
    parser.add_argument("--attempts", type=int, default=PickUpTrials.number_of_trials)
    parser.add_argument(
        "--near-trials",
        type=int,
        nargs="+",
        help="the database ids of stored trials to draw the pick-ups near; drawn "
        "uniformly from the whole space when left out",
    )
    parser.add_argument(
        "--spread",
        type=float,
        default=NearSuccessfulPickUps.spread,
        help="how far the pick-ups spread around the trials, as a fraction of the "
        "range of each parameter",
    )
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.INFO, force=True)
    # the probabilistic models draw from numpy's global random state
    np.random.seed(arguments.draw_seed)
    database = GraspDatabase.connect(arguments.database)
    model_registry = (
        UniformPriorRegistry()
        if not arguments.near_trials
        else NearSuccessfulPickUps(
            centres=[
                database.pick_up_trial(database_id=trial_id).parameters
                for trial_id in arguments.near_trials
            ],
            spread=arguments.spread,
        )
    )
    trials = PickUpTrials(
        kitchen=KitchenAroundTheObject(
            task=tasks[arguments.task](
                robot=arguments.robot, seed=arguments.seed, physics=True
            )
        ),
        starting_world=StoredStartingWorld(
            database=database, world_id=arguments.starting_world
        ),
        number_of_trials=arguments.attempts,
        model_registry=model_registry,
    )
    tried = list(trials.run())
    lifted = sum(trial.result.lifted for trial in tried)
    print(f"{lifted} of {len(tried)} pick-ups lifted the object")


if __name__ == "__main__":
    main()
