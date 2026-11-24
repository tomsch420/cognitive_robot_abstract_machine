"""
Trying ways of letting go of a held object that rests on its target, from a starting
world stored in the database: the fingers take hold again, open with a force, wait and
the hand moves away in a straight line. What a trial varies by is a
:class:`ReleaseParameters`, drawn from a :class:`ReleaseParameterSpace` by a model.

Run it with ``python -m experiments.grasp_learning.release_trials``; see ``--help``.
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
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.kitchen_grasp_trials import AttemptRecording
from experiments.grasp_learning.pick_up_trials import StoredStartingWorld
from experiments.grasp_learning.records import (
    ReleaseParameters,
    ReleaseResult,
    ReleaseStage,
    ReleaseTrial,
)
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.pick_place import PickPlaceSinkToCounter
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.skills import (
    BodyContacts,
    MoveTheToolInAStraightLineAction,
)
from experiments.robocasa_atomic_tasks.stored_runs import (
    PlanRecorder,
    SimulatedMomentRecorder,
    StoredTaskRun,
)
from experiments.robocasa_atomic_tasks.tasks import PickAndPlaceTask
from krrood.bounds import Bounds
from krrood.parametrization.probabilistic_backend import ProbabilisticBackend
from krrood.entity_query_language.factories import ConditionType, a
from krrood.entity_query_language.query.match import Match
from krrood.parametrization.model_registries import (
    ModelRegistry,
    UniformPriorRegistry,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

# %% the values letting go may take


@dataclass(kw_only=True)
class ReleaseParameterSpace:
    """
    The values every parameter of letting go may take; each upper bound is excluded.
    """

    opening: Bounds[float] = field(default_factory=lambda: Bounds(lower=0.6, upper=1.0))
    """
    The values of :attr:`ReleaseParameters.opening`.
    """

    opening_force: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=5.0, upper=150.0)
    )
    """
    The values of :attr:`ReleaseParameters.opening_force`.
    """

    settling_duration: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.0, upper=1.5)
    )
    """
    The values of :attr:`ReleaseParameters.settling_duration`.
    """

    retreat_tilt: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.0, upper=0.5 * math.pi)
    )
    """
    The values of :attr:`ReleaseParameters.retreat_tilt`.
    """

    retreat_turn: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=-math.pi, upper=math.pi)
    )
    """
    The values of :attr:`ReleaseParameters.retreat_turn`.
    """

    retreat_distance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.04, upper=0.15)
    )
    """
    The values of :attr:`ReleaseParameters.retreat_distance`.
    """

    retreat_velocity: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.02, upper=0.15)
    )
    """
    The values of :attr:`ReleaseParameters.retreat_velocity`.
    """

    def statement(self) -> Match[ReleaseParameters]:
        """
        :return: The statement of every way of letting go this space allows.
        """
        parameters = a(ReleaseParameters)(
            opening=...,
            opening_force=...,
            settling_duration=...,
            retreat_tilt=...,
            retreat_turn=...,
            retreat_distance=...,
            retreat_velocity=...,
        )
        bounded = [
            (parameters.opening, self.opening),
            (parameters.opening_force, self.opening_force),
            (parameters.settling_duration, self.settling_duration),
            (parameters.retreat_tilt, self.retreat_tilt),
            (parameters.retreat_turn, self.retreat_turn),
            (parameters.retreat_distance, self.retreat_distance),
            (parameters.retreat_velocity, self.retreat_velocity),
        ]
        conditions: list[ConditionType] = [
            condition
            for variable, bounds in bounded
            for condition in (variable >= bounds.lower, variable < bounds.upper)
        ]
        return parameters.where(*conditions)

    def drawn_parameters(
        self, *, number_of_releases: int, model_registry: ModelRegistry
    ) -> list[ReleaseParameters]:
        """
        :param number_of_releases: How many ways of letting go to draw.
        :param model_registry: Answers the statement of :meth:`statement`.
        :return: The drawn ways of letting go.
        """
        return list(
            self.statement()
            .limit(n=number_of_releases)
            .evaluate(backend=ProbabilisticBackend(model_registry=model_registry))
        )


# %% letting go


@dataclass(kw_only=True)
class ReleasePerformer:
    """
    Lets go of a held object resting on its target in a physically simulated episode,
    each time from :attr:`start`, with the robot allowed to touch the object and the
    gripper allowed to touch the target.
    """

    episode: RoboCasaEpisode
    """
    The episode, simulated physically.
    """

    task: PickAndPlaceTask
    """
    The task the episode was composed for.
    """

    held: Body
    """
    The object held.
    """

    target: Body
    """
    What the object rests on.
    """

    start: MujocoSimulationSnapshot
    """
    The moment of the simulation every trial starts from.
    """

    holding_force: float = 28.36
    """
    The most force each finger presses with while it takes hold again, in newtons.
    """

    taking_hold_duration: timedelta = timedelta(seconds=0.5)
    """
    How long the fingers press on the object before they open.
    """

    resting_duration: timedelta = timedelta(seconds=1)
    """
    How long the object is left to come to rest once the hand moved away.
    """

    def perform(self, *, parameters: ReleaseParameters) -> ReleaseTrial:
        """
        :param parameters: The way of letting go to try.
        :return: The trial, with its result; its :attr:`ReleaseTrial.starting_world_id`
            is left at ``-1`` for the caller to fill in.
        """
        episode = self.episode
        simulation = episode.simulation
        simulation.restore_snapshot(snapshot=self.start)
        gripper = episode.arm.end_effector
        recording = AttemptRecording(
            plan_recorder=PlanRecorder(simulation=simulation),
            moment_recorder=SimulatedMomentRecorder(
                _world=episode.world, simulation=simulation
            ),
        )
        episode.executor_extensions.append(recording.plan_recorder)
        with episode.allowances.rules_applied(
            ignore_rules=[
                AllowCollisionBetweenGroups(
                    body_group_a=list(episode.robot.bodies_with_collision),
                    body_group_b=[self.held],
                ),
                AllowCollisionBetweenGroups(
                    body_group_a=list(gripper.bodies_with_collision),
                    body_group_b=[self.target],
                ),
            ]
        ):
            episode.limit_finger_force(force=self.holding_force)
            episode.run(
                [
                    MoveFingersToWidthAction(
                        two_finger_gripper=gripper, width=0.0, squeeze=gripper.squeeze
                    )
                ]
            )
            episode.let_physics_run(self.taking_hold_duration)
            result, failed_stage = self.let_go(parameters=parameters)
        episode.executor_extensions.remove(recording.plan_recorder)
        recording.moment_recorder.finish()
        logger.info(
            "Letting go %s: separation %.4f m, carried along %.3f m, accomplished %s",
            "went through" if failed_stage is None else f"failed {failed_stage}",
            result.finger_separation_after_opening,
            result.object_carried_along,
            result.accomplished,
        )
        task = self.task
        return ReleaseTrial(
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

    def let_go(
        self, *, parameters: ReleaseParameters
    ) -> tuple[ReleaseResult, ReleaseStage | None]:
        """
        Open the fingers, wait, move the hand away and let the object come to rest.

        :param parameters: The way of letting go.
        :return: What came of it, and the part whose plan failed, ``None`` when none
            did.
        """
        episode = self.episode
        gripper = episode.arm.end_effector
        fingers = list(gripper.bodies_with_collision)
        contacts = BodyContacts(world=episode.world)
        episode.limit_finger_force(force=parameters.opening_force)
        opening = MoveFingersToWidthAction(
            two_finger_gripper=gripper, width=parameters.opening * gripper.separation_when_open
        )
        failed_stage = None
        if not episode.run_part(part=ReleaseStage.OPENING, steps=[opening]):
            failed_stage = ReleaseStage.OPENING
        episode.let_physics_run(timedelta(seconds=parameters.settling_duration))
        separation = gripper.fingertips.separation
        touched_after_opening = contacts.touch(fingers, [self.held])
        world_P_object = self.held.global_pose.to_np()[:3, 3]
        retreat = MoveTheToolInAStraightLineAction(
            arm=episode.arm,
            target_pose=self.retreat_pose(parameters=parameters),
            cruise_linear_velocity=parameters.retreat_velocity,
        )
        if failed_stage is None and not episode.run_part(
            part=ReleaseStage.RETREATING, steps=[retreat]
        ):
            failed_stage = ReleaseStage.RETREATING
        episode.let_physics_run(self.resting_duration)
        carried_along = np.linalg.norm(
            self.held.global_pose.to_np()[:3, 3] - world_P_object
        )
        episode.judge.mirror.synchronize()
        return (
            ReleaseResult(
                finger_separation_after_opening=float(separation),
                fingers_touched_the_object_after_opening=touched_after_opening,
                object_carried_along=float(carried_along),
                fingers_touched_the_object_at_the_end=contacts.touch(
                    fingers, [self.held]
                ),
                accomplished=episode.judge.pass_judgement().accomplished,
            ),
            failed_stage,
        )

    def retreat_pose(self, *, parameters: ReleaseParameters) -> Pose:
        """
        :param parameters: The way of letting go.
        :return: Where the tool frame goes when the hand moves away, in the world's root
            frame, turned as the tool frame is now.
        """
        gripper = self.episode.arm.end_effector
        tool_V_retreat = parameters.retreat_direction(
            approach_axis=gripper.approach_axis.to_np()[:3],
            closing_axis=gripper.closing_axis.to_np()[:3],
        )
        world_T_tool = gripper.tool_frame.global_pose.to_np()
        world_T_tool[:3, 3] += (
            world_T_tool[:3, :3] @ tool_V_retreat * parameters.retreat_distance
        )
        return HomogeneousTransformationMatrix(
            world_T_tool, reference_frame=self.episode.world.root
        ).pose


# %% the trials


@dataclass(kw_only=True)
class ReleaseTrials:
    """
    Tries ways of letting go drawn from :attr:`parameter_space` from a stored starting
    world of the whole kitchen and stores every one as soon as it is tried.
    """

    task: PickAndPlaceTask
    """
    The task whose episode the starting world was saved from; the picked object is the
    one held, the receptacle the target it rests on.
    """

    starting_world: StoredStartingWorld
    """
    The world every trial starts from.
    """

    number_of_trials: int = 10
    """
    How many ways of letting go are drawn.
    """

    parameter_space: ReleaseParameterSpace = field(
        default_factory=ReleaseParameterSpace
    )
    """
    What the ways of letting go are drawn from.
    """

    model_registry: ModelRegistry = field(default_factory=UniformPriorRegistry)
    """
    Answers the statement of the ways of letting go.
    """

    def run(self) -> Iterator[ReleaseTrial]:
        """
        :return: The stored trials, as they are tried.
        """
        episode = self.task.compose()
        self.starting_world.put_into(world=episode.world)
        drawn = self.parameter_space.drawn_parameters(
            number_of_releases=self.number_of_trials,
            model_registry=self.model_registry,
        )
        with episode.executed(physics=True):
            performer = ReleasePerformer(
                episode=episode,
                task=self.task,
                held=episode.graspable(self.task.picked_object).root,
                target=episode.object_body(self.task.receptacle),
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
    Try ways of letting go from a stored starting world and report how many accomplished
    the task.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", required=True, help="the database URI")
    parser.add_argument(
        "--starting-world",
        type=int,
        required=True,
        help="the database id of the world every trial starts from",
    )
    parser.add_argument(
        "--seed", type=int, default=0, help="seeds RoboCasa's choice of the objects"
    )
    parser.add_argument(
        "--draw-seed", type=int, default=0, help="seeds the drawing of the trials"
    )
    parser.add_argument("--attempts", type=int, default=ReleaseTrials.number_of_trials)
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.INFO, force=True)
    # the probabilistic models draw from numpy's global random state
    np.random.seed(arguments.draw_seed)
    trials = ReleaseTrials(
        task=PickPlaceSinkToCounter(
            robot=RobotModel.PANDA_OMRON, seed=arguments.seed, physics=True
        ),
        starting_world=StoredStartingWorld(
            database=GraspDatabase.connect(arguments.database),
            world_id=arguments.starting_world,
        ),
        number_of_trials=arguments.attempts,
    )
    tried = list(trials.run())
    accomplished = sum(trial.result.accomplished for trial in tried)
    print(f"{accomplished} of {len(tried)} ways of letting go accomplished the task")


if __name__ == "__main__":
    main()
