"""
Trials of the PandaOmron opening RoboCasa's oven door in MuJoCo, either pulling it all
the way down by its handle or pulling it partway, letting go and pushing it down from
its inner face, to learn from where and how it succeeds.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from numpy.typing import NDArray

from experiments.robocasa_atomic_tasks.doors import OpenOven
from experiments.robocasa_atomic_tasks.oven_door_trials import (
    BaseMotion,
    HandleApproach,
    OvenDoorScene,
    OvenDoorSetup,
    StandBeforeTheHandle,
)
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.skills import (
    OpenByTheHandleAction,
    PressAction,
    PushAction,
)
from krrood.bounds import Bounds
from krrood.entity_query_language.factories import ConditionType, or_
from krrood.entity_query_language.query.match import Match
from krrood.ormatic.data_access_objects.dao import DataAccessObject
from krrood.parametrization.trials.campaigns import TrialPerformer
from krrood.parametrization.trials.database import DatabaseTrialStore
from krrood.parametrization.trials.parameter_spaces import ParameterSpace
from krrood.parametrization.trials.records import (
    Trial,
    TrialMeasurements,
    TrialParameters,
)
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

# %% what a trial varies by


class OpeningStrategy(Enum):
    """
    How the door is brought down.
    """

    PULLING_ALL_THE_WAY = "pulling all the way"
    """
    The gripper pulls the door by its handle down to its lower stop and lets go.
    """

    PULLING_PARTWAY_THEN_PUSHING = "pulling partway, then pushing"
    """
    The gripper pulls the door by its handle partway down, lets go and pushes the rest
    of the way down on the door's inner face, unless the door already fell open.
    """


@dataclass(kw_only=True)
class PressOnTheInnerFace:
    """
    Where the closed fingertips press on the door's inner face, relative to the middle
    of its top edge.
    """

    depth_below_the_top: float
    """
    How far below the door's top edge the fingertips press, in meters.
    """

    sideways: float
    """
    How far to the right of the door's middle the fingertips press, as seen by a robot
    facing the oven, in meters.
    """

    def pressed_point(self, *, door: Body) -> NDArray[np.float64]:
        """
        :param door: The door, whose frame's y axis points into the closed oven and whose
            z axis points up.
        :return: The point the fingertips press on, in the door's frame; they come in
            along its negative y axis.
        """
        box = door.collision.as_bounding_box_collection_in_frame(door).bounding_box()
        return np.array(
            [
                (box.min_x + box.max_x) / 2 + self.sideways,
                box.max_y,
                box.max_z - self.depth_below_the_top,
            ]
        )


@dataclass(kw_only=True)
class OpenOvenDoorParameters(TrialParameters):
    """
    Everything a trial of opening the oven door varies by.
    """

    stand: StandBeforeTheHandle
    """
    Where the base stands while it opens the door.
    """

    strategy: OpeningStrategy
    """
    How the door is brought down.
    """

    base_motion: BaseMotion
    """
    Whether the base moves while the arm moves the door.
    """

    release_angle: float
    """
    How far, in radians, the door is pulled open before the gripper lets go of the
    handle when :attr:`strategy` pushes it the rest of the way.
    """

    release_clearance: float
    """
    The gap in meters between the handle and the gripper once it has let go.
    """

    press: PressOnTheInnerFace
    """
    Where the fingertips press on the door when :attr:`strategy` pushes it.
    """


@dataclass(kw_only=True)
class OpenOvenDoorSpace(ParameterSpace[OpenOvenDoorParameters]):
    """
    The trials of opening the oven door, standing in front of it or beside it.
    """

    sideways: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=-0.9, upper=0.3)
    )
    """
    The values of :attr:`StandBeforeTheHandle.sideways`.
    """

    distance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.4, upper=1.0)
    )
    """
    The values of :attr:`StandBeforeTheHandle.distance`.
    """

    turn: Bounds[float] = field(default_factory=lambda: Bounds(lower=-0.6, upper=0.6))
    """
    The values of :attr:`StandBeforeTheHandle.turn`.
    """

    release_angle: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.2, upper=0.9)
    )
    """
    The values of :attr:`OpenOvenDoorParameters.release_angle`.
    """

    release_clearance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.05, upper=0.3)
    )
    """
    The values of :attr:`OpenOvenDoorParameters.release_clearance`.
    """

    press_depth: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.02, upper=0.2)
    )
    """
    The values of :attr:`PressOnTheInnerFace.depth_below_the_top`.
    """

    press_sideways: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=-0.3, upper=0.3)
    )
    """
    The values of :attr:`PressOnTheInnerFace.sideways`.
    """

    def conditions(
        self, *, parameters: Match[OpenOvenDoorParameters]
    ) -> list[ConditionType]:
        return [
            *self.conditions_within(
                value=parameters.stand.sideways, bounds=self.sideways
            ),
            *self.conditions_within(
                value=parameters.stand.distance, bounds=self.distance
            ),
            *self.conditions_within(value=parameters.stand.turn, bounds=self.turn),
            or_(*[parameters.strategy == strategy for strategy in OpeningStrategy]),
            or_(*[parameters.base_motion == motion for motion in BaseMotion]),
            *self.conditions_within(
                value=parameters.release_angle, bounds=self.release_angle
            ),
            *self.conditions_within(
                value=parameters.release_clearance, bounds=self.release_clearance
            ),
            *self.conditions_within(
                value=parameters.press.depth_below_the_top, bounds=self.press_depth
            ),
            *self.conditions_within(
                value=parameters.press.sideways, bounds=self.press_sideways
            ),
        ]


# %% what a trial measures


@dataclass(kw_only=True)
class OpenOvenDoorMeasurements(TrialMeasurements):
    """
    What is measured while the oven door is opened.
    """

    stand_position_error: float
    """
    How far the base stopped from the stand it drove to, in meters.
    """

    stand_turn_error: float
    """
    How far the base stopped turned from the stand it drove to, in radians.
    """

    door_angle_after_pulling: float | None = None
    """
    Where the door's joint came to rest once the gripper let go of the handle, in
    radians; ``None`` if the base never reached its stand.
    """

    door_angle: float
    """
    Where the door's joint ended up, in radians; ``0`` is shut.
    """

    base_drift: float | None = None
    """
    How far the base moved, in meters, while the arm moved the door; ``None`` if the
    base never reached its stand.
    """

    pushed: bool = False
    """
    Whether the fingertips pushed on the door's inner face.
    """

    driving_failure: str | None = None
    """
    The name of the failure the drive to the stand ended with, if any.
    """

    pulling_failure: str | None = None
    """
    The name of the failure pulling the door by its handle ended with, if any.
    """

    pushing_failure: str | None = None
    """
    The name of the failure pushing the door down ended with, if any.
    """


@dataclass(kw_only=True)
class OpenOvenDoorTrial(Trial[OpenOvenDoorParameters, OpenOvenDoorMeasurements]):
    """
    One try of opening the oven door, succeeded when RoboCasa judges it open.
    """


# %% opening the door


@dataclass(kw_only=True)
class OpenOvenDoorPerformer(TrialPerformer[OpenOvenDoorParameters, OpenOvenDoorTrial]):
    """
    Opens the oven door in a physically simulated episode, each time from the same
    moment of the simulation.
    """

    scene: OvenDoorScene
    """
    The scene every trial starts from.
    """

    press_approach_distance: float = 0.1
    """
    How far before the door's inner face, in meters, the straight push onto it starts.
    """

    push_goal_short_of_the_stop: float = 0.03
    """
    How far short of the door's lower stop, in radians, the push aims.
    """

    def perform(self, *, parameters: OpenOvenDoorParameters) -> OpenOvenDoorTrial:
        episode = self.scene.episode
        stand = parameters.stand.stand_on_the_floor(
            handle_middle=self.scene.handle_middle,
            heading_facing_the_oven=self.scene.start.heading,
        )
        driving = self.scene.drive_to(stand=stand)
        reached = self.scene.where_the_base_stands()
        measurements = OpenOvenDoorMeasurements(
            stand_position_error=stand.position_error(reached=reached),
            stand_turn_error=stand.turn_error(reached=reached),
            door_angle=self.door_angle,
        )
        if driving.failure is not None:
            measurements.driving_failure = type(driving.failure).__name__
            return self.ended_trial(parameters=parameters, measurements=measurements)
        episode.robot.mobile_base.full_body_controlled = (
            parameters.base_motion is BaseMotion.MOVING_WITH_THE_ARM
        )
        with episode.allowances.bodies_may_touch(
            self.wrist, [self.scene.door, self.scene.handle.root]
        ):
            self.pull(parameters=parameters, measurements=measurements)
            if (
                parameters.strategy is OpeningStrategy.PULLING_PARTWAY_THEN_PUSHING
                and not episode.judge.pass_judgement().accomplished
            ):
                self.push(parameters=parameters, measurements=measurements)
        episode.let_physics_run()
        measurements.door_angle = self.door_angle
        measurements.base_drift = reached.position_error(
            reached=self.scene.where_the_base_stands()
        )
        return self.ended_trial(parameters=parameters, measurements=measurements)

    @property
    def door_angle(self) -> float:
        """
        Where the door's joint is now, in radians.
        """
        return float(self.scene.door.parent_connection.position)

    @property
    def wrist(self) -> list[Body]:
        """
        The two links of the arm the hand hangs from, which come close to the handle as
        the door swings down.
        """
        hand = self.scene.episode.arm.end_effector.root
        forearm_end = hand.parent_connection.parent
        return [forearm_end, forearm_end.parent_connection.parent]

    def pull(
        self,
        *,
        parameters: OpenOvenDoorParameters,
        measurements: OpenOvenDoorMeasurements,
    ) -> None:
        """
        Pull the door by its handle as far as the strategy asks and let go of it.

        :param parameters: What the trial varies by.
        :param measurements: Where the outcome is noted.
        """
        episode = self.scene.episode
        pulling = episode.run(
            [
                OpenByTheHandleAction(
                    grasp_candidate=self.scene.grasp(
                        approach=HandleApproach.FROM_THE_FRONT
                    ),
                    arm=episode.arm,
                    opened_joint_position=(
                        parameters.release_angle
                        if parameters.strategy
                        is OpeningStrategy.PULLING_PARTWAY_THEN_PUSHING
                        else None
                    ),
                    release_clearance=parameters.release_clearance,
                )
            ]
        )
        episode.let_physics_run()
        measurements.door_angle_after_pulling = self.door_angle
        if pulling.failure is not None:
            measurements.pulling_failure = type(pulling.failure).__name__

    def push(
        self,
        *,
        parameters: OpenOvenDoorParameters,
        measurements: OpenOvenDoorMeasurements,
    ) -> None:
        """
        Press the closed fingertips on the door's inner face and push the door down.

        :param parameters: What the trial varies by.
        :param measurements: Where the outcome is noted.
        """
        episode = self.scene.episode
        door = self.scene.door
        point_in_door = parameters.press.pressed_point(door=door)
        world_T_door = door.global_pose.to_np()
        joint = door.parent_connection
        measurements.pushed = True
        pushing = episode.run(
            [
                PushAction(
                    press=PressAction(
                        point=(world_T_door @ np.append(point_in_door, 1.0))[:3],
                        approach=world_T_door[:3, :3] @ np.array([0.0, -1.0, 0.0]),
                        arm=episode.arm,
                        approach_distance=self.press_approach_distance,
                    ),
                    part=door,
                    joint=joint,
                    goal_position=joint.dof.limits.upper.position
                    - self.push_goal_short_of_the_stop,
                    threshold=self.push_goal_short_of_the_stop,
                )
            ]
        )
        if pushing.failure is not None:
            measurements.pushing_failure = type(pushing.failure).__name__

    def ended_trial(
        self,
        *,
        parameters: OpenOvenDoorParameters,
        measurements: OpenOvenDoorMeasurements,
    ) -> OpenOvenDoorTrial:
        """
        :param parameters: What the trial varied by.
        :param measurements: What was measured.
        :return: The trial, judged as the world stands now.
        """
        trial = OpenOvenDoorTrial(
            parameters=parameters,
            measurements=measurements,
            succeeded=self.scene.episode.judge.pass_judgement().accomplished,
        )
        logger.info("Trial %s", trial)
        return trial


# %% the campaign's setup


@dataclass(kw_only=True)
class OpenOvenDoorTrialsInADatabase(DatabaseTrialStore[OpenOvenDoorTrial]):
    """
    Keeps trials of opening the oven door in a database of the experiments' ORM
    interface.
    """

    @property
    def data_access_object_class(self) -> type[DataAccessObject]:
        from experiments.orm.ormatic_interface import OpenOvenDoorTrialDAO

        return OpenOvenDoorTrialDAO


@dataclass(kw_only=True)
class OpenOvenDoorSetup(OvenDoorSetup[OpenOvenDoorParameters, OpenOvenDoorTrial]):
    """
    The oven door, shut, to be opened.
    """

    def create_task(self) -> OpenOven:
        return OpenOven(robot=RobotModel.PANDA_OMRON, physics=True, seed=self.seed)

    def create_performer(self, *, scene: OvenDoorScene) -> OpenOvenDoorPerformer:
        return OpenOvenDoorPerformer(scene=scene)

    def create_store(self) -> OpenOvenDoorTrialsInADatabase:
        return OpenOvenDoorTrialsInADatabase(database_url=self.database_url)
