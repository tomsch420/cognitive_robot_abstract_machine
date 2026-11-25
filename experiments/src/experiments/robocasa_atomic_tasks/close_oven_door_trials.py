"""
Trials of the PandaOmron closing RoboCasa's oven door by its handle with
:class:`~experiments.robocasa_atomic_tasks.skills.CloseByTheHandleAction`, in MuJoCo, to learn
from where and how it succeeds.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum

from experiments.robocasa_atomic_tasks.doors import CloseOven
from experiments.robocasa_atomic_tasks.let_down_doors import (
    HandleApproach,
    StandBeforeTheHandle,
    StandOnTheFloor,
)
from experiments.robocasa_atomic_tasks.oven_door_trials import (
    BaseMotion,
    OvenDoorScene,
    OvenDoorSetup,
)
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.skills import CloseByTheHandleAction
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

logger = logging.getLogger(__name__)

# %% what a trial varies by


@dataclass(kw_only=True)
class CloseOvenDoorParameters(TrialParameters):
    """
    Everything a trial of closing the oven door varies by.
    """

    stand: StandBeforeTheHandle
    """
    Where the base stands while it closes the door.
    """

    approach: HandleApproach
    """
    How the gripper comes at the handle.
    """

    base_motion: BaseMotion
    """
    Whether the base moves while the arm closes the door.
    """


@dataclass(kw_only=True)
class CloseOvenDoorSpace(ParameterSpace[CloseOvenDoorParameters]):
    """
    The trials of closing the oven door from in front of it.
    """

    sideways: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=-0.3, upper=0.3)
    )
    """
    The values of :attr:`StandBeforeTheHandle.sideways`.
    """

    distance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.35, upper=0.75)
    )
    """
    The values of :attr:`StandBeforeTheHandle.distance`.
    """

    turn: Bounds[float] = field(default_factory=lambda: Bounds(lower=-0.4, upper=0.4))
    """
    The values of :attr:`StandBeforeTheHandle.turn`.
    """

    def conditions(
        self, *, parameters: Match[CloseOvenDoorParameters]
    ) -> list[ConditionType]:
        return [
            *self.conditions_within(
                value=parameters.stand.sideways, bounds=self.sideways
            ),
            *self.conditions_within(
                value=parameters.stand.distance, bounds=self.distance
            ),
            *self.conditions_within(value=parameters.stand.turn, bounds=self.turn),
            or_(*[parameters.approach == approach for approach in HandleApproach]),
            or_(*[parameters.base_motion == motion for motion in BaseMotion]),
        ]


# %% what a trial measures


class CloseOvenDoorStage(Enum):
    """
    The stages of a trial of closing the oven door.
    """

    DRIVING_TO_THE_STAND = "driving to the stand"
    """
    The base drives to where it closes the door from.
    """

    CLOSING_THE_DOOR = "closing the door"
    """
    The arm takes hold of the handle, swings the door shut and lets go.
    """


@dataclass(kw_only=True)
class CloseOvenDoorMeasurements(TrialMeasurements):
    """
    What is measured while the oven door is closed.
    """

    stand_position_error: float
    """
    How far the base stopped from the stand it drove to, in meters.
    """

    stand_turn_error: float
    """
    How far the base stopped turned from the stand it drove to, in radians.
    """

    door_angle: float
    """
    Where the door's joint ended up, in radians; ``0`` is shut.
    """

    failed_stage: CloseOvenDoorStage | None = None
    """
    The stage whose plan failed; ``None`` if every plan ended as planned.
    """

    failure: str | None = None
    """
    The name of the failure the plan of :attr:`failed_stage` ended with.
    """


@dataclass(kw_only=True)
class CloseOvenDoorTrial(Trial[CloseOvenDoorParameters, CloseOvenDoorMeasurements]):
    """
    One try of closing the oven door, succeeded when RoboCasa judges it shut.
    """


# %% closing the door


@dataclass(kw_only=True)
class CloseOvenDoorPerformer(
    TrialPerformer[CloseOvenDoorParameters, CloseOvenDoorTrial]
):
    """
    Closes the oven door in a physically simulated episode, each time from the same
    moment of the simulation.
    """

    scene: OvenDoorScene
    """
    The scene every trial starts from.
    """

    def perform(self, *, parameters: CloseOvenDoorParameters) -> CloseOvenDoorTrial:
        episode = self.scene.episode
        driving = self.scene.drive_to(stand=parameters.stand)
        reached = self.scene.where_the_base_stands()
        if driving.outcome.failure is not None:
            return self.ended_trial(
                parameters=parameters,
                stand=driving.stand,
                reached=reached,
                failed_stage=CloseOvenDoorStage.DRIVING_TO_THE_STAND,
                failure=driving.outcome.failure,
            )
        episode.robot.mobile_base.full_body_controlled = (
            parameters.base_motion is BaseMotion.MOVING_WITH_THE_ARM
        )
        closing = episode.run(
            [
                CloseByTheHandleAction(
                    grasp_candidate=parameters.approach.closest_grasp(
                        handle=self.scene.handle
                    ),
                    arm=episode.arm,
                    closed_joint_position=0.0,
                )
            ]
        )
        episode.let_physics_run()
        return self.ended_trial(
            parameters=parameters,
            stand=driving.stand,
            reached=reached,
            failed_stage=(
                None if closing.failure is None else CloseOvenDoorStage.CLOSING_THE_DOOR
            ),
            failure=closing.failure,
        )

    def ended_trial(
        self,
        *,
        parameters: CloseOvenDoorParameters,
        stand: StandOnTheFloor,
        reached: StandOnTheFloor,
        failed_stage: CloseOvenDoorStage | None,
        failure: Exception | None,
    ) -> CloseOvenDoorTrial:
        """
        :param parameters: What the trial varied by.
        :param stand: Where the base drove to.
        :param reached: Where the base stopped driving.
        :param failed_stage: The stage whose plan failed, if any.
        :param failure: What that plan failed with, if anything.
        :return: The trial, its door measured as the world stands now.
        """
        trial = CloseOvenDoorTrial(
            parameters=parameters,
            measurements=CloseOvenDoorMeasurements(
                stand_position_error=stand.position_error(reached=reached),
                stand_turn_error=stand.turn_error(reached=reached),
                door_angle=float(self.scene.door.parent_connection.position),
                failed_stage=failed_stage,
                failure=None if failure is None else type(failure).__name__,
            ),
            succeeded=self.scene.episode.judge.pass_judgement().accomplished,
        )
        logger.info("Trial %s", trial)
        return trial


# %% the campaign's setup


@dataclass(kw_only=True)
class CloseOvenDoorTrialsInADatabase(DatabaseTrialStore[CloseOvenDoorTrial]):
    """
    Keeps trials of closing the oven door in a database of the experiments' ORM
    interface.
    """

    @property
    def data_access_object_class(self) -> type[DataAccessObject]:
        from experiments.orm.ormatic_interface import CloseOvenDoorTrialDAO

        return CloseOvenDoorTrialDAO


@dataclass(kw_only=True)
class CloseOvenDoorSetup(OvenDoorSetup[CloseOvenDoorParameters, CloseOvenDoorTrial]):
    """
    The oven door, swung open, to be closed.
    """

    def create_task(self) -> CloseOven:
        return CloseOven(robot=RobotModel.PANDA_OMRON, physics=True, seed=self.seed)

    def create_performer(self, *, scene: OvenDoorScene) -> CloseOvenDoorPerformer:
        return CloseOvenDoorPerformer(scene=scene)

    def create_store(self) -> CloseOvenDoorTrialsInADatabase:
        return CloseOvenDoorTrialsInADatabase(database_url=self.database_url)
