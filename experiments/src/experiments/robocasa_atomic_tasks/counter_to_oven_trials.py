"""
Trials of the PandaOmron moving the object of RoboCasa's PickPlaceCounterToOven from
its plate onto the oven tray in MuJoCo, to learn how close the robot may come to the
kitchen and where it carries the object for the move to succeed.
"""

from __future__ import annotations

import logging
from contextlib import contextmanager
from dataclasses import dataclass, field

import numpy as np
from typing_extensions import Iterator

from experiments.robocasa_atomic_tasks.counter_to_oven import (
    MoveFromTheCounterOntoTheOvenTray,
)
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.pick_place import PickPlaceCounterToOven
from experiments.robocasa_atomic_tasks.robocasa_names import RoboCasaObjectName
from experiments.robocasa_atomic_tasks.robots import RobotModel
from krrood.bounds import Bounds
from krrood.entity_query_language.factories import ConditionType
from krrood.entity_query_language.query.match import Match
from krrood.ormatic.data_access_objects.dao import DataAccessObject
from krrood.parametrization.trials.campaigns import TrialPerformer, TrialSetup
from krrood.parametrization.trials.database import DatabaseTrialStore
from krrood.parametrization.trials.parameter_spaces import ParameterSpace
from krrood.parametrization.trials.records import (
    Trial,
    TrialMeasurements,
    TrialParameters,
)
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot

logger = logging.getLogger(__name__)

# %% what a trial varies by


@dataclass(kw_only=True)
class CounterToOvenParameters(TrialParameters):
    """
    Everything a trial of moving the object onto the oven tray varies by, each the
    field of :class:`MoveFromTheCounterOntoTheOvenTray` of the same name.
    """

    surroundings_buffer_zone_distance: float
    """
    How close to everything else the robot may come while it reaches above the object,
    carries it and puts it down, in metres.
    """

    counter_buffer_zone_distance: float
    """
    How close to the fixtures around the object the robot may come while it takes hold
    of it, in metres.
    """

    hand_clearance_from_the_oven: float
    """
    How far in front of the oven the object is carried to at least, in metres.
    """

    carry_height: float
    """
    How far above the tray's top the underside of the object is carried to, in metres.
    """

    carry_clearance: float
    """
    How far beyond half the height of the object the path it is carried along keeps its
    middle from everything else, in metres.
    """

    margin_inside_the_tray: float
    """
    How far inside the tray's front edge the box of the object is carried to at least,
    in metres.
    """


@dataclass(kw_only=True)
class CounterToOvenSpace(ParameterSpace[CounterToOvenParameters]):
    """
    The trials of moving the object onto the oven tray.
    """

    surroundings_buffer_zone_distance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.008, upper=0.022)
    )
    """
    The values of :attr:`CounterToOvenParameters.surroundings_buffer_zone_distance`.
    """

    counter_buffer_zone_distance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.001, upper=0.004)
    )
    """
    The values of :attr:`CounterToOvenParameters.counter_buffer_zone_distance`.
    """

    hand_clearance_from_the_oven: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.1, upper=0.18)
    )
    """
    The values of :attr:`CounterToOvenParameters.hand_clearance_from_the_oven`.
    """

    carry_height: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.03, upper=0.09)
    )
    """
    The values of :attr:`CounterToOvenParameters.carry_height`.
    """

    carry_clearance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.03, upper=0.1)
    )
    """
    The values of :attr:`CounterToOvenParameters.carry_clearance`.
    """

    margin_inside_the_tray: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.0, upper=0.05)
    )
    """
    The values of :attr:`CounterToOvenParameters.margin_inside_the_tray`.
    """

    def conditions(
        self, *, parameters: Match[CounterToOvenParameters]
    ) -> list[ConditionType]:
        return [
            *self.conditions_within(
                value=parameters.surroundings_buffer_zone_distance,
                bounds=self.surroundings_buffer_zone_distance,
            ),
            *self.conditions_within(
                value=parameters.counter_buffer_zone_distance,
                bounds=self.counter_buffer_zone_distance,
            ),
            *self.conditions_within(
                value=parameters.hand_clearance_from_the_oven,
                bounds=self.hand_clearance_from_the_oven,
            ),
            *self.conditions_within(
                value=parameters.carry_height, bounds=self.carry_height
            ),
            *self.conditions_within(
                value=parameters.carry_clearance, bounds=self.carry_clearance
            ),
            *self.conditions_within(
                value=parameters.margin_inside_the_tray,
                bounds=self.margin_inside_the_tray,
            ),
        ]


# %% what a trial measures


@dataclass(kw_only=True)
class CounterToOvenMeasurements(TrialMeasurements):
    """
    What is measured once the object was moved, or a step of the move failed.
    """

    seed: int
    """
    The seed RoboCasa sampled the episode with.
    """

    object_distance_from_the_tray_middle: float
    """
    How far the object's origin ended up from the middle of the tray, horizontally, in
    metres.
    """

    object_height_above_the_tray: float
    """
    How far the object's origin ended up above the top of the tray, in metres.
    """

    failure: str | None = None
    """
    The name of the failure the move ended with, if any.
    """

    failure_message: str | None = None
    """
    The first line of the failure's message, naming the motion that failed, if any.
    """


@dataclass(kw_only=True)
class CounterToOvenTrial(Trial[CounterToOvenParameters, CounterToOvenMeasurements]):
    """
    One try of moving the object onto the oven tray, succeeded when RoboCasa judges the
    task accomplished.
    """


# %% moving the object


@dataclass(kw_only=True)
class CounterToOvenPerformer(
    TrialPerformer[CounterToOvenParameters, CounterToOvenTrial]
):
    """
    Moves the object onto the oven tray in a physically simulated episode, each time
    from the same moment of the simulation.
    """

    episode: RoboCasaEpisode
    """
    The episode, physically simulated.
    """

    seed: int
    """
    The seed RoboCasa sampled the episode with.
    """

    starting_moment: MujocoSimulationSnapshot
    """
    The moment of the simulation every trial starts from.
    """

    def perform(self, *, parameters: CounterToOvenParameters) -> CounterToOvenTrial:
        episode = self.episode
        episode.simulation.restore_snapshot(snapshot=self.starting_moment)
        episode.robot.mobile_base.full_body_controlled = True
        outcome = MoveFromTheCounterOntoTheOvenTray(
            episode=episode,
            graspable=episode.graspable(RoboCasaObjectName.OBJECT),
            plate=episode.object_body(RoboCasaObjectName.OBJECT_CONTAINER),
            tray=episode.object_body(RoboCasaObjectName.OVEN_TRAY),
            oven=episode.fixture_body(episode.environment.oven),
            surroundings_buffer_zone_distance=parameters.surroundings_buffer_zone_distance,
            counter_buffer_zone_distance=parameters.counter_buffer_zone_distance,
            hand_clearance_from_the_oven=parameters.hand_clearance_from_the_oven,
            carry_height=parameters.carry_height,
            carry_clearance=parameters.carry_clearance,
            margin_inside_the_tray=parameters.margin_inside_the_tray,
        ).attempt()
        episode.let_physics_run()
        episode.judge.mirror.synchronize()
        tray = episode.bounds.of_body(episode.object_body(RoboCasaObjectName.OVEN_TRAY))
        tray_middle = (tray.lower + tray.upper) / 2
        object_position = episode.object_body(
            RoboCasaObjectName.OBJECT
        ).global_pose.to_np()[:3, 3]
        measurements = CounterToOvenMeasurements(
            seed=self.seed,
            object_distance_from_the_tray_middle=float(
                np.linalg.norm(object_position[:2] - tray_middle[:2])
            ),
            object_height_above_the_tray=float(object_position[2] - tray.upper[2]),
        )
        if outcome.failure is not None:
            measurements.failure = type(outcome.failure).__name__
            measurements.failure_message = str(outcome.failure).splitlines()[0]
        trial = CounterToOvenTrial(
            parameters=parameters,
            measurements=measurements,
            succeeded=episode.judge.pass_judgement().accomplished,
        )
        logger.info("Trial %s", trial)
        return trial


# %% the campaign's setup


@dataclass(kw_only=True)
class CounterToOvenTrialsInADatabase(DatabaseTrialStore[CounterToOvenTrial]):
    """
    Keeps trials of moving the object onto the oven tray in a database of the
    experiments' ORM interface.
    """

    @property
    def data_access_object_class(self) -> type[DataAccessObject]:
        from experiments.orm.ormatic_interface import CounterToOvenTrialDAO

        return CounterToOvenTrialDAO


@dataclass(kw_only=True)
class CounterToOvenSetup(TrialSetup[CounterToOvenParameters, CounterToOvenTrial]):
    """
    The PandaOmron in a RoboCasa PickPlaceCounterToOven episode, physically simulated in
    MuJoCo, the object on its plate on the counter.
    """

    database_url: str
    """
    Where the trials are kept.
    """

    seed: int = 0
    """
    The seed RoboCasa samples the episode with.
    """

    @contextmanager
    def performing(self) -> Iterator[CounterToOvenPerformer]:
        task = PickPlaceCounterToOven(
            robot=RobotModel.PANDA_OMRON, physics=True, seed=self.seed
        )
        episode = task.compose()
        with episode.executed(collision_avoidance=True, physics=True):
            episode.let_physics_run()
            yield CounterToOvenPerformer(
                episode=episode,
                seed=self.seed,
                starting_moment=episode.simulation.take_snapshot(),
            )

    def create_store(self) -> CounterToOvenTrialsInADatabase:
        return CounterToOvenTrialsInADatabase(database_url=self.database_url)
