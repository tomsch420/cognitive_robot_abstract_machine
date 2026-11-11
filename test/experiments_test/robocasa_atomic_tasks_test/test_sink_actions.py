"""
Tests of the actions at the sink: lifting the hand out of the basin and turning the
faucet on with one finger, and the preconditions that refuse them.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.exceptions import PreconditionsNotMet
from experiments.robocasa_atomic_tasks.robocasa_names import SinkJoint
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink import TurnOnSinkFaucet
from experiments.robocasa_atomic_tasks.sink_actions import (
    LiftTheHandOutOfTheSinkAction,
    TurnOffTheFaucetWithAFingerAction,
    TurnOnTheFaucetWithAFingerAction,
    TurnTheWaterColdWithAFingerAction,
    TurnTheWaterHotWithAFingerAction,
)
from krrood.entity_query_language.factories import evaluate_condition, variable
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.world_entity import Body

# %% the sink of a composed episode


@dataclass
class SinkOfAnEpisode:
    """
    An episode of turning the faucet on, in a physical simulation, and the moment it was
    composed in.
    """

    episode: RoboCasaEpisode
    """
    The episode.
    """

    composed: MujocoSimulationSnapshot
    """
    The simulation as it was right after the episode was composed.
    """

    def lift_the_hand_out(self) -> LiftTheHandOutOfTheSinkAction:
        """
        :return: A new action lifting the hand out of the sink, with its parameters at
            their defaults.
        """
        return LiftTheHandOutOfTheSinkAction(arm=self.episode.arm, basin=self.basin)

    def turn_on_the_faucet(self) -> TurnOnTheFaucetWithAFingerAction:
        """
        :return: A new action turning the faucet on, with its parameters at their
            defaults.
        """
        joints = self.episode.joints
        return TurnOnTheFaucetWithAFingerAction(
            arm=self.episode.arm,
            handle=joints.part_moved_by(self.joint_name(SinkJoint.HANDLE)).root,
            lift_joint=joints.connection(self.joint_name(SinkJoint.HANDLE)),
            temperature_joint=self.temperature_joint,
            basin=self.basin,
        )

    def turn_off_the_faucet(self) -> TurnOffTheFaucetWithAFingerAction:
        """
        :return: A new action turning the faucet off, with its parameters at their
            defaults.
        """
        joints = self.episode.joints
        return TurnOffTheFaucetWithAFingerAction(
            arm=self.episode.arm,
            handle=joints.part_moved_by(self.joint_name(SinkJoint.HANDLE)).root,
            lift_joint=joints.connection(self.joint_name(SinkJoint.HANDLE)),
            basin=self.basin,
        )

    def turn_the_water_hot(self) -> TurnTheWaterHotWithAFingerAction:
        """
        :return: A new action turning the water hot, with its parameters at their
            defaults.
        """
        joints = self.episode.joints
        return TurnTheWaterHotWithAFingerAction(
            arm=self.episode.arm,
            handle=joints.part_moved_by(self.joint_name(SinkJoint.HANDLE)).root,
            lift_joint=joints.connection(self.joint_name(SinkJoint.HANDLE)),
            temperature_joint=self.temperature_joint,
            basin=self.basin,
        )

    def turn_the_water_cold(self) -> TurnTheWaterColdWithAFingerAction:
        """
        :return: A new action turning the water cold, with its parameters at their
            defaults.
        """
        joints = self.episode.joints
        return TurnTheWaterColdWithAFingerAction(
            arm=self.episode.arm,
            handle=joints.part_moved_by(self.joint_name(SinkJoint.HANDLE)).root,
            lift_joint=joints.connection(self.joint_name(SinkJoint.HANDLE)),
            temperature_joint=self.temperature_joint,
            basin=self.basin,
        )

    @property
    def temperature_joint(self) -> ActiveConnection1DOF:
        """
        :return: The joint the faucet's lever is swung about to choose the water's
            temperature.
        """
        return self.episode.joints.connection(
            self.joint_name(SinkJoint.TEMPERATURE_HANDLE)
        )

    def joint_name(self, joint: SinkJoint) -> str:
        """
        :param joint: One of the sink's joints.
        :return: The name RoboCasa gives it.
        """
        return f"{self.episode.environment.sink.naming_prefix}{joint}"

    @property
    def basin(self) -> Body:
        """
        :return: The sink's basin.
        """
        return self.episode.world.get_body_by_name(
            self.episode.environment.sink.root_body
        )

    @property
    def hand_is_in_the_basin(self) -> bool:
        """
        :return: Whether a body of the gripper is inside the basin.
        """
        hand = variable(Body, self.episode.arm.end_effector.bodies_with_collision)
        return evaluate_condition(InsideOf(body=hand, other=self.basin))


@pytest.fixture(scope="module")
def composed_sink() -> Iterator[SinkOfAnEpisode]:
    episode = TurnOnSinkFaucet(robot=RobotModel("panda_omron")).compose()
    with episode.executed(physics=True):
        yield SinkOfAnEpisode(
            episode=episode, composed=episode.simulation.take_snapshot()
        )


@pytest.fixture
def sink(composed_sink: SinkOfAnEpisode) -> SinkOfAnEpisode:
    composed_sink.episode.simulation.restore_snapshot(composed_sink.composed)
    return composed_sink


# %% lifting the hand out of the sink


def test_lifting_the_hand_out_of_the_sink_leaves_it_outside_the_basin(
    sink: SinkOfAnEpisode,
):
    assert sink.hand_is_in_the_basin

    outcome = sink.episode.run([sink.lift_the_hand_out()])

    assert outcome.succeeded
    assert not sink.hand_is_in_the_basin


def test_lifting_the_hand_out_is_refused_once_it_is_out(sink: SinkOfAnEpisode):
    sink.episode.run([sink.lift_the_hand_out()]).raise_if_failed()

    outcome = sink.episode.run([sink.lift_the_hand_out()])

    assert isinstance(outcome.failure, PreconditionsNotMet)


# %% turning the faucet on


def test_turning_the_faucet_on_once_the_hand_is_out_makes_the_water_run(
    sink: SinkOfAnEpisode,
):
    sink.episode.run([sink.lift_the_hand_out()]).raise_if_failed()

    outcome = sink.episode.run([sink.turn_on_the_faucet()])

    assert outcome.succeeded
    sink.episode.judge.mirror.synchronize()
    assert sink.episode.judge.pass_judgement().accomplished


def test_turning_the_faucet_on_with_the_lever_swung_to_hot_makes_the_water_run(
    sink: SinkOfAnEpisode,
):
    sink.temperature_joint.position = sink.episode.joints.world_position_along_range(
        sink.joint_name(SinkJoint.TEMPERATURE_HANDLE), 1.0
    )
    sink.episode.run([sink.lift_the_hand_out()]).raise_if_failed()

    outcome = sink.episode.run([sink.turn_on_the_faucet()])

    assert outcome.succeeded
    sink.episode.judge.mirror.synchronize()
    assert sink.episode.judge.pass_judgement().accomplished


def test_turning_the_faucet_on_is_refused_while_the_hand_is_in_the_sink(
    sink: SinkOfAnEpisode,
):
    outcome = sink.episode.run([sink.turn_on_the_faucet()])

    assert isinstance(outcome.failure, PreconditionsNotMet)


def test_turning_the_faucet_on_is_refused_while_the_water_runs(
    sink: SinkOfAnEpisode,
):
    sink.episode.run([sink.lift_the_hand_out()]).raise_if_failed()
    sink.episode.run([sink.turn_on_the_faucet()]).raise_if_failed()

    outcome = sink.episode.run([sink.turn_on_the_faucet()])

    assert isinstance(outcome.failure, PreconditionsNotMet)
