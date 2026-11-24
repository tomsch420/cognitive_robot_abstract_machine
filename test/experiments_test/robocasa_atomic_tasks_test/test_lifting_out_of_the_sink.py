"""
Tests of lifting an object out of the sink: the broccoli of the sink to counter pick and
place, taken from the front by the PandaOmron standing beside the sink, and the
precondition that refuses the action once the broccoli is out.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.exceptions import PreconditionsNotMet
from experiments.robocasa_atomic_tasks.pick_place import PickPlaceSinkToCounter
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink_actions import (
    LiftAnObjectOutOfTheSinkAction,
)
from krrood.entity_query_language.factories import evaluate_condition, variable
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable
from semantic_digital_twin.world_description.world_entity import Body

# %% the broccoli in the sink


@dataclass
class BroccoliInTheSink:
    """
    An episode of the sink to counter pick and place in a physical simulation, with the
    robot standing beside the sink, and the moment it stood there.
    """

    task: PickPlaceSinkToCounter
    """
    The task the episode was composed for.
    """

    episode: RoboCasaEpisode
    """
    The episode.
    """

    stood: MujocoSimulationSnapshot
    """
    The simulation as it was with the robot standing beside the sink.
    """

    grasp: SurfaceGrasp
    """
    A grasp on the broccoli from the front that lifts it from where the robot stands.
    """

    def lift_it_out(self) -> LiftAnObjectOutOfTheSinkAction:
        """
        :return: A new action lifting the broccoli out of the sink by :attr:`grasp`,
            with its other parameters at their defaults.
        """
        return LiftAnObjectOutOfTheSinkAction(
            arm=self.episode.arm,
            basin=self.basin,
            grasp=self.grasp.grasp_candidate(graspable=self.broccoli.grasped_part()),
        )

    @property
    def broccoli(self) -> IsGraspable:
        """
        :return: The broccoli.
        """
        return self.episode.graspable(self.task.picked_object)

    @property
    def basin(self) -> Body:
        """
        :return: The sink's basin.
        """
        return self.episode.world.get_body_by_name(
            self.episode.environment.sink.root_body
        )

    @property
    def broccoli_is_in_the_basin(self) -> bool:
        """
        :return: Whether the broccoli lies inside the basin.
        """
        return evaluate_condition(
            InsideOf(body=variable(Body, [self.broccoli.root]), other=self.basin)
        )


@pytest.fixture(scope="module")
def broccoli_in_the_sink() -> Iterator[BroccoliInTheSink]:
    document = json.loads(
        (Path(__file__).parent / "broccoli_in_the_sink_stood.json").read_text()
    )
    task = PickPlaceSinkToCounter(robot=RobotModel.PANDA_OMRON, physics=True)
    episode = task.compose()
    world = episode.world
    degrees_of_freedom = {str(dof.name): dof for dof in world.degrees_of_freedom}
    for name, position in document["degree_of_freedom_positions"].items():
        world.state[degrees_of_freedom[name].id].position = position
    world.notify_state_change()
    with episode.executed(physics=True):
        yield BroccoliInTheSink(
            task=task,
            episode=episode,
            stood=episode.simulation.take_snapshot(),
            grasp=SurfaceGrasp(**document["grasp"]),
        )


@pytest.fixture
def sink(broccoli_in_the_sink: BroccoliInTheSink) -> BroccoliInTheSink:
    broccoli_in_the_sink.episode.simulation.restore_snapshot(
        snapshot=broccoli_in_the_sink.stood
    )
    return broccoli_in_the_sink


# %% lifting the broccoli out


def test_lifting_the_broccoli_out_of_the_sink_takes_it_out_of_the_basin(
    sink: BroccoliInTheSink,
):
    assert sink.broccoli_is_in_the_basin

    outcome = sink.episode.run([sink.lift_it_out()])

    assert outcome.succeeded
    assert not sink.broccoli_is_in_the_basin


def test_lifting_the_broccoli_out_is_refused_once_it_is_out(sink: BroccoliInTheSink):
    sink.episode.run([sink.lift_it_out()]).raise_if_failed()

    outcome = sink.episode.run([sink.lift_it_out()])

    assert isinstance(outcome.failure, PreconditionsNotMet)
