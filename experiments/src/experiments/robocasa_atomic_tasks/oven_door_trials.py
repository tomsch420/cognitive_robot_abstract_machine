"""
What trials of the PandaOmron moving RoboCasa's oven door by its handle, in MuJoCo, have
in common: whether the base moves with the arm and the scene every trial starts from.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum

from typing_extensions import TYPE_CHECKING, Iterator

from coraplex.plans.executors import PlanOutcome
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from experiments.robocasa_atomic_tasks.door_handles import RoboCasaDoorHandleSplitter
from experiments.robocasa_atomic_tasks.doors import ManipulateFixtureDoors
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.let_down_doors import (
    StandBeforeTheHandle,
    StandOnTheFloor,
)
from krrood.parametrization.trials.campaigns import TrialPerformer, TrialSetup
from krrood.parametrization.trials.records import ParametersTypeVar, TrialTypeVar
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle

if TYPE_CHECKING:
    from semantic_digital_twin.world_description.world_entity import Body

# %% what a trial varies by


class BaseMotion(Enum):
    """
    Whether the base moves while the arm moves the door.
    """

    STANDING_STILL = "standing still"
    """
    Only the arm moves.
    """

    MOVING_WITH_THE_ARM = "moving with the arm"
    """
    The base moves together with the arm, as one body.
    """


# %% the scene a trial starts from


@dataclass(kw_only=True)
class OvenDoorScene:
    """
    The physically simulated episode a trial with the oven door starts from.
    """

    episode: RoboCasaEpisode
    """
    The episode, physically simulated.
    """

    handle: Handle
    """
    The handle of the oven door, split off into a body of its own.
    """

    starting_moment: MujocoSimulationSnapshot
    """
    The moment of the simulation every trial starts from.
    """

    @property
    def door(self) -> Body:
        """
        The door the handle is mounted on.
        """
        return self.handle.root.parent_connection.parent

    def drive_to(self, *, stand: StandBeforeTheHandle) -> DriveToTheStand:
        """
        Restore :attr:`starting_moment` and drive the base, alone, to a stand before the
        handle as it is at that moment.

        :param stand: Where the base drives to.
        :return: The drive.
        """
        self.episode.simulation.restore_snapshot(snapshot=self.starting_moment)
        self.episode.robot.mobile_base.full_body_controlled = False
        on_the_floor = stand.stand_before(handle=self.handle)
        return DriveToTheStand(
            stand=on_the_floor,
            outcome=self.episode.run(
                [
                    NavigateAction(
                        planar_goal_pose=on_the_floor.to_pose(world=self.episode.world)
                    )
                ]
            ),
        )

    def where_the_base_stands(self) -> StandOnTheFloor:
        """
        :return: Where the base stands now.
        """
        return StandOnTheFloor.from_base_pose(
            base_pose=self.episode.robot.root.global_pose.to_np()
        )


@dataclass(kw_only=True)
class DriveToTheStand:
    """
    The base driving to a stand before the handle.
    """

    stand: StandOnTheFloor
    """
    Where the base was to stand.
    """

    outcome: PlanOutcome
    """
    How the drive ended.
    """


@dataclass(kw_only=True)
class OvenDoorSetup(TrialSetup[ParametersTypeVar, TrialTypeVar], ABC):
    """
    The PandaOmron in a RoboCasa episode with the oven, the oven door's handle split off
    into a body of its own, physically simulated in MuJoCo.
    """

    database_url: str
    """
    Where the trials are kept.
    """

    seed: int = 0
    """
    The seed RoboCasa samples the episode with.
    """

    @abstractmethod
    def create_task(self) -> ManipulateFixtureDoors:
        """
        :return: A new task of the oven door, with the PandaOmron in physics, sampled with
            :attr:`seed`.
        """

    @abstractmethod
    def create_performer(
        self, *, scene: OvenDoorScene
    ) -> TrialPerformer[ParametersTypeVar, TrialTypeVar]:
        """
        :param scene: The scene every trial starts from.
        :return: A new performer of the trials in that scene.
        """

    @contextmanager
    def performing(self) -> Iterator[TrialPerformer[ParametersTypeVar, TrialTypeVar]]:
        task = self.create_task()
        episode = task.compose()
        (door,) = task.doors(episode)
        RoboCasaDoorHandleSplitter().split_handle_off(door=door)
        with episode.executed(collision_avoidance=True, physics=True):
            episode.let_physics_run()
            yield self.create_performer(
                scene=OvenDoorScene(
                    episode=episode,
                    handle=door.handle,
                    starting_moment=episode.simulation.take_snapshot(),
                )
            )
