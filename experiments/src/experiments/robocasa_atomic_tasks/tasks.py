"""
RoboCasa's atomic tasks as the PR2 performs them, and the kinds of tasks they share
their solutions by.

Every task is a subclass of :class:`RoboCasaAtomicTask` named after the RoboCasa task it
performs, in the module named after the RoboCasa file that defines that task.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from typing_extensions import Any

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.recording import HeadCameraVideo
from experiments.robocasa_atomic_tasks.robocasa_names import RoboCasaObjectName
from krrood.ormatic.utils import classproperty
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

logger = logging.getLogger(__name__)


# %% one of RoboCasa's tasks, performed by the PR2


@dataclass
class RoboCasaAtomicTask(ABC):
    """
    One of RoboCasa's atomic tasks, performed by a PR2.

    A subclass is named after the RoboCasa task it performs and says, in :meth:`solve`,
    what the PR2 does.
    """

    layout: LayoutType = LayoutType.LAYOUT001
    """
    The kitchen layout.
    """

    style: StyleType = StyleType.STYLE001
    """
    The visual style of the kitchen.
    """

    seed: int | None = 0
    """
    The seed RoboCasa samples the objects and their placements with.
    """

    collision_avoidance: bool = True
    """
    Whether the PR2 avoids colliding with the kitchen and with itself while moving.
    """

    physics: bool = False
    """
    Whether the PR2 acts in a physical simulation of the kitchen in MuJoCo rather than a
    kinematic one.
    """

    video: Path | None = None
    """
    Where a video of the episode seen from the PR2's head camera is written, if
    anywhere.
    """

    task_parameters: dict[str, Any] = field(default_factory=dict)
    """
    Arguments of the RoboCasa task itself, by name.
    """

    episode: RoboCasaEpisode | None = field(default=None, init=False, repr=False)
    """
    The episode composed for the task, once it is.
    """

    @classproperty
    def robocasa_task_name(cls) -> str:
        """
        The name of the RoboCasa task environment performed, the name of the class.
        """
        return cls.__name__

    def compose(self) -> RoboCasaEpisode:
        """
        :return: A new episode of the task.
        """
        self.episode = RoboCasaEpisode.compose(
            self.robocasa_task_name,
            self.layout,
            self.style,
            seed=self.seed,
            task_parameters=self.task_parameters or None,
        )
        return self.episode

    @abstractmethod
    def solve(self, episode: RoboCasaEpisode) -> None:
        """
        Have the PR2 perform the task.

        :param episode: The episode the task is performed in.
        """

    def run(self) -> bool:
        """
        Compose an episode, have the PR2 perform the task and judge it; a video, if
        asked for, shows the episode as far as it got.

        :return: Whether RoboCasa's own check finds the task accomplished.
        """
        episode = self.compose()
        logger.info("%s: %s", self.robocasa_task_name, episode.task.instruction)
        video = None
        if self.video is not None:
            video = HeadCameraVideo(
                world=episode.world,
                robot=episode.robot,
                tool_frame=episode.arm.end_effector.tool_frame,
            )
        try:
            with episode.executed(self.collision_avoidance, self.physics):
                self.solve(episode)
                episode.let_physics_run()
            if video is not None:
                video.stop()
            succeeded = episode.judge.succeeded()
            if not succeeded:
                episode.judge.log_judgement()
            return succeeded
        finally:
            if video is not None:
                video.render(self.video)


# %% kinds of tasks


@dataclass
class JointGoalTask(RoboCasaAtomicTask, ABC):
    """
    A task accomplished by moving joints of the kitchen's fixtures to positions by the
    parts hanging from them, such as opening a door by its handle or turning a knob.
    """

    @abstractmethod
    def joint_goals(self, episode: RoboCasaEpisode) -> list[JointGoal]:
        """
        :param episode: The episode the task is performed in.
        :return: The joints to move, in the order they are moved.
        """

    def solve(self, episode: RoboCasaEpisode) -> None:
        for goal in self.joint_goals(episode):
            episode.operate(goal.joint_name, goal.world_position)


@dataclass
class JointGoal:
    """
    A joint of the kitchen and the position it is to be moved to.
    """

    joint_name: str
    """
    The name RoboCasa gives the joint.
    """

    world_position: float
    """
    The position, as the world states it.
    """


@dataclass
class DoorShareTask(JointGoalTask, ABC):
    """
    A task accomplished by moving the doors, drawers or racks of a fixture to a share of
    their travel; in a physical simulation, what is to be shut is pushed shut, see
    :meth:`RoboCasaEpisode.push_shut`.
    """

    share: float = 1.0
    """
    The share of their travel the joints are moved to: 0 closed, 1 fully open.
    """

    @abstractmethod
    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        """
        :param episode: The episode the task is performed in.
        :return: The names RoboCasa gives the joints to move.
        """

    def target_share(self, episode: RoboCasaEpisode) -> float:
        """
        :param episode: The episode the task is performed in.
        :return: The share of their travel the joints are moved to.
        """
        return self.share

    def joint_goals(self, episode: RoboCasaEpisode) -> list[JointGoal]:
        share = self.target_share(episode)
        return [
            JointGoal(name, episode.joints.world_position_at_share(name, share))
            for name in self.joint_names(episode)
        ]

    def solve(self, episode: RoboCasaEpisode) -> None:
        for goal in self.joint_goals(episode):
            shut = episode.joints.world_position_at_share(goal.joint_name, 0.0)
            if episode.physically_simulated and goal.world_position == shut:
                episode.push_shut(goal.joint_name, goal.world_position)
            else:
                episode.operate(goal.joint_name, goal.world_position)


@dataclass
class PushTask(RoboCasaAtomicTask, ABC):
    """
    A task accomplished by pushing a part of a fixture with the fingertips until the
    joint it hangs from stands at a position, such as a switch or a lever.
    """

    @abstractmethod
    def pushed_joint(self, episode: RoboCasaEpisode) -> JointGoal:
        """
        :param episode: The episode the task is performed in.
        :return: The joint pushed and the position it is pushed to.
        """

    def solve(self, episode: RoboCasaEpisode) -> None:
        goal = self.pushed_joint(episode)
        episode.push(goal.joint_name, goal.world_position)


@dataclass
class PressTask(RoboCasaAtomicTask, ABC):
    """
    A task accomplished by pressing a button RoboCasa senses by contact alone.
    """

    @abstractmethod
    def button_geom(self, episode: RoboCasaEpisode) -> str:
        """
        :param episode: The episode the task is performed in.
        :return: The name RoboCasa gives the geom of the button.
        """

    def solve(self, episode: RoboCasaEpisode) -> None:
        episode.press(self.button_geom(episode))


@dataclass
class PickAndPlaceTask(RoboCasaAtomicTask, ABC):
    """
    A task accomplished by picking one of the task's objects up and putting it down
    somewhere RoboCasa asks for.
    """

    picked_object: RoboCasaObjectName = RoboCasaObjectName.OBJECT
    """
    The object moved.
    """

    retreat_distance: float = 0.1
    """
    How far above its spot the object is carried before it is lowered onto it, in
    metres.
    """

    @abstractmethod
    def spots(self, episode: RoboCasaEpisode) -> list[np.ndarray]:
        """
        :param episode: The episode the task is performed in, with the object in the
            gripper.
        :return: Where the middle of the object's underside may come to lie, best
            first.
        """

    def solve(self, episode: RoboCasaEpisode) -> None:
        graspable = episode.graspable(self.picked_object)
        episode.pick_up(graspable)
        episode.place(
            graspable, self.spots(episode), retreat_distance=self.retreat_distance
        )
