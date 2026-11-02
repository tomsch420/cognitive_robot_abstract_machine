"""
A PR2 performs RoboCasa's stove tasks (``kitchen_stove.py``): it takes hold of the knob
of a burner and turns it.

The tasks run in layout 2, whose cooktop has its knobs along its front edge; layout 1's
range has them on a panel at the back, beyond the reach of a PR2 standing clear of it.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.tasks import JointGoal, JointGoalTask
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

# %% the burners' knobs


@dataclass
class StoveKnobTask(JointGoalTask):
    """
    A task about the knob of the burner RoboCasa names.
    """

    layout: LayoutType = LayoutType.LAYOUT002

    @staticmethod
    def knob_joint_name(episode: RoboCasaEpisode) -> str:
        """
        :param episode: An episode of a stove task.
        :return: The name RoboCasa gives the joint of the knob the task is about.
        """
        stove = episode.environment.stove
        return f"{stove.naming_prefix}knob_{episode.environment.knob}_joint"


@dataclass
class TurnOnStove(StoveKnobTask):
    """
    Turn the knob of the burner RoboCasa names from off to on.
    """

    knob_position: float = 0.8
    """
    How far the knob is turned, in radians: RoboCasa counts a burner on from 0.35 rad.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> list[JointGoal]:
        name = self.knob_joint_name(episode)
        return [
            JointGoal(name, episode.joints.world_position_of(name, self.knob_position))
        ]


@dataclass
class TurnOffStove(TurnOnStove):
    """
    Turn the knob of the burner RoboCasa names back to off.
    """

    knob_position: float = 0.0


@dataclass
class LowerHeat(StoveKnobTask):
    """
    Turn the knob of the burner under the kettle from high back to low heat.
    """

    low_heat_lower_bound: float = 0.35
    """
    The lowest knob position RoboCasa counts as low heat, in radians.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> list[JointGoal]:
        name = self.knob_joint_name(episode)
        low = (self.low_heat_lower_bound + episode.environment.stove.STOVE_HIGH_MIN) / 2
        return [JointGoal(name, episode.joints.world_position_of(name, low))]
