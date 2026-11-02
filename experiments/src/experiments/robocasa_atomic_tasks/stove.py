"""
A PR2 performs RoboCasa's stove tasks (``kitchen_stove.py``): it takes hold of the knob
of a burner and turns it.

The tasks run in layout 2, whose cooktop has its knobs along its front edge; layout 1's
range has them on a panel at the back, beyond the reach of a PR2 standing clear of it.

Run one with::

    python -m experiments.robocasa_atomic_tasks.stove TurnOnStove
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import ClassVar, List

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

from experiments.robocasa_atomic_tasks.episode import (
    JointGoalTask,
    RoboCasaEpisode,
    main,
)


def knob_joint_name(episode: RoboCasaEpisode) -> str:
    """
    :param episode: An episode of a stove task.
    :return: The name RoboCasa gives the joint of the knob the task is about.
    """
    stove = episode.environment.stove
    return f"{stove.naming_prefix}knob_{episode.environment.knob}_joint"


@dataclass
class TurnOnStove(JointGoalTask):
    """
    Turn the knob of the burner RoboCasa names from off to on.
    """

    layout: LayoutType = LayoutType.LAYOUT002

    knob_position: ClassVar[float] = 0.8
    """
    How far the knob is turned, in radians: RoboCasa counts a burner on from 0.35 rad.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        name = knob_joint_name(episode)
        return [(name, episode.world_position_of(name, self.knob_position))]


@dataclass
class TurnOffStove(TurnOnStove):
    """
    Turn the knob of the burner RoboCasa names back to off.
    """

    knob_position: ClassVar[float] = 0.0


@dataclass
class LowerHeat(JointGoalTask):
    """
    Turn the knob of the burner under the kettle from high back to low heat.
    """

    layout: LayoutType = LayoutType.LAYOUT002

    low_heat_lower_bound: ClassVar[float] = 0.35
    """
    The lowest knob position RoboCasa counts as low heat, in radians.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        name = knob_joint_name(episode)
        low = (
            self.low_heat_lower_bound + episode.environment.stove.STOVE_HIGH_MIN
        ) / 2
        return [(name, episode.world_position_of(name, low))]


TASKS = [TurnOnStove, TurnOffStove, LowerHeat]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)
