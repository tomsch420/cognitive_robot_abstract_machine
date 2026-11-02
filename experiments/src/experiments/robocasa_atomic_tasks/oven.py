"""
A PR2 performs RoboCasa's oven tasks (``kitchen_oven.py``): it turns the oven's
temperature knob and slides its rack, in layout 2, since layout 1 has no oven.

Run one with::

    python -m experiments.robocasa_atomic_tasks.oven PreheatOven
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
    DoorShareTask,
    JointGoalTask,
    RoboCasaEpisode,
    main,
    rack_joint_name,
)


@dataclass
class PreheatOven(JointGoalTask):
    """
    Turn the oven's temperature knob up.
    """

    layout: LayoutType = LayoutType.LAYOUT002

    temperature_share: ClassVar[float] = 0.5
    """
    How far the temperature knob is turned, as a share of its range: RoboCasa counts
    the oven preheating from 0.25.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        name = episode.environment.oven._joint_names["temperature"]
        return [
            (name, episode.world_position_along_range(name, self.temperature_share))
        ]


@dataclass
class SlideOvenRack(DoorShareTask):
    """
    Pull the oven's rack fully out, or push it fully in, as RoboCasa asks.
    """

    layout: LayoutType = LayoutType.LAYOUT002
    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        environment = episode.environment
        return [rack_joint_name(episode, environment.oven, environment.rack_level)]

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        share = 1.0 if episode.environment.should_pull else 0.0
        return [
            (name, episode.world_position_at_share(name, share))
            for name in self.joint_names(episode)
        ]


TASKS = [PreheatOven, SlideOvenRack]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)
