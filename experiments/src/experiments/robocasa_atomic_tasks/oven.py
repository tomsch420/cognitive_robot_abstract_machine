"""
A PR2 performs RoboCasa's oven tasks (``kitchen_oven.py``): it turns the oven's
temperature knob and slides its rack, in layout 2, since layout 1 has no oven.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robocasa_names import FixtureJointRole
from experiments.robocasa_atomic_tasks.tasks import (
    DoorShareTask,
    JointGoal,
    JointGoalTask,
)
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

# %% the oven's tasks


@dataclass
class PreheatOven(JointGoalTask):
    """
    Turn the oven's temperature knob up.
    """

    layout: LayoutType = LayoutType.LAYOUT002

    temperature_share: float = 0.5
    """
    How far the temperature knob is turned, as a share of its range: RoboCasa counts the
    oven preheating from 0.25.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> list[JointGoal]:
        name = episode.environment.oven._joint_names[FixtureJointRole.TEMPERATURE]
        return [
            JointGoal(
                name,
                episode.joints.world_position_along_range(name, self.temperature_share),
            )
        ]


@dataclass
class SlideOvenRack(DoorShareTask):
    """
    Pull the oven's rack fully out, or push it fully in, as RoboCasa asks.
    """

    layout: LayoutType = LayoutType.LAYOUT002

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        environment = episode.environment
        return [
            episode.joints.rack_joint_name(environment.oven, environment.rack_level)
        ]

    def target_share(self, episode: RoboCasaEpisode) -> float:
        return 1.0 if episode.environment.should_pull else 0.0
