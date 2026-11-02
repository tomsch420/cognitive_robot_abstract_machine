"""
A PR2 performs RoboCasa's toaster task (``kitchen_toaster.py``): it pushes the lever of
the slot the bread is in down with its fingertips.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robocasa_names import (
    FixtureJointRole,
    RoboCasaObjectName,
)
from experiments.robocasa_atomic_tasks.tasks import JointGoal, PushTask

# %% the lever


@dataclass
class TurnOnToaster(PushTask):
    """
    Push the lever of the slot pair holding the bread fully down, the first slot pair's
    if none does.
    """

    def pushed_joint(self, episode: RoboCasaEpisode) -> JointGoal:
        environment = episode.environment
        toaster = environment.toaster
        episode.judge.mirror.synchronize()
        slot_pair = next(
            (
                slot_pair
                for slot_pair in range(len(toaster.get_state(environment)))
                if toaster.check_slot_contact(
                    environment, RoboCasaObjectName.OBJECT, slot_pair
                )
            ),
            0,
        )
        name = toaster._joint_names[FixtureJointRole.LEVER.of_slot_pair(slot_pair)]
        return JointGoal(name, episode.joints.world_position_at_share(name, 1.0))
