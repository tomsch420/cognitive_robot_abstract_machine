"""
A PR2 performs RoboCasa's toaster task (``kitchen_toaster.py``): it pushes the lever of
the slot the bread is in down with its fingertips.

Run it with::

    python -m experiments.robocasa_atomic_tasks.toaster TurnOnToaster
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.episode import (
    PushTask,
    RoboCasaEpisode,
    main,
)


@dataclass
class TurnOnToaster(PushTask):
    """
    Push the lever of the slot holding the bread fully down.
    """

    def pushed_joint(self, episode: RoboCasaEpisode) -> tuple[str, float]:
        environment = episode.environment
        toaster = environment.toaster
        episode.mirror.synchronize()
        slot_pair = next(
            (
                slot_pair
                for slot_pair in range(len(toaster.get_state(environment)))
                if toaster.check_slot_contact(environment, "obj", slot_pair)
            ),
            0,
        )
        name = toaster._joint_names[f"lever_{slot_pair}"]
        return name, episode.world_position_at_share(name, 1.0)


TASKS = [TurnOnToaster]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)
