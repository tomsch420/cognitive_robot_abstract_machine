"""
Every one of RoboCasa's atomic tasks the PR2 performs, found in the modules named after
the RoboCasa files that define them.
"""

from __future__ import annotations

import inspect
from enum import Enum
from types import ModuleType

from robosuite.environments.base import REGISTERED_ENVS

from experiments.robocasa_atomic_tasks import (
    blender,
    coffee,
    doors,
    drawer,
    electric_kettle,
    microwave,
    navigate,
    oven,
    pick_place,
    sink,
    stand_mixer,
    stove,
    toaster,
    toaster_oven,
)
from experiments.robocasa_atomic_tasks.tasks import RoboCasaAtomicTask
from krrood.ormatic.utils import classproperty
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    import robocasa.environments  # noqa: F401  registers RoboCasa's environments

# %% the modules of the tasks


class TaskModule(Enum):
    """
    The modules of the tasks, one per file RoboCasa defines its atomic tasks in.
    """

    BLENDER = blender
    COFFEE = coffee
    DOORS = doors
    DRAWER = drawer
    ELECTRIC_KETTLE = electric_kettle
    MICROWAVE = microwave
    NAVIGATE = navigate
    OVEN = oven
    PICK_PLACE = pick_place
    SINK = sink
    STAND_MIXER = stand_mixer
    STOVE = stove
    TOASTER = toaster
    TOASTER_OVEN = toaster_oven

    @property
    def tasks(self) -> list[type[RoboCasaAtomicTask]]:
        """
        The tasks defined in the module: its concrete subclasses of
        :class:`~experiments.robocasa_atomic_tasks.tasks.RoboCasaAtomicTask` named after
        one of RoboCasa's environments.
        """
        module: ModuleType = self.value
        return [
            candidate
            for candidate in vars(module).values()
            if isinstance(candidate, type)
            and issubclass(candidate, RoboCasaAtomicTask)
            and candidate.__module__ == module.__name__
            and not inspect.isabstract(candidate)
            and candidate.robocasa_task_name in REGISTERED_ENVS
        ]

    @classproperty
    def tasks_by_name(cls) -> dict[str, type[RoboCasaAtomicTask]]:
        """
        Every task of every module, by the name of the RoboCasa task it performs.
        """
        return {
            task.robocasa_task_name: task for module in cls for task in module.tasks
        }
