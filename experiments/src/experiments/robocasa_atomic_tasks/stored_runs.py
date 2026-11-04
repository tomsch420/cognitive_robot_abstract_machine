"""
Runs of a task as the world's state moved through them in the physical simulation, kept
in a database and replayed in a world composed anew.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from experiments.robocasa_atomic_tasks.exceptions import (
    DegreesOfFreedomNotInTheWorld,
)
from experiments.robocasa_atomic_tasks.robots import RobotModel
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from semantic_digital_twin.callbacks.callback import StateChangeCallback
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.derivatives import Derivatives

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

logger = logging.getLogger(__name__)

# %% a stored run


@dataclass
class SimulatedMoment:
    """
    The positions of the world's degrees of freedom at one moment of a run.
    """

    simulation_time: float
    """
    How much simulated time had passed since the run began, in seconds.
    """

    positions: list[float]
    """
    The position of every degree of freedom, in the order the run names them.
    """


@dataclass
class StoredTaskRun:
    """
    A run of one of the atomic tasks, as the world's state moved through it.
    """

    task: str
    """
    The name of the task performed.
    """

    layout: LayoutType
    """
    The kitchen layout it was performed in.
    """

    style: StyleType
    """
    The visual style of the kitchen.
    """

    seed: int | None
    """
    The seed RoboCasa sampled the objects and their placements with.
    """

    robot: RobotModel
    """
    The robot that performed it.
    """

    degree_of_freedom_names: list[str]
    """
    The names of the world's degrees of freedom, in the order every moment lists their
    positions.
    """

    moments: list[SimulatedMoment]
    """
    The moments of the run, in the order they passed.
    """


# %% recording a run


@dataclass(eq=False)
class SimulatedMomentRecorder(StateChangeCallback):
    """
    Keeps the world's state as the physical simulation moves it, at most once per
    :attr:`period` of simulated time.

    A state counts once the simulated time has moved on from it, so a state the world
    only passes through while a plan is checked, and returns from at the same moment, is
    never kept.
    """

    simulation: MujocoSim = field(kw_only=True)
    """
    The physical simulation of the world.
    """

    period: float = field(kw_only=True, default=1 / 30)
    """
    How much simulated time at least lies between two kept moments, in seconds.
    """

    moments: list[SimulatedMoment] = field(kw_only=True, default_factory=list)
    """
    The moments kept.
    """

    latest: SimulatedMoment | None = field(kw_only=True, default=None)
    """
    The latest state of the world, at the simulated time it was seen at.
    """

    def on_state_change(self, **kwargs) -> None:
        simulation_time = self.simulation.simulator.current_simulation_time
        if self.latest is not None and simulation_time != self.latest.simulation_time:
            self.keep_latest()
        self.latest = SimulatedMoment(
            simulation_time=simulation_time,
            positions=self._world.state.positions.tolist(),
        )

    def keep_latest(self) -> None:
        """
        Keep the latest state, unless the last kept moment lies less than :attr:`period`
        before it.
        """
        if (
            self.moments
            and self.latest.simulation_time - self.moments[-1].simulation_time
            < self.period
        ):
            return
        self.moments.append(self.latest)

    def finish(self) -> list[SimulatedMoment]:
        """
        Stop recording.

        :return: The moments kept, ending with the latest state.
        """
        self.stop()
        if self.latest is not None and (
            not self.moments or self.moments[-1] is not self.latest
        ):
            self.moments.append(self.latest)
        return self.moments

    @property
    def degree_of_freedom_names(self) -> list[str]:
        """
        The names of the world's degrees of freedom, in the order a moment lists their
        positions.
        """
        world = self._world
        return [
            str(world.get_degree_of_freedom_by_id(identifier).name)
            for identifier in world.state.ids
        ]


# %% replaying a run


@dataclass
class TaskRunReplay:
    """
    Moves a world through the moments of a stored run, as fast as they passed in the
    simulation.
    """

    world: World
    """
    The world replayed in, holding every degree of freedom the run names.
    """

    run: StoredTaskRun
    """
    The run replayed.
    """

    def columns(self) -> list[int]:
        """
        :return: The column of the world's state that each degree of freedom the run
            names stands in.
        :raises DegreesOfFreedomNotInTheWorld: If the world lacks any of them.
        """
        names = {
            str(degree_of_freedom.name): degree_of_freedom
            for degree_of_freedom in self.world.degrees_of_freedom
        }
        missing = [
            name for name in self.run.degree_of_freedom_names if name not in names
        ]
        if missing:
            raise DegreesOfFreedomNotInTheWorld(names=missing)
        return self.world.state.column_indices(
            [names[name] for name in self.run.degree_of_freedom_names]
        )

    def play(self, speed: float = 1.0) -> None:
        """
        Move the world through every moment of the run, waiting between two moments as
        long as the simulated time between them.

        :param speed: How many times faster than it passed the run is replayed.
        """
        columns = self.columns()
        began = time.monotonic()
        start = self.run.moments[0].simulation_time
        for moment in self.run.moments:
            delay = (moment.simulation_time - start) / speed - (
                time.monotonic() - began
            )
            if delay > 0:
                time.sleep(delay)
            positions = self.world.state.positions.copy()
            positions[columns] = moment.positions
            self.world.state.set_derivative(Derivatives.position, positions)
            self.world.notify_state_change()
        logger.info(
            "Replayed %d moments of %s, %.0f s of simulated time",
            len(self.run.moments),
            self.run.task,
            self.run.moments[-1].simulation_time - start,
        )
