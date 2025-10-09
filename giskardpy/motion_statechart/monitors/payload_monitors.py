from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple

import numpy as np
from line_profiler import profile

import semantic_world.spatial_types.spatial_types as cas
from giskardpy.motion_statechart.data_types import ObservationState
from giskardpy.god_map import god_map
from giskardpy.middleware import get_middleware
from giskardpy.motion_statechart.monitors.monitors import PayloadMonitor, Monitor


@dataclass
class CheckMaxTrajectoryLength(Monitor):
    length: float

    def __post_init__(self):
        self.observation_expression = cas.greater(god_map.time_symbol, self.length)


@dataclass
class Print(PayloadMonitor):
    message = str

    def __call__(self):
        get_middleware().loginfo(self.message)
        self.state = ObservationState.true


@dataclass
class Sleep(PayloadMonitor):
    seconds: float
    start_time: float = field(default=None, init=False)

    def __post_init__(self):
        self.start_time = None

    def __call__(self):
        if self.start_time is None:
            self.start_time = god_map.time
        self.state = god_map.time - self.start_time >= self.seconds


@dataclass
class CollisionMatrixUpdater(PayloadMonitor):
    new_collision_matrix: Dict[Tuple[str, str], float]

    @profile
    def __call__(self):
        god_map.collision_scene.set_collision_matrix(self.new_collision_matrix)
        god_map.collision_scene.reset_cache()
        self.state = ObservationState.true


@dataclass
class PayloadAlternator(PayloadMonitor):
    mod: int = 2

    def __call__(self):
        self.state = np.floor(god_map.time) % self.mod == 0


@dataclass
class Counter(PayloadMonitor):
    number: int
    counter: int = field(default=0, init=False)

    def __call__(self):
        if self.state == ObservationState.unknown:
            self.counter = 0
        if self.counter >= self.number:
            self.state = ObservationState.true
        else:
            self.state = ObservationState.false
        self.counter += 1


@dataclass
class Pulse(PayloadMonitor):
    after_ticks: int
    true_for_ticks: int = 1
    ticks: int = field(default=0, init=False)

    def __call__(self):
        if self.state == ObservationState.unknown:
            self.counter = 0
        if self.after_ticks <= self.counter <= self.after_ticks + self.true_for_ticks:
            self.state = ObservationState.true
        else:
            self.state = ObservationState.false
        self.counter += 1
