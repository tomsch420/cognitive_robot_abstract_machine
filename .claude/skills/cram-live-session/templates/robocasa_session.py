# %% cell setup: the task, its world and robot, shown live in cramera
from __future__ import annotations

import logging
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO)
logging.getLogger("physics_simulators").setLevel(logging.WARNING)
from cramera_bridge.coraplex_backend import CoraplexBackendRegistration

registration = CoraplexBackendRegistration()
registration.__enter__()
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink import TurnOnSinkFaucet

task = TurnOnSinkFaucet(robot=RobotModel("panda_omron"))
episode = task.compose()
world = episode.world

# %% cell physics
task.physics = True
episode.collision_avoidance = task.collision_avoidance
episode.simulation = episode.start_physical_simulation()
executor = episode.create_executor()

# %% cell the world's lock takes the viewer's lock first
# Workaround: cramera's callbacks take the publisher's lock inside the world's lock, while
# its threads take them the other way round, which deadlocks a session. Drop this cell once
# cramera keeps the world's lock outermost.
import threading
from dataclasses import dataclass


@dataclass
class ViewerFirstLock:
    """
    The world's lock, taken only together with the viewer's lock, the viewer's first.
    """

    viewer_lock: threading.RLock
    """
    The cramera publisher's lock.
    """

    world_lock: threading.RLock
    """
    The world's own lock.
    """

    def acquire(self, blocking: bool = True, timeout: float = -1) -> bool:
        """
        :return: Whether both locks were taken; neither is held otherwise.
        """
        if not self.viewer_lock.acquire(blocking, timeout):
            return False
        if self.world_lock.acquire(blocking, timeout):
            return True
        self.viewer_lock.release()
        return False

    def release(self) -> None:
        """
        Release both locks, the world's first.
        """
        self.world_lock.release()
        self.viewer_lock.release()

    def __enter__(self) -> ViewerFirstLock:
        self.acquire()
        return self

    def __exit__(self, *exception: object) -> None:
        self.release()


world._world_lock = ViewerFirstLock(
    registration.live.publisher.world_lock, world._world_lock
)

# %% cell the live session
from experiments.live_session.session import LiveSession

session = LiveSession.from_episode(episode)
session.take_snapshot("composed")
session.report("session started")
