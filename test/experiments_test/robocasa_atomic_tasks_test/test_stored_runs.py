"""
Recording the world's state as a physical simulation moves it, storing the run and
replaying it in another world.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from experiments.robocasa_atomic_tasks.exceptions import (
    DegreesOfFreedomNotInTheWorld,
    NoStoredRun,
)
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.stored_runs import (
    SimulatedMoment,
    SimulatedMomentRecorder,
    StoredTaskRun,
    TaskRunReplay,
)
from experiments.robocasa_atomic_tasks.task_run_database import TaskRunDatabase
from krrood_spatial_math.spatial_types import Vector3
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import RevoluteConnection
from semantic_digital_twin.world_description.world_entity import Body

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

# %% a world with a lever


@dataclass
class AdvancingSimulatedTime:
    """
    Stands in for a physical simulation whose simulated time the test advances.
    """

    current_simulation_time: float = 0.0
    """
    How much simulated time has passed, in seconds.
    """

    @property
    def simulator(self) -> AdvancingSimulatedTime:
        """
        The simulator, which keeps the simulated time.
        """
        return self


@dataclass
class WorldWithALever:
    """
    A world of a root and a lever turning on it.
    """

    world: World
    """
    The world.
    """

    lever_joint: RevoluteConnection
    """
    The joint the lever turns on.
    """

    @property
    def lever_position(self) -> float:
        """
        The position the lever stands at.
        """
        return self.lever_joint.position

    @property
    def lever_name(self) -> str:
        """
        The name of the lever joint's degree of freedom.
        """
        return str(self.lever_joint.dof.name)


@pytest.fixture
def world_with_a_lever() -> WorldWithALever:
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("kitchen"))
        world.add_kinematic_structure_entity(root)
        joint = RevoluteConnection.create_with_dofs(
            world=world,
            parent=root,
            child=Body(name=PrefixedName("lever")),
            axis=Vector3.X(),
        )
        world.add_connection(joint)
    return WorldWithALever(world=world, lever_joint=joint)


def recorded_lever_positions(
    recorder: SimulatedMomentRecorder, lever: WorldWithALever
) -> list[tuple[float, float]]:
    """
    :param recorder: A recorder that finished.
    :param lever: The world it recorded.
    :return: The simulated time and the lever's position of every moment kept.
    """
    column = recorder.degree_of_freedom_names.index(lever.lever_name)
    return [
        (moment.simulation_time, moment.positions[column])
        for moment in recorder.finish()
    ]


# %% recording


def test_a_state_left_at_the_same_simulated_time_is_not_kept(world_with_a_lever):
    lever = world_with_a_lever
    simulated_time = AdvancingSimulatedTime()
    recorder = SimulatedMomentRecorder(_world=lever.world, simulation=simulated_time)

    simulated_time.current_simulation_time = 0.1
    lever.lever_joint.position = 0.5
    with lever.world.reset_state_context():
        lever.lever_joint.position = 0.9
    simulated_time.current_simulation_time = 0.2
    lever.lever_joint.position = 0.6

    assert recorded_lever_positions(recorder, lever) == [(0.1, 0.5), (0.2, 0.6)]


def test_a_moment_closer_than_the_period_to_the_last_kept_one_is_dropped(
    world_with_a_lever,
):
    lever = world_with_a_lever
    simulated_time = AdvancingSimulatedTime()
    recorder = SimulatedMomentRecorder(
        _world=lever.world, simulation=simulated_time, period=0.04
    )

    for simulation_time, position in [(0.0, 0.1), (0.01, 0.2), (0.05, 0.3)]:
        simulated_time.current_simulation_time = simulation_time
        lever.lever_joint.position = position

    assert recorded_lever_positions(recorder, lever) == [(0.0, 0.1), (0.05, 0.3)]


# %% storing


def run_of_the_lever(lever: WorldWithALever, task: str) -> StoredTaskRun:
    """
    :param lever: The world the run moved.
    :param task: The name of the task performed.
    :return: A run that turned the lever to 0.2 and then to 0.4.
    """
    return StoredTaskRun(
        task=task,
        layout=LayoutType.LAYOUT001,
        style=StyleType.STYLE001,
        seed=0,
        robot=RobotModel.PANDA_OMRON,
        degree_of_freedom_names=[lever.lever_name],
        moments=[
            SimulatedMoment(simulation_time=0.0, positions=[0.2]),
            SimulatedMoment(simulation_time=0.1, positions=[0.4]),
        ],
    )


@pytest.fixture
def database() -> TaskRunDatabase:
    return TaskRunDatabase.connect("sqlite:///:memory:")


def test_the_run_of_a_task_stored_last_is_read_back(database, world_with_a_lever):
    earlier = run_of_the_lever(world_with_a_lever, "AdjustWaterTemperature")
    later = run_of_the_lever(world_with_a_lever, "AdjustWaterTemperature")
    later.seed = 1
    database.add(earlier)
    database.add(later)
    database.add(run_of_the_lever(world_with_a_lever, "OpenDrawer"))

    assert database.latest_run_of("AdjustWaterTemperature") == later


def test_reading_the_run_of_a_task_never_stored_raises(database):
    with pytest.raises(NoStoredRun):
        database.latest_run_of("AdjustWaterTemperature")


# %% replaying


def test_a_replay_leaves_the_world_at_the_last_moment(world_with_a_lever):
    lever = world_with_a_lever
    run = run_of_the_lever(lever, "AdjustWaterTemperature")

    TaskRunReplay(world=lever.world, run=run).play(speed=100.0)

    assert lever.lever_position == pytest.approx(run.moments[-1].positions[0])


def test_replaying_in_a_world_without_a_degree_of_freedom_of_the_run_raises(
    world_with_a_lever,
):
    run = run_of_the_lever(world_with_a_lever, "AdjustWaterTemperature")
    run.degree_of_freedom_names.append("faucet_handle")

    with pytest.raises(DegreesOfFreedomNotInTheWorld):
        TaskRunReplay(world=world_with_a_lever.world, run=run).play()
