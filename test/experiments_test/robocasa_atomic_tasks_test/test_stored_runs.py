"""
Recording the world's state and the plans as a physical simulation moves them, storing
the run and replaying it in another world.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import pytest

from coraplex.plans.executors import KinematicallySimulatedPlanExecutor, PlanExecutor
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.executor import StatechartExecutor, TickingExecutorExtension
from cramph.monitors import CountTicks
from cramph.statechart import Statechart
from experiments.robocasa_atomic_tasks.exceptions import (
    DegreesOfFreedomNotInTheWorld,
    NoStoredRun,
)
from experiments.robocasa_atomic_tasks.judge import (
    JudgedJointPosition,
    JudgedObject,
    Judgement,
)
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.stored_runs import (
    PlanRecorder,
    RecordedPlan,
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


# %% a plan performed in the world


TICK_DURATION = 0.1
"""
How much simulated time passes with every tick of :func:`recorded_plan`, in seconds.
"""


@dataclass
class ExtensionNotingWhatItIsShown(TickingExecutorExtension):
    """
    An executor extension that notes the plan it was shown, every state it was shown
    after a tick and how often a run stopped.
    """

    plan_node_names: list[str] = field(default_factory=list)
    """
    The names of the top-level nodes of the plan compiled.
    """

    life_cycle_values: list[list[float]] = field(default_factory=list)
    """
    The life cycle value of every node after each tick.
    """

    stopped_runs: int = 0
    """
    How often a run stopped.
    """

    def after_compile(self, executor: PlanExecutor) -> None:
        self.plan_node_names = [str(node.name) for node in executor.plan_nodes]

    def after_tick(self, executor: StatechartExecutor) -> None:
        self.life_cycle_values.append(
            executor.statechart.life_cycle_state.data.tolist()
        )

    def after_run(self, executor: PlanExecutor) -> None:
        self.stopped_runs += 1


@pytest.fixture
def recorded_plan(world_with_a_lever) -> RecordedPlan:
    """
    :return: A plan of two steps counting ticks, performed while the simulated time
        advanced by :data:`TICK_DURATION` before every tick.
    """
    simulated_time = AdvancingSimulatedTime()
    recorder = PlanRecorder(simulation=simulated_time)
    executor = KinematicallySimulatedPlanExecutor(
        StatechartContext(world=world_with_a_lever.world), extensions=[recorder]
    )
    statechart = Statechart(context=executor.context)
    statechart.add_node(
        Sequence(
            name="count ticks twice",
            nodes=[
                CountTicks(name="count two ticks", ticks=2),
                CountTicks(name="count three ticks", ticks=3),
            ],
        )
    )
    executor.compile(statechart)
    while not statechart.is_ended():
        simulated_time.current_simulation_time += TICK_DURATION
        executor.tick()
    executor.finish_run()
    return recorder.plans[0]


# %% recording


def test_a_recorded_plan_holds_the_simulated_time_of_every_change_of_its_state(
    recorded_plan,
):
    assert recorded_plan.state_change_times == pytest.approx(
        [
            item.tick_count * TICK_DURATION
            for item in recorded_plan.statechart.state_history.items
        ]
    )


def test_a_recorded_plan_names_its_top_level_nodes_without_what_the_executor_added(
    recorded_plan,
):
    statechart = recorded_plan.statechart
    plan_nodes = [
        statechart.get_node_by_index(index) for index in recorded_plan.plan_node_indices
    ]

    assert plan_nodes == [
        node for node in statechart.top_level_nodes if node.name == "count ticks twice"
    ]


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
        judgement=Judgement(
            accomplished=False,
            judged_objects=[
                JudgedObject(
                    name="obj",
                    position=[1.0, 2.0, 0.9],
                    distance_to_gripper=0.3,
                    touched_bodies=["counter_main", "obj_main"],
                )
            ],
            judged_joint_positions=[
                JudgedJointPosition(
                    joint_name="sink_main_group_handle_joint", position=0.38
                )
            ],
        ),
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


def test_the_plans_of_a_run_are_read_back(database, world_with_a_lever, recorded_plan):
    run = run_of_the_lever(world_with_a_lever, "AdjustWaterTemperature")
    run.plans = [recorded_plan]
    database.add(run)

    read_back = database.latest_run_of("AdjustWaterTemperature").plans

    assert [
        (
            [(type(node), node.name) for node in plan.statechart.nodes],
            plan.plan_node_indices,
            plan.state_change_times,
        )
        for plan in read_back
    ] == [
        (
            [(type(node), node.name) for node in recorded_plan.statechart.nodes],
            recorded_plan.plan_node_indices,
            recorded_plan.state_change_times,
        )
    ]


def test_the_judgement_of_a_run_is_read_back(database, world_with_a_lever):
    run = run_of_the_lever(world_with_a_lever, "AdjustWaterTemperature")
    database.add(run)

    assert database.latest_run_of("AdjustWaterTemperature").judgement == run.judgement


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


def run_with_the_recorded_plan(
    lever: WorldWithALever, plan: RecordedPlan
) -> StoredTaskRun:
    """
    :param lever: The world the run moved.
    :param plan: The plan the robot performed in the run.
    :return: :func:`run_of_the_lever` with `plan`.
    """
    run = run_of_the_lever(lever, "AdjustWaterTemperature")
    run.plans = [plan]
    return run


def test_a_replayed_plan_is_shown_with_its_top_level_nodes(
    world_with_a_lever, recorded_plan
):
    shown = ExtensionNotingWhatItIsShown()

    TaskRunReplay(
        world=world_with_a_lever.world,
        run=run_with_the_recorded_plan(world_with_a_lever, recorded_plan),
        executor_extensions=[shown],
    ).play(speed=1000.0)

    assert shown.plan_node_names == ["count ticks twice"]


def test_a_replayed_plan_goes_through_every_state_it_was_recorded_in(
    world_with_a_lever, recorded_plan
):
    recorded_life_cycle_values = [
        item.life_cycle_state.covering_every_node().data.tolist()
        for item in recorded_plan.statechart.state_history.items
    ]
    shown = ExtensionNotingWhatItIsShown()

    TaskRunReplay(
        world=world_with_a_lever.world,
        run=run_with_the_recorded_plan(world_with_a_lever, recorded_plan),
        executor_extensions=[shown],
    ).play(speed=1000.0)

    assert shown.life_cycle_values == recorded_life_cycle_values


def test_the_run_of_a_replayed_plan_stops_once_after_its_last_state(
    world_with_a_lever, recorded_plan
):
    shown = ExtensionNotingWhatItIsShown()

    TaskRunReplay(
        world=world_with_a_lever.world,
        run=run_with_the_recorded_plan(world_with_a_lever, recorded_plan),
        executor_extensions=[shown],
    ).play(speed=1000.0)

    assert shown.stopped_runs == 1
