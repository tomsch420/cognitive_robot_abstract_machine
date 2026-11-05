"""
Runs of a task as the world's state and the plans moved through them in the physical
simulation, kept in a database and replayed in a world composed anew.
"""

from __future__ import annotations

import logging
import time
from collections import deque
from dataclasses import dataclass, field

from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PlanExecutor
from cramph.context import StatechartContext
from cramph.executor import Executor, ExecutorExtension, StatechartExecutor
from cramph.state_history import (
    StateHistory,
    StateHistoryItem,
    StateHistoryObserver,
)
from cramph.statechart import Statechart
from experiments.robocasa_atomic_tasks.exceptions import (
    DegreesOfFreedomNotInTheWorld,
)
from experiments.robocasa_atomic_tasks.judge import Judgement
from experiments.robocasa_atomic_tasks.robots import RobotModel
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from semantic_digital_twin.callbacks.callback import StateChangeCallback
from semantic_digital_twin.robots.robot_parts import AbstractRobot
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
class RecordedPlan:
    """
    A plan performed in a run, in the statechart it ended in, with the simulated time of
    every change of its state.
    """

    statechart: Statechart
    """
    The statechart the plan ran as, holding the history of its state.
    """

    plan_node_indices: list[int]
    """
    The indices of the plan's top-level nodes, without the nodes the executor added to
    run them.
    """

    state_change_times: list[float]
    """
    How much simulated time had passed since the run began when each item of the
    statechart's state history was recorded, in seconds.
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

    judgement: Judgement
    """
    RoboCasa's verdict on the world as the run left it.
    """

    plans: list[RecordedPlan] = field(default_factory=list)
    """
    The plans the robot performed in the run, in the order they began.
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


@dataclass(eq=False)
class PlanStateChangeTimer(StateHistoryObserver):
    """
    Notes the simulated time at which the state of a recorded plan changes.
    """

    plan: RecordedPlan
    """
    The plan whose state history is observed.
    """

    simulation: MujocoSim
    """
    The physical simulation the plan is performed in.
    """

    def on_state_change(self, history: StateHistory) -> None:
        self.plan.state_change_times.append(
            self.simulation.simulator.current_simulation_time
        )


@dataclass
class PlanRecorder(ExecutorExtension):
    """
    Keeps every plan the executors it is added to compile, with the simulated time of
    every change of its state.
    """

    simulation: MujocoSim = field(kw_only=True)
    """
    The physical simulation the plans are performed in.
    """

    plans: list[RecordedPlan] = field(kw_only=True, default_factory=list)
    """
    The plans kept, in the order they were compiled.
    """

    def after_compile(self, executor: PlanExecutor) -> None:
        statechart = executor.statechart
        plan = RecordedPlan(
            statechart=statechart,
            plan_node_indices=[node.index for node in executor.plan_nodes],
            state_change_times=[
                self.simulation.simulator.current_simulation_time
                for _ in statechart.state_history.items
            ],
        )
        self.plans.append(plan)
        statechart.state_history.add_observer(
            PlanStateChangeTimer(plan=plan, simulation=self.simulation)
        )


# %% replaying a run


@dataclass
class ReplayedPlanExecutor(PlanExecutor, StatechartExecutor):
    """
    Shows a plan performed before as it went, without running it: :meth:`compile` takes
    the statechart the plan ended in, and every :meth:`tick` puts it into the next state
    it was recorded in, telling the extensions as a tick of the plan did.
    """

    plan_node_indices: list[int] = field(kw_only=True)
    """
    See :attr:`RecordedPlan.plan_node_indices`.
    """

    recorded_states: list[StateHistoryItem] = field(default_factory=list, init=False)
    """
    The states the statechart was recorded in, in order, as :meth:`compile` took them.
    """

    shown_state_count: int = field(default=0, init=False)
    """
    How many of :attr:`recorded_states` were shown.
    """

    def compile(self, statechart: Statechart) -> None:
        """
        Take `statechart`, which performed the plan, with its recorded states to show
        again and its history emptied, and tell every extension; the statechart is
        neither prepared nor compiled again, nor ticked.

        :param statechart: The statechart, built in :attr:`context`.
        """
        Executor.compile(self, statechart)
        self.plan_nodes = [
            statechart.get_node_by_index(index) for index in self.plan_node_indices
        ]
        self.recorded_states = list(statechart.state_history.items)
        self.shown_state_count = 0
        statechart.state_history = StateHistory()
        self._notify_compiled()

    @property
    def has_shown_every_state(self) -> bool:
        """
        Whether every recorded state was shown.
        """
        return self.shown_state_count == len(self.recorded_states)

    def tick(self) -> None:
        """
        Put the statechart into the next recorded state, every node that joined it later
        not started yet, in place of ticking it.
        """
        ticking_extensions = self.ticking_extensions
        for extension in ticking_extensions:
            extension.before_tick(self)
        recorded = self.recorded_states[self.shown_state_count]
        shown = StateHistoryItem(
            tick_count=recorded.tick_count,
            life_cycle_state=recorded.life_cycle_state.covering_every_node(),
            observation_state=recorded.observation_state.covering_every_node(),
        )
        self.tick_count = recorded.tick_count
        self.statechart.life_cycle_state.data = shown.life_cycle_state.data.copy()
        self.statechart.observation_state.data = shown.observation_state.data.copy()
        self.statechart.state_history.append(shown)
        self.shown_state_count += 1
        for extension in ticking_extensions:
            extension.after_tick(self)

    def _run(self) -> None:
        """
        Show every recorded state not shown yet, then tell every extension that the run
        stopped.
        """
        while not self.has_shown_every_state:
            self.tick()
        self.finish_run()


@dataclass
class PlanReplay:
    """
    One recorded plan of a run, shown by an executor of its own from its first recorded
    state on.
    """

    plan: RecordedPlan
    """
    The plan.
    """

    context: StatechartContext
    """
    The context the plan is shown in, on the world the run is replayed in.
    """

    executor_extensions: list[ExecutorExtension]
    """
    Added to the executor showing the plan.
    """

    executor: ReplayedPlanExecutor | None = field(default=None, init=False)
    """
    The executor showing the plan, once its first state was shown.
    """

    def show_next_state(self) -> None:
        """
        Show the next recorded state of the plan, compiling it before the first, and end
        its run after the last.
        """
        if self.executor is None:
            self.executor = ReplayedPlanExecutor(
                self.context,
                plan_node_indices=self.plan.plan_node_indices,
                extensions=list(self.executor_extensions),
            )
            self.plan.statechart.context = self.context
            self.executor.compile(self.plan.statechart)
        self.executor.tick()
        if self.executor.has_shown_every_state:
            self.executor.finish_run()


@dataclass
class PlanStateChange:
    """
    A change of the state of a replayed plan, at the simulated time it happened at.
    """

    simulation_time: float
    """
    How much simulated time had passed since the run began, in seconds.
    """

    plan: PlanReplay
    """
    The plan whose state changed.
    """


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

    robot: AbstractRobot | None = None
    """
    The robot that performs the run's plans in :attr:`world`, if the run is replayed
    with them.
    """

    executor_extensions: list[ExecutorExtension] = field(default_factory=list)
    """
    Added to the executor showing each plan of the run.
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

    def plan_state_changes(self) -> list[PlanStateChange]:
        """
        :return: Every change of the state of a plan of the run, in the order of the
            simulated time they happened at, each plan shown in a context of its own.
        """
        changes = []
        for plan in self.run.plans:
            context_extensions = [] if self.robot is None else [RobotAccess(self.robot)]
            replay = PlanReplay(
                plan=plan,
                context=StatechartContext.with_extensions(
                    self.world, context_extensions
                ),
                executor_extensions=self.executor_extensions,
            )
            changes.extend(
                PlanStateChange(simulation_time=simulation_time, plan=replay)
                for simulation_time in plan.state_change_times
            )
        return sorted(changes, key=lambda change: change.simulation_time)

    def play(self, speed: float = 1.0) -> None:
        """
        Move the world through every moment of the run, waiting between two moments as
        long as the simulated time between them, and the plans of the run through every
        state they were recorded in by the moment they changed it.

        :param speed: How many times faster than it passed the run is replayed.
        """
        columns = self.columns()
        plan_state_changes = deque(self.plan_state_changes())
        began = time.monotonic()
        start = self.run.moments[0].simulation_time
        for moment in self.run.moments:
            delay = (moment.simulation_time - start) / speed - (
                time.monotonic() - began
            )
            if delay > 0:
                time.sleep(delay)
            while (
                plan_state_changes
                and plan_state_changes[0].simulation_time <= moment.simulation_time
            ):
                plan_state_changes.popleft().plan.show_next_state()
            positions = self.world.state.positions.copy()
            positions[columns] = moment.positions
            self.world.state.set_derivative(Derivatives.position, positions)
            self.world.notify_state_change()
        for change in plan_state_changes:
            change.plan.show_next_state()
        logger.info(
            "Replayed %d moments of %s, %.0f s of simulated time",
            len(self.run.moments),
            self.run.task,
            self.run.moments[-1].simulation_time - start,
        )
