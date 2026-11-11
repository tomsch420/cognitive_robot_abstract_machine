"""
Tests of a node that adds its task to the statechart, and so expands it, only once it
starts.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from cramph.composites import Deferred, Sequence
from cramph.data_types import LifeCycleValues, ObservationStateValues
from cramph.executor import StatechartExecutor
from cramph.monitors import CountTicks
from cramph.node import (
    CompositeNode,
    EndStatechart,
    NodeArtifacts,
    StatechartNode,
    SucceedsOnObservingTrue,
    expanded_child_field,
)
from cramph.nodes_for_testing import (
    NodeFailingOnObservingFalse,
    NodeSucceedingOnObservingTrue,
)
from cramph.statechart import Statechart

# %% mimics


@dataclass(repr=False, eq=False, kw_only=True)
class NodeNotingWhatRanBeforeItExpanded(SucceedsOnObservingTrue, CompositeNode):
    """
    A composite node that, when it expands, notes the life cycle state of a node it is
    told ran before it, and then runs a single child observing True.
    """

    ran_before: StatechartNode
    """
    The node whose life cycle state is noted.
    """

    noted_states: list[LifeCycleValues] = field(default_factory=list, init=False)
    """
    The life cycle state :attr:`ran_before` had each time this node expanded.
    """

    child: NodeSucceedingOnObservingTrue = expanded_child_field()
    """
    The child observing True.
    """

    def expand(self) -> None:
        self.noted_states.append(self.ran_before.life_cycle_state)
        self.child = NodeSucceedingOnObservingTrue(
            name="child", observation=ObservationStateValues.TRUE
        )
        self._add_child_to_statechart(self.child)

    def build_artifacts(self) -> NodeArtifacts:
        return NodeArtifacts(observation=self.child.observation_variable)


def _run_until_it_ends(
    executor: StatechartExecutor, node: StatechartNode
) -> Statechart:
    """
    Run a statechart holding `node` until `node` succeeds or fails.

    :return: The statechart that ran.
    """
    statechart = Statechart(context=executor.context)
    statechart.add_node(node)
    statechart.add_node(EndStatechart.when_true(node))
    statechart.add_node(EndStatechart.when_failed(node))
    executor.compile(statechart)
    executor.tick_until_end(timeout=30)
    return statechart


# %% expanding the task once started


def test_the_task_expands_only_after_the_steps_before_it_ended(
    statechart_executor: StatechartExecutor,
) -> None:
    first = CountTicks(name="first", ticks=3)
    task = NodeNotingWhatRanBeforeItExpanded(name="task", ran_before=first)

    _run_until_it_ends(
        statechart_executor,
        Sequence(name="steps", nodes=[first, Deferred(name="deferred", task=task)]),
    )

    assert task.noted_states == [LifeCycleValues.SUCCEEDED]


def test_the_node_succeeds_once_its_task_succeeded(
    statechart_executor: StatechartExecutor,
) -> None:
    deferred = Deferred(
        name="deferred",
        task=NodeSucceedingOnObservingTrue(
            name="task", observation=ObservationStateValues.TRUE
        ),
    )

    _run_until_it_ends(statechart_executor, deferred)

    assert deferred.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_the_node_fails_once_its_task_failed_instead_of_running_it_again(
    statechart_executor: StatechartExecutor,
) -> None:
    task = NodeFailingOnObservingFalse(
        name="task", observation=ObservationStateValues.FALSE
    )
    deferred = Deferred(name="deferred", task=task)

    _run_until_it_ends(statechart_executor, deferred)

    assert deferred.children == [task]
    assert deferred.life_cycle_state == LifeCycleValues.FAILED
