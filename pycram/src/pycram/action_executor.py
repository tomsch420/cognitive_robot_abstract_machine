from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from itertools import groupby

from typing_extensions import TYPE_CHECKING, Any, List, Type

from giskardpy.motion_statechart.goals.templates import Sequence, Parallel

if TYPE_CHECKING:

    from pycram.plans.plan import Plan
    from pycram.plans.plan_node import ActionNode, PlanNode, MotionNode
    from semantic_digital_twin.world import World
    from pycram.robot_plans import BaseMotion
    from pycram.plans.condition_nodes import ConditionNode
    from pycram.language import LanguageNode


@dataclass
class ActionExecutor:
    action_node: ActionNode
    """
    Root node of the action sub-plan that should be executed
    """

    plan: Plan
    """
    Plan to which the action node belongs
    """

    world: World
    """
    World in which the action should be executed.
    """

    def execute(self):
        pass


@dataclass
class GraphParser(ABC):
    """
    Base class for parsing a plan graph into executable units.
    """

    @abstractmethod
    def parse(self) -> Any:
        """
        Parses the graph and returns the result.
        """
        pass

    def expand_sub_actions(self, children: List[PlanNode]) -> List[Executable]:
        """
        Expands sub-actions within a list of plan nodes into executables.

        :param children: The list of plan nodes to expand.
        :return: A list of executables.
        """
        from pycram.plans.plan_node import ActionNode, MotionNode
        from pycram.language import LanguageNode

        result = []

        for child in children:
            if isinstance(child, ActionNode):
                result.append(*ActionGraphParser(child).parse())
            elif isinstance(child, LanguageNode):
                result.append(*LanguageGraphParser(child).parse())
            elif isinstance(child, MotionNode):
                result.append(MotionExecutable(child))
            else:
                result.append(child)
        return result

    def cluster_list_by_type(
        self, flat_list: List, cluster_type: Type[Any]
    ) -> List[List[Executable]]:
        groups = list(
            (
                list(g)
                for _, g in groupby(
                    flat_list, key=lambda m: isinstance(m, cluster_type)
                )
            )
        )
        return groups


@dataclass
class ActionGraphParser(GraphParser):
    """
    Parser for action nodes in a plan graph.
    """

    action_node: ActionNode
    """
    The action node to parse.
    """

    def parse(self) -> List[Executable]:
        """
        Parses the action node and its children into a list of executables.

        :return: A list of executables.
        """
        children = self.action_node.children
        pre_condition_executable = ConditionExecutable(children.pop(0))
        post_condition_executable = ConditionExecutable(children.pop(-1))

        return self.expand_sub_actions(children)


@dataclass
class LanguageGraphParser(GraphParser):

    language_node: LanguageNode

    def parse(self) -> List[Executable]:
        executables: List[Executable] = []

        for child in self.language_node.descendants:
            if isinstance(child, LanguageNode):
                executables.append(LanguageGraphParser(child).parse())
            elif isinstance(child, ActionNode):
                executables.append(ActionGraphParser(child).parse())
            elif isinstance(child, MotionNode):
                executables.append(MotionExecutable(child))

    def _build_sequential_executable(self, motions): ...


@dataclass
class MotionsParser(GraphParser):
    """
    Parser for motion descriptions.
    """

    motions: List[BaseMotion]
    """
    The list of base motions to parse.
    """

    def parse(self) -> Any:
        """
        Parses the motions.
        """
        pass


@dataclass
class Executable:
    """
    Base class for executable units.
    """

    def execute(self) -> None:
        """
        Executes the unit.
        """
        pass


@dataclass
class LanguageExecutable(Executable):

    motions: List[MotionExecutable]

    def execute(self) -> None:
        pass

    @property
    def motion_state_chart(self):
        return Parallel(
            nodes=[
                motion.motion_node.designator.motion_chart for motion in self.motions
            ]
        )


@dataclass
class SequentialExecutable(LanguageExecutable):

    @property
    def motion_state_chart(self):
        return Sequence(
            nodes=[
                motion.motion_node.designator.motion_chart for motion in self.motions
            ]
        )


@dataclass
class ParallelExecutable(LanguageExecutable):

    def execute(self) -> None:
        pass

    @property
    def motion_state_chart(self):
        return Parallel(
            nodes=[
                motion.motion_node.designator.motion_chart for motion in self.motions
            ]
        )


@dataclass
class ConditionExecutable(Executable):
    """
    An executable unit for a condition node.
    """

    condition_node: ConditionNode
    """
    The condition node to execute.
    """

    def execute(self) -> None:
        """
        Executes the condition node.
        """
        pass


@dataclass
class MotionExecutable(Executable):

    motion_node: MotionNode

    def execute(self) -> None:
        pass


@dataclass
class ExecutionPackage:
    """
    A package of executables to be executed sequentially.
    """

    execution_list: List[Executable]
    """
    The list of executables.
    """

    def execute(self) -> None:
        """
        Executes all executables in the package.
        """
        for executable in self.execution_list:
            executable.execute()
