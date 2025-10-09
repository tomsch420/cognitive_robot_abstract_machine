from dataclasses import dataclass
from typing import Dict, Optional, Union

import semantic_world.spatial_types.spatial_types as cas
from giskardpy.data_types.exceptions import GoalInitalizationException
from giskardpy.motion_statechart.monitors.monitors import Monitor
from giskardpy.god_map import god_map
from semantic_world.connections import (
    Has1DOFState,
    RevoluteConnection,
    ActiveConnection,
)
from semantic_world.prefixed_name import PrefixedName
from semantic_world.spatial_types.derivatives import Derivatives


@dataclass
class JointGoalReached(Monitor):
    goal_state: Dict[Union[PrefixedName, str], float]
    threshold: float = 0.01

    def __post_init__(self):
        comparison_list = []
        for joint_name, goal in self.goal_state.items():
            connection: Has1DOFState = god_map.world.get_connection_by_name(joint_name)
            current = connection.dof.symbols.position
            if (
                isinstance(connection, RevoluteConnection)
                and connection.dof.has_position_limits()
            ):
                error = cas.shortest_angular_distance(current, goal)
            else:
                error = goal - current
            comparison_list.append(cas.less(cas.abs(error), self.threshold))
        expression = cas.logic_all(cas.Expression(comparison_list))
        self.observation_expression = expression


@dataclass
class JointPositionAbove(Monitor):
    connection: ActiveConnection
    threshold: float

    def __post_init__(self):
        if not isinstance(self.connection, Has1DOFState):
            raise GoalInitalizationException(
                f"Connection {self.connection} must be of type Has1DOFState"
            )
        if (
            isinstance(self.connection, RevoluteConnection)
            and self.connection.dof.has_position_limits()
        ):
            raise GoalInitalizationException(
                f"{self.__class__.__name__} does not support joints of type continuous."
            )

        current = self.connection.dof.symbols.position
        expression = cas.greater(current, self.threshold)
        self.observation_expression = expression
