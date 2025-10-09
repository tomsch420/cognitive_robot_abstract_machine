from __future__ import division

from dataclasses import dataclass
from typing import Optional, Union

from semantic_world.connections import Has1DOFState
from semantic_world.prefixed_name import PrefixedName
from giskardpy.motion_statechart.goals.goal import Goal
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from giskardpy.motion_statechart.tasks.task import WEIGHT_ABOVE_CA
from giskardpy.god_map import god_map
import semantic_world.spatial_types.spatial_types as cas
from semantic_world.world_entity import Body, Connection


@dataclass
class Open(Goal):
    tip_link: Body
    environment_link: Body
    goal_joint_state: Optional[float] = None
    max_velocity: float = 100
    weight: float = WEIGHT_ABOVE_CA

    def __post_init__(self):
        """
        Open a container in an environment.
        Only works with the environment was added as urdf.
        Assumes that a handle has already been grasped.
        Can only handle containers with 1 dof, e.g. drawers or doors.
        :param tip_link: end effector that is grasping the handle
        :param environment_link: name of the handle that was grasped
        :param goal_joint_state: goal state for the container. default is maximum joint state.
        :param weight:
        """
        self.handle_link = self.environment_link
        self.connection: Union[Has1DOFState, Connection] = (
            god_map.world.get_controlled_parent_connection(self.handle_link)
        )

        max_position = self.connection.dof.upper_limits.position
        if self.goal_joint_state is None:
            self.goal_joint_state = max_position
        else:
            self.goal_joint_state = min(max_position, self.goal_joint_state)

        goal_state = {self.connection.name: self.goal_joint_state}
        hinge_goal = JointPositionList(
            goal_state=goal_state, name=f"{self.name}/hinge", weight=self.weight
        )
        self.add_task(hinge_goal)

        handle_pose = cas.TransformationMatrix(
            reference_frame=self.tip_link, child_frame=self.tip_link
        )

        hold_handle = CartesianPose(
            root_link=self.handle_link,
            tip_link=self.tip_link,
            name=f"{self.name}/hold handle",
            goal_pose=handle_pose,
            weight=self.weight,
        )
        self.add_task(hold_handle)
        self.observation_expression = cas.logic_and(
            hinge_goal.observation_expression, hold_handle.observation_expression
        )


@dataclass
class Close(Open):

    def __post_init__(self):
        """
        Same as Open, but will use minimum value as default for goal_joint_state
        """
        self.connection: Union[Has1DOFState, Connection] = (
            god_map.world.get_controlled_parent_connection(self.handle_link)
        )
        min_position = self.connection.dof.lower_limits.position
        if self.goal_joint_state is None:
            self.goal_joint_state = min_position
        else:
            self.goal_joint_state = max(min_position, self.goal_joint_state)
        super().__post_init__()
