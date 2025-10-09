from dataclasses import dataclass
from typing import Optional, Union

import semantic_world.spatial_types.spatial_types as cas
from giskardpy.motion_statechart.goals.goal import Goal
from giskardpy.god_map import god_map
from giskardpy.motion_statechart.tasks.task import WEIGHT_BELOW_CA, Task

import numpy as np

from semantic_world.world_entity import Body


@dataclass
class PrePushDoor(Goal):
    root_link: Body
    tip_link: Body
    door_object: Body
    door_handle: Body
    reference_linear_velocity: float = 0.1
    reference_angular_velocity: float = 0.5
    weight: float = WEIGHT_BELOW_CA

    def __post_init__(self):
        """
        The objective is to push the object until desired rotation is reached
        """
        object_joint_name = god_map.world.get_controlled_parent_connection(
            self.door_object
        )
        object_V_object_rotation_axis = (
            object_joint_name.origin.to_quaternion().to_axis_angle()
        )

        root_T_tip = god_map.world.compose_forward_kinematics_expression(
            self.root_link, self.tip_link
        )
        root_T_door = god_map.world.compose_forward_kinematics_expression(
            self.root_link, self.door_object
        )
        door_P_handle = god_map.world.compute_forward_kinematics(
            self.door_object, self.door_handle
        )
        temp_point = np.asarray(
            [door_P_handle.x.to_np(), door_P_handle.y.to_np(), door_P_handle.z.to_np()]
        )

        door_V_v1 = np.zeros(3)
        # axis pointing in the direction of handle frame from door joint frame
        direction_axis = np.argmax(abs(temp_point))
        door_V_v1[direction_axis] = 1
        door_V_v2 = object_V_object_rotation_axis  # B
        door_V_v1 = cas.Vector3(door_V_v1)  # A

        door_Pose_tip = god_map.world.compose_forward_kinematics_expression(
            self.door_object, self.tip_link
        )
        door_P_tip = door_Pose_tip.to_position()
        dist, door_P_nearest = cas.distance_point_to_plane(
            door_P_tip, door_V_v2, door_V_v1
        )

        root_P_nearest_in_rotated_door = cas.dot(
            cas.TransformationMatrix(root_T_door), cas.Point3(door_P_nearest)
        )

        god_map.debug_expression_manager.add_debug_expression(
            "goal_point_on_plane", cas.Point3(root_P_nearest_in_rotated_door)
        )

        push_door_task = Task(name="pre push door")
        self.add_task(push_door_task)
        push_door_task.add_point_goal_constraints(
            frame_P_current=root_T_tip.to_position(),
            frame_P_goal=cas.Point3(root_P_nearest_in_rotated_door),
            reference_velocity=self.reference_linear_velocity,
            weight=self.weight,
        )
