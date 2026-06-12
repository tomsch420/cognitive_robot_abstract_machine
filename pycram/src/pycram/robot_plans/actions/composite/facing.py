from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

import numpy as np
from typing_extensions import Optional, Any

from pycram.config.action_conf import ActionConfig
from pycram.plans.factories import sequential
from pycram.plans.plan_node import PlanNode
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.actions.core.navigation import NavigateAction, LookAtAction
from semantic_digital_twin.spatial_types import (
    Quaternion,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose


@dataclass
class FaceAtAction(ActionDescription):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.
    """

    pose: Pose
    """
    The pose to face 
    """
    keep_joint_states: bool = ActionConfig.face_at_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    @property
    def _action_plan(self) -> PlanNode:
        # get the robot position
        robot_position = self.robot.root.global_transform

        # calculate orientation for robot to face the object
        angle = (
            np.arctan2(
                robot_position.y - self.pose.y,
                robot_position.x - self.pose.x,
            )
            + np.pi
        )

        # create new robot pose
        new_robot_pose = Pose(
            robot_position.to_position(),
            Quaternion.from_rpy(0, 0, angle),
            reference_frame=self.world.root,
        )

        return sequential(
            [
                NavigateAction(
                    new_robot_pose, self.keep_joint_states
                ).action_plan_for_context(
                    self.context
                ),  # turn robot
                LookAtAction(self.pose).action_plan_for_context(
                    self.context
                ),  # look at the target
            ]
        )

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        # The validation will be done in the LookAtActionPerformable.perform() method so no need to validate here.
        pass
