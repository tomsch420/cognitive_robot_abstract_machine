"""
The robot sets how hot a mixer tap runs by swinging the tap's lever with one fingertip
of the open gripper.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from coraplex.robot_plans.actions.base import SequentialAction
from cramph.node import StatechartNode
from experiments.robocasa_atomic_tasks.joints import FixtureJoints
from experiments.robocasa_atomic_tasks.robocasa_names import WaterTemperature
from experiments.robocasa_atomic_tasks.skills import (
    FingerApproach,
    FingerPressAction,
    FingerWithdrawAction,
    PushAction,
)
from krrood.entity_query_language.factories import a, variable
from krrood.entity_query_language.query.match import Match
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.semantic_annotations.semantic_annotations import MixerLever

# %% swinging the lever


@dataclass(eq=False, repr=False, kw_only=True)
class AdjustWaterTemperatureAction(SequentialAction):
    """
    Swings the lever of a mixer tap on its temperature joint as far as it goes towards
    hot or cold water: sets one fingertip of the open gripper against the lever's part
    farthest from that joint, swings the joint to its end while the fingertip stays on
    the lever, and draws the gripper back from where the lever left it.

    .. note:: The robot stands within reach of the lever when the action begins.
    """

    lever: MixerLever
    """
    The lever swung.
    """

    temperature: WaterTemperature
    """
    How hot the water is to run.
    """

    arm: Arm
    """
    The arm whose fingertip swings the lever.
    """

    finger_approach: FingerApproach
    """
    How the open gripper comes in beside the lever, its fingers closing along the way
    the lever is pushed.
    """

    threshold_share: float = 0.002
    """
    How close the temperature joint has to come to its end, as a share of its travel;
    tight, since the lever comes to rest a little short of where the push leaves it.
    """

    @classmethod
    def finger_approaches(
        cls, lever: MixerLever, temperature: WaterTemperature
    ) -> list[FingerApproach]:
        """
        :param lever: The lever swung.
        :param temperature: How hot the water is to run.
        :return: The ways the gripper may come in beside the lever, best first, see
            :meth:`FingerApproach.across`.
        """
        joint = lever.temperature_joint
        end = temperature.temperature_joint_position(lever)
        point = FixtureJoints.point_for_a_fingertip(joint, lever.root, end)
        return FingerApproach.across(
            FixtureJoints.push_direction(joint, end, point=point)
        )

    @classmethod
    def from_finger_approaches(
        cls, lever: MixerLever, temperature: WaterTemperature, arm: Arm
    ) -> Match:
        """
        :param lever: The lever swung.
        :param temperature: How hot the water is to run.
        :param arm: The arm whose fingertip swings the lever.
        :return: The action, with the way the gripper comes in left open among
            :meth:`finger_approaches`.
        """
        return a(cls)(
            lever=lever,
            temperature=temperature,
            arm=arm,
            finger_approach=variable(
                FingerApproach, domain=cls.finger_approaches(lever, temperature)
            ),
        )

    def create_steps(self) -> list[StatechartNode]:
        joint = self.lever.temperature_joint
        end = self.temperature.temperature_joint_position(self.lever)
        point = FixtureJoints.point_for_a_fingertip(joint, self.lever.root, end)
        approach = self.finger_approach.approach
        closing = self.finger_approach.closing
        world_T_swung = self.swing_of_the_lever_to(end)
        return [
            PushAction(
                press=FingerPressAction(
                    point=point, approach=approach, closing=closing, arm=self.arm
                ),
                part=self.lever.root,
                joint=joint,
                goal_position=end,
                threshold=self.threshold_share * FixtureJoints.travel_of(joint),
            ),
            FingerWithdrawAction(
                point=world_T_swung[:3, :3] @ point + world_T_swung[:3, 3],
                approach=world_T_swung[:3, :3] @ approach,
                closing=world_T_swung[:3, :3] @ closing,
                arm=self.arm,
            ),
        ]

    def swing_of_the_lever_to(self, position: float) -> np.ndarray:
        """
        :param position: A position of the lever's temperature joint.
        :return: The transform taking a point of the lever from where it lies now to
            where it lies once the joint stands at ``position``, in the world's root
            frame.
        """
        joint = self.lever.temperature_joint
        world_T_now = self.lever.root.global_pose.to_np()
        with joint._world.reset_state_context():
            joint.position = position
            world_T_swung = self.lever.root.global_pose.to_np()
        return world_T_swung @ np.linalg.inv(world_T_now)
