"""
The robot sets how hot a mixer tap runs by swinging the tap's lever with the side of one
fingertip of the open gripper.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from coraplex.robot_plans.actions.base import SequentialAction
from cramph.node import StatechartNode
from experiments.robocasa_atomic_tasks.geometry import BodyBounds
from experiments.robocasa_atomic_tasks.joints import FixtureJoints
from experiments.robocasa_atomic_tasks.robocasa_names import WaterTemperature
from experiments.robocasa_atomic_tasks.skills import (
    FingerSideApproach,
    FingerSidePressAction,
    FingerSideWithdrawAction,
    PushAction,
)
from krrood.entity_query_language.factories import a, variable
from krrood.entity_query_language.query.match import Match
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.semantic_annotations.semantic_annotations import MixerLever
from semantic_digital_twin.world_description.world_entity import Body

# %% swinging the lever


@dataclass(eq=False, repr=False, kw_only=True)
class AdjustWaterTemperatureAction(SequentialAction):
    """
    Swings the lever of a mixer tap on its temperature joint as far as it goes towards
    hot or cold water: the open gripper comes in level beside the lever, its fingers one
    above the other, and rests the side of its lower fingertip behind the lever's part
    farthest from that joint; the joint is swung to its end with the base held and the
    fingertip kept on the lever, and the gripper draws back from where the lever left it.

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

    finger_side_approach: FingerSideApproach
    """
    How the open gripper comes in beside the lever.
    """

    wrist: list[Body] = field(default_factory=list)
    """
    The bodies of the arm just behind its gripper, which may brush what stands around
    the lever.
    """

    surroundings: list[Body] = field(default_factory=list)
    """
    The bodies standing around the lever, such as the tap's spout and the wall behind
    it, which the wrist may touch.
    """

    threshold_share: float = 0.002
    """
    How close the temperature joint has to come to its end, as a share of its travel;
    tight, since the lever comes to rest a little short of where the push leaves it.
    """

    approach_distance: float = 0.1
    """
    How far before the lever, along the approach, the gripper starts its straight way in
    and ends its straight way back, in metres.
    """

    @classmethod
    def finger_approaches(
        cls,
        lever: MixerLever,
        temperature: WaterTemperature,
        surroundings: list[Body],
        hand_distance: float = 0.1,
    ) -> list[FingerSideApproach]:
        """
        :param lever: The lever swung.
        :param temperature: How hot the water is to run.
        :param surroundings: The bodies standing around the lever.
        :param hand_distance: How far behind the fingertip the hand is taken to rest
            when the room around it is compared, in metres.
        :return: The ways the gripper may come in beside the lever, from where its hand
            has the most room among ``surroundings`` first, see
            :meth:`FingerSideApproach.beside`.
        """
        joint = lever.temperature_joint
        end = temperature.temperature_joint_position(lever)
        point = FixtureJoints.point_for_a_fingertip(joint, lever.root, end)
        push = FixtureJoints.push_direction(joint, end, point=point)
        bounds = BodyBounds(lever._world)

        def room_for_the_hand(finger_side_approach: FingerSideApproach) -> float:
            hand = point - finger_side_approach.approach * hand_distance
            return min(
                (
                    bounds.distance_to_nearest_shape(body, hand)
                    for body in surroundings
                    if body.collision.shapes
                ),
                default=np.inf,
            )

        return sorted(
            FingerSideApproach.beside(push), key=room_for_the_hand, reverse=True
        )

    @classmethod
    def from_finger_approaches(
        cls,
        lever: MixerLever,
        temperature: WaterTemperature,
        arm: Arm,
        wrist: list[Body],
        surroundings: list[Body],
    ) -> Match:
        """
        :param lever: The lever swung.
        :param temperature: How hot the water is to run.
        :param arm: The arm whose fingertip swings the lever.
        :param wrist: See :attr:`wrist`.
        :param surroundings: See :attr:`surroundings`.
        :return: The action, with the way the gripper comes in left open among
            :meth:`finger_approaches`.
        """
        return a(cls)(
            lever=lever,
            temperature=temperature,
            arm=arm,
            finger_side_approach=variable(
                FingerSideApproach,
                domain=cls.finger_approaches(lever, temperature, surroundings),
            ),
            wrist=wrist,
            surroundings=surroundings,
        )

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper and :attr:`wrist` to touch
            :attr:`surroundings` and the lever.
        """
        gripper = [
            body
            for body in self.arm._world.get_kinematic_structure_entities_of_branch(
                self.arm.end_effector.root
            )
            if isinstance(body, Body) and body.collision.shapes
        ]
        return [
            AllowCollisionBetweenGroups(
                body_group_a=[*gripper, *self.wrist],
                body_group_b=[*self.surroundings, self.lever.root],
            )
        ]

    def create_steps(self) -> list[StatechartNode]:
        joint = self.lever.temperature_joint
        end = self.temperature.temperature_joint_position(self.lever)
        point = FixtureJoints.point_for_a_fingertip(joint, self.lever.root, end)
        approach = self.finger_side_approach.approach
        closing = self.finger_side_approach.closing
        push = self.finger_side_approach.push
        world_T_swung = self.swing_of_the_lever_to(end)
        swung = world_T_swung[:3, :3]
        return [
            PushAction(
                press=FingerSidePressAction(
                    point=point,
                    approach=approach,
                    closing=closing,
                    push=push,
                    arm=self.arm,
                    approach_distance=self.approach_distance,
                ),
                part=self.lever.root,
                joint=joint,
                goal_position=end,
                threshold=self.threshold_share * FixtureJoints.travel_of(joint),
                hold_base=True,
            ),
            FingerSideWithdrawAction(
                point=swung @ point + world_T_swung[:3, 3],
                approach=swung @ approach,
                closing=swung @ closing,
                push=swung @ push,
                arm=self.arm,
                distance=self.approach_distance,
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
