"""
Putting an object into a fixture by taking it from above and carrying it there with the
whole body, the arm above the object kept clear of what hangs over the way.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from coraplex.robot_plans.actions.base import SequentialAction
from coraplex.robot_plans.actions.core.robot_body import SetGripperAction
from coraplex.robot_plans.mixins import HasApproachesGraspPoses, OperatesGripper
from cramph.composites import Deferred
from cramph.node import StatechartNode
from experiments.robocasa_atomic_tasks.geometry import BodyBounds
from experiments.robocasa_atomic_tasks.sink_actions import BackOffAction
from experiments.robocasa_atomic_tasks.skills import (
    CarryThroughFreeSpaceAction,
    LowerTheToolUntilTouchingAction,
    RaiseTheToolAction,
    ReachThroughFreeSpaceAction,
)
from krrood_spatial_math.spatial_types import Point3
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForEndEffector,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.world_description.world_entity import Body

# %% putting an object into a fixture


@dataclass(eq=False, repr=False, kw_only=True)
class PutIntoAFixtureFromAboveAction(
    HasApproachesGraspPoses, OperatesGripper, SequentialAction
):
    """
    Takes an object from above until the palm touches it, carries it with the whole body
    to above a spot inside a fixture, sets it down there with the arm alone until it
    touches the fixture, lets go of it and backs away.

    .. note:: Keep this action even though no task's solution uses it: it solves
        RoboCasa's ``PickPlaceCounterToSink`` with the PandaOmron in physics.
    """

    arm: Arm
    """
    The arm whose gripper takes the object.
    """

    grasp: GraspCandidate
    """
    A grasp coming down onto the object from above.
    """

    spot: Point3
    """
    Where the middle of the object's underside is to come to lie.
    """

    fixture_bodies: list[Body]
    """
    The bodies of the fixture, which the object is set down on and the gripper may
    touch.
    """

    approach_clearance: float = 0.02
    """
    Differs from :attr:`HasApproachesGraspPoses.approach_clearance` in leaving the
    gripper just above the object, from where the palm is lowered onto it.
    """

    lift_height: float = 0.1
    """
    How far the object is lifted straight up once the gripper closed on it, in metres.
    """

    hover_height: float = 0.08
    """
    How far above its spot the carry with the whole body leaves the object, in metres.
    """

    room_for_the_arm: float = 0.3
    """
    How far below every obstacle the carry with the whole body keeps the object out, in
    metres: the room the arm above the object needs.
    """

    lowering_distance: float = 0.25
    """
    How far the tool frame goes down at most while the palm comes onto the object or the
    object onto the fixture, in metres.
    """

    retreat_height: float = 0.25
    """
    How far the tool frame rises once it let go of the object, in metres.
    """

    back_off_distance: float = 0.3
    """
    How far the base backs off at the end, in metres.
    """

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to touch the object it takes and the
            fixture it puts the object into.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
        gripper = self.arm.end_effector
        held = self.grasp.graspable.root
        palm = list(
            set.intersection(
                *(set(finger.bodies_with_collision) for finger in gripper.fingers)
            )
        )
        return [
            SetGripperAction(
                configuration=gripper.default_configuration(GripperState.OPEN)
            ),
            Deferred(
                name="reach above the object",
                task=ReachThroughFreeSpaceAction(
                    goal=self.grasp_pose_sequence(
                        self.grasp.grasp_pose, gripper, self.grasp
                    ).pre_grasp,
                    arm=self.arm,
                ),
            ),
            Deferred(
                name="bring the palm onto the object",
                task=LowerTheToolUntilTouchingAction(
                    arm=self.arm,
                    bodies=palm,
                    others=[held],
                    distance=self.lowering_distance,
                ),
            ),
            self.gripper_goal_type.moving_gripper_to(
                gripper.default_configuration(GripperState.CLOSE), tolerate_stall=True
            ),
            Deferred(
                name="lift the object",
                task=RaiseTheToolAction(arm=self.arm, height=self.lift_height),
            ),
            Deferred(
                name="carry the object above its spot",
                task=CarryThroughFreeSpaceAction(
                    arm=self.arm,
                    carried=held,
                    goal=self.middle_of_the_object_above_the_spot(self.hover_height),
                    room_below_obstacles=self.room_for_the_arm,
                ),
            ),
            Deferred(
                name="bring the object onto its spot",
                task=CarryThroughFreeSpaceAction(
                    arm=self.arm,
                    carried=held,
                    goal=self.middle_of_the_object_above_the_spot(0.0),
                    hold_base=True,
                ),
            ),
            Deferred(
                name="set the object down",
                task=LowerTheToolUntilTouchingAction(
                    arm=self.arm,
                    bodies=[held],
                    others=self.fixture_bodies,
                    distance=self.lowering_distance,
                    hold_base=True,
                ),
            ),
            SetGripperAction(
                configuration=gripper.default_configuration(GripperState.OPEN)
            ),
            Deferred(
                name="rise off the object",
                task=RaiseTheToolAction(
                    arm=self.arm, height=self.retreat_height, hold_base=True
                ),
            ),
            Deferred(
                name="back off",
                task=BackOffAction(distance=self.back_off_distance),
            ),
        ]

    def middle_of_the_object_above_the_spot(self, height: float) -> Point3:
        """
        :param height: How far above the spot the object's underside is to be, in
            metres.
        :return: Where the middle of the box bounding the object then lies.
        """
        bounds = BodyBounds(world=self.world).of_body(self.grasp.graspable.root)
        half_height = (bounds.upper[2] - bounds.lower[2]) / 2
        spot = self.world.transform(self.spot, self.world.root).to_np()[:3]
        return Point3(
            *(spot + np.array([0.0, 0.0, half_height + height])),
            reference_frame=self.world.root,
        )
