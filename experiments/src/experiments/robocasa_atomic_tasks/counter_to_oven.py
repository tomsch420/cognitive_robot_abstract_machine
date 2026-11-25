"""
Moving an object lying on a plate on the counter onto the tray of the open oven with the
PandaOmron in a physical simulation: the gripper comes down onto the object from
straight above, closes across its narrow side, lifts it, carries it through the free
space to above the tray, lowers it until it touches the tray, lets go and withdraws.

Each step is a plan of its own, planned from the world as the step before left it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from coraplex.plans.executors import PlanOutcome
from coraplex.robot_plans.actions.core.robot_body import (
    MoveFingersToWidthAction,
    SetGripperAction,
)
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.geometry import (
    ApproachFrame,
    HorizontalFootprint,
)
from experiments.robocasa_atomic_tasks.sink_actions import BackOffAction
from experiments.robocasa_atomic_tasks.skills import (
    CarryThroughFreeSpaceAction,
    LowerTheToolUntilTouchingAction,
    RaiseTheToolAction,
    ReachThroughFreeSpaceAction,
)
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from krrood_spatial_math.spatial_types import Point3, Pose
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.world_entity import Body

# %% moving the object


@dataclass(kw_only=True)
class MoveFromTheCounterOntoTheOvenTray:
    """
    Moves an object lying on a plate on the counter onto the tray of the open oven.

    The defaults are those of the PandaOmron.
    """

    episode: RoboCasaEpisode
    """
    The episode the object is moved in, simulated physically.
    """

    graspable: IsGraspable
    """
    The object moved.
    """

    plate: Body
    """
    The plate the object lies on.
    """

    tray: Body
    """
    The oven's tray the object is put onto.
    """

    oven: Body
    """
    The body of the oven the tray belongs to.
    """

    hand_clearance_from_the_oven: float = 0.135
    """
    How far in front of the oven the object is carried to at least, so that the hand
    above it clears the oven, in metres.
    """

    reach_height: float = 0.1
    """
    How far above the top of the object the tool frame comes before it goes down onto
    the object, in metres.
    """

    finger_gap_above_the_plate: float = 0.005
    """
    How far above the plate the fingers close, in metres.
    """

    squeeze: float = 0.001
    """
    How far past the object's width each finger is sent as it closes, in metres.
    """

    counter_buffer_zone_distance: float = 0.002
    """
    How close to the object and the fixtures around it the robot may come while it takes
    hold of it, in metres.
    """

    lift_height: float = 0.1
    """
    How far the object is lifted straight up off the plate, in metres.
    """

    surroundings_buffer_zone_distance: float = 0.016
    """
    How close to everything else the robot may come while it reaches above the object,
    carries it and puts it down, in metres.
    """

    carry_clearance: float = 0.07
    """
    How far beyond half the height of the object the path it is carried along keeps its
    middle from everything else, in metres.
    """

    room_below_the_obstacles: float = 0.02
    """
    How far below every obstacle the path the object is carried along also stays out, in
    metres.
    """

    carry_height: float = 0.06
    """
    How far above the tray's top the underside of the object is carried to, in metres.
    """

    margin_inside_the_tray: float = 0.005
    """
    How far inside the tray's front edge the box of the object is carried to at least,
    in metres.
    """

    lowering_overshoot: float = 0.1
    """
    How far beyond :attr:`carry_height` the object is lowered at most before it touches
    the tray, for the tray's floor lying below its rim, in metres.
    """

    lift_off_height: float = 0.05
    """
    How far the open hand rises off the object once it lies on the tray, before the base
    backs off, in metres.
    """

    withdrawal_height: float = 0.2
    """
    How far the open hand rises once the base backed off, in metres.
    """

    back_off_distance: float = 0.2
    """
    How far the base backs off once the hand rose off the object, in metres.
    """

    def perform(self) -> None:
        """
        Take hold of the object on the plate, carry it onto the tray and withdraw.

        :raises PlanFailure: Why a step failed.
        """
        self.attempt().raise_if_failed()

    def attempt(self) -> PlanOutcome:
        """
        Take hold of the object on the plate, carry it onto the tray and withdraw, as
        far as the steps succeed.

        :return: How the last step run ended.
        """
        holding = self.take_hold_of_the_object()
        if not holding.succeeded:
            return holding
        return self.put_the_object_onto_the_tray()

    def take_hold_of_the_object(self) -> PlanOutcome:
        """
        Reach above the middle of the object, open the hand, lower it until it touches
        the object, the plate or a fixture around it, which the hand may touch
        meanwhile, raise it by :attr:`finger_gap_above_the_plate`, close the fingers
        across the narrow side of the object and lift it.

        :return: How the last step run ended; the steps after a failed one are not run.
        """
        arm = self.episode.arm
        gripper = arm.end_effector
        held = self.graspable.root
        footprint = HorizontalFootprint.from_body(held)
        hand = set(gripper.bodies_with_collision)
        reaching = self.episode.run(
            [
                ReachThroughFreeSpaceAction(
                    goal=self.pose_above_the_object(footprint),
                    arm=arm,
                    surroundings_buffer=self.surroundings_buffer_zone_distance,
                )
            ]
        )
        if not reaching.succeeded:
            return reaching
        around_the_object = self.episode.fixtures.bodies_near(
            self.episode.bounds.of_branch(held), margin=self.episode.handling_margin
        )
        with self.episode.allowances.rules_applied(
            ignore_rules=[
                AllowCollisionBetweenGroups(
                    body_group_a=list(gripper.bodies_with_collision),
                    body_group_b=[held, self.plate, *around_the_object],
                )
            ],
            default_rules=[
                AvoidCollisionBetweenGroups(
                    body_group_a=list(self.episode.robot.bodies_with_collision),
                    body_group_b=around_the_object,
                    buffer_zone_distance=self.counter_buffer_zone_distance,
                    violated_distance=0.0,
                ),
                AvoidCollisionBetweenGroups(
                    body_group_a=[
                        body
                        for body in self.episode.robot.bodies_with_collision
                        if body not in hand
                    ],
                    body_group_b=[held],
                    buffer_zone_distance=self.counter_buffer_zone_distance,
                    violated_distance=0.0,
                ),
            ],
        ):
            return self.episode.run_steps_in_turn(
                [
                    lambda: SetGripperAction(
                        gripper_configuration=gripper.default_configuration(
                            GripperState.OPEN
                        )
                    ),
                    lambda: LowerTheToolUntilTouchingAction(
                        arm=arm,
                        bodies=list(hand),
                        others=[held, self.plate, *around_the_object],
                        distance=2 * self.reach_height,
                        hold_base=True,
                    ),
                    lambda: RaiseTheToolAction(
                        arm=arm, height=self.finger_gap_above_the_plate, hold_base=True
                    ),
                    lambda: MoveFingersToWidthAction(
                        two_finger_gripper=gripper,
                        width=footprint.width,
                        squeeze=self.squeeze,
                    ),
                    lambda: RaiseTheToolAction(
                        arm=arm, height=self.lift_height, hold_base=True
                    ),
                ]
            )

    def put_the_object_onto_the_tray(self) -> PlanOutcome:
        """
        Turn the hand so that the object lies across the way out of the oven, carry it
        through the free space to above the tray, lower it until it touches the tray,
        let go, raise the hand off it, back the base off and raise the hand; the robot
        keeps :attr:`surroundings_buffer_zone_distance` from everything else but the
        object and the tray, which the hand may touch.

        :return: How the last step run ended; the steps after a failed one are not run.
        """
        arm = self.episode.arm
        gripper = arm.end_effector
        held = self.graspable.root
        robot = set(self.episode.robot.bodies)
        with self.episode.allowances.rules_applied(
            ignore_rules=[
                AllowCollisionBetweenGroups(
                    body_group_a=list(gripper.bodies_with_collision),
                    body_group_b=[held, self.tray],
                )
            ],
            default_rules=[
                AvoidCollisionBetweenGroups(
                    body_group_a=list(self.episode.robot.bodies_with_collision),
                    body_group_b=[
                        body
                        for body in self.episode.world.bodies_with_collision
                        if body not in robot
                    ],
                    buffer_zone_distance=self.surroundings_buffer_zone_distance,
                    violated_distance=0.0,
                )
            ],
        ):
            return self.episode.run_steps_in_turn(
                [
                    self.turn_laying_the_object_across,
                    lambda: CarryThroughFreeSpaceAction(
                        arm=arm,
                        carried=held,
                        goal=self.point_above_the_tray(),
                        clearance=self.carry_clearance,
                        room_below_obstacles=self.room_below_the_obstacles,
                    ),
                    lambda: LowerTheToolUntilTouchingAction(
                        arm=arm,
                        bodies=[held, *self.finger_bodies()],
                        others=[self.tray],
                        distance=self.carry_height + self.lowering_overshoot,
                        hold_base=True,
                    ),
                    lambda: SetGripperAction(
                        gripper_configuration=gripper.default_configuration(
                            GripperState.OPEN
                        )
                    ),
                    lambda: RaiseTheToolAction(
                        arm=arm, height=self.lift_off_height, hold_base=True
                    ),
                    lambda: BackOffAction(distance=self.back_off_distance),
                    lambda: RaiseTheToolAction(
                        arm=arm, height=self.withdrawal_height, hold_base=True
                    ),
                ]
            )

    def pose_above_the_object(self, footprint: HorizontalFootprint) -> Pose:
        """
        :param footprint: The footprint of the object.
        :return: The tool frame goal :attr:`reach_height` above the top of the object,
            pointing straight down with the fingers closing across its narrow side.
        """
        world = self.episode.world
        top = self.episode.bounds.of_body(self.graspable.root).upper[2]
        position = np.array(
            [footprint.middle[0], footprint.middle[1], top + self.reach_height]
        )
        world_T_grasp = ApproachFrame(
            position=position,
            approach=np.array([0.0, 0.0, -1.0]),
            closing=footprint.narrow_axis,
        ).in_reference_frame(world.root)
        return self.episode.arm.end_effector.tool_frame_goal_for_grasp(
            world_T_grasp.pose
        )

    def point_above_the_tray(self) -> Point3:
        """
        :return: Where the middle of the object is carried to: above the middle of the
            tray, moved horizontally out of the oven until it lies at least
            :attr:`hand_clearance_from_the_oven` in front of the box bounding the oven,
            but no further than keeps the object's box :attr:`margin_inside_the_tray`
            inside the tray's front edge, its underside
            :attr:`carry_height` above the tray's top.
        """
        bounds = self.episode.bounds
        tray = bounds.of_body(self.tray)
        oven = bounds.of_body(self.oven)
        held = bounds.of_body(self.graspable.root)
        middle = (tray.lower + tray.upper) / 2
        out_of_the_oven = self.direction_out_of_the_oven()
        oven_front = oven.extent_along(direction=out_of_the_oven).upper
        tray_front = tray.extent_along(direction=out_of_the_oven).upper
        held_extent = held.extent_along(direction=out_of_the_oven)
        held_reach = (held_extent.upper - held_extent.lower) / 2
        along = out_of_the_oven @ middle
        target = min(
            max(along, oven_front + self.hand_clearance_from_the_oven),
            tray_front - held_reach - self.margin_inside_the_tray,
        )
        point = middle + max(target - along, 0.0) * out_of_the_oven
        point[2] = (
            tray.upper[2] + self.carry_height + (held.upper[2] - held.lower[2]) / 2
        )
        return Point3(*point, reference_frame=self.episode.world.root)

    def direction_out_of_the_oven(self) -> np.ndarray:
        """
        :return: The horizontal unit vector from the oven's origin towards the middle of
            the tray, which is slid out of the oven.
        """
        tray = self.episode.bounds.of_body(self.tray)
        out_of_the_oven = (tray.lower + tray.upper) / 2 - self.oven.global_pose.to_np()[
            :3, 3
        ]
        out_of_the_oven[2] = 0.0
        return out_of_the_oven / np.linalg.norm(out_of_the_oven)

    def turn_laying_the_object_across(self) -> JointPositionList:
        """
        :return: A new goal turning the wrist's last joint, the hand pointing straight
            down, so that the long axis of the held object's footprint lies square to
            :meth:`direction_out_of_the_oven`: the shortest such turn within the joint's
            limits.
        """
        long_axis = HorizontalFootprint.from_body(self.graspable.root).long_axis
        across = np.cross([0.0, 0.0, 1.0], self.direction_out_of_the_oven())
        yaw = np.arctan2(np.cross(long_axis, across)[2], long_axis @ across)
        wrist = [
            connection
            for connection in self.episode.arm.active_connections
            if isinstance(connection, ActiveConnection1DOF)
        ][-1]
        axis = wrist.child.global_pose.to_np()[:3, :3] @ wrist.axis.to_np()[:3]
        limits = wrist.dof.limits
        targets = [
            wrist.position + (yaw + half_turn) / axis[2]
            for half_turn in (0.0, -np.pi, np.pi)
        ]
        reachable = [
            target
            for target in targets
            if limits.lower.position <= target <= limits.upper.position
        ]
        target = min(reachable, key=lambda target: abs(target - wrist.position))
        return JointPositionList(
            goal_state=JointState(connections=[wrist], target_values=[target])
        )

    def finger_bodies(self) -> list[Body]:
        """
        :return: The bodies of the gripper's fingers, without the palm they all share.
        """
        fingers = [
            set(finger.bodies_with_collision)
            for finger in self.episode.arm.end_effector.fingers
        ]
        return list(set.union(*fingers) - set.intersection(*fingers))
