"""
What Tiago's RoboCasa tasks share where it moves a fixture by its handle: a drawer along
its rails, a door about its hinge.

The handle is held by the grip of the fingers alone and the fixture moves because the
hand draws it along. Nothing commands the fixture's own joint: a joint written into
carries the fixture with it whatever the hand is doing, which is the thing these
experiments are to show happening by contact.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.pick_up import GraspingAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from experiments.tiago_robocasa.robocasa_task import TiagoRoboCasaTask
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    PrismaticConnection,
    RevoluteConnection,
)
from semantic_digital_twin.world_description.world_entity import Body

# %% a task that moves a fixture by its handle


@dataclass
class HandleHold:
    """
    How the hand took hold of a handle and where it drew the fixture to, which letting
    go of the handle is worked out from.
    """

    grasp: GraspCandidate
    """
    The grasp the handle was taken by.
    """

    pre_grasp: Pose
    """
    Where the hand waited in front of the handle before it took hold.
    """

    drawn_to: float
    """
    Where the hand drew the mechanism's joint to.
    """


@dataclass
class TiagoHandleTask(TiagoRoboCasaTask, ABC):
    """
    One of RoboCasa's tasks in which Tiago takes hold of a fixture's handle and draws
    the fixture along by it.
    """

    handle_reach: float = 0.9
    """
    How far in front of its base's centre Tiago takes hold of the handle, in metres.

    Measured on a RoboCasa drawer, whose handle stands 0.79 m high: the hand gets onto
    it from 0.90 m to 0.95 m in front of the base and from nowhere nearer, because the
    arm hangs from a shoulder at the height of the counter and cannot fold back under
    itself. Nothing is in its way there -- the same holds with collision avoidance
    switched off.
    """

    base_follows_the_arm: bool = True
    """
    The base is driven along with the arm: the stretch of space the arm alone can hold
    the hand in is centimetres deep at a handle's height, while the fixture's travel is
    half a metre, so the hand can only stay on the handle if the base comes too.
    """

    waits_in_joint_space: bool = False
    """
    The hand is brought to where it waits in front of the handle by the joint positions
    solved for that pose, and a stand counts only if the arm reaches that pose too: a
    hand steered there directly may stall turning towards it.
    """

    tool_frame_tolerance: float = 0.03
    """
    How close the tool frame has to come to a goal, in metres: a simulated arm trails
    its commands while it moves, and a hand that is held to the millimetre never
    arrives.
    """

    finger_clearance: float = 0.015
    """
    How much room the open fingers are given to pass the handle, in metres, on top of
    how far they reach past the grasp.

    The hand waits that far in front of the handle and backs off that far again.
    """

    stall_minimum_time: float = 0.3
    """
    How long the fingers have to be stalled before the handle counts as held, in
    seconds.

    Closing on a bar stalls the fingers rather than shutting them.
    """

    closing_velocity: float = 0.05
    """
    How fast the fingers close on the handle, in metres per second.
    """

    pull_velocity: float = 0.05
    """
    How fast the hand draws the fixture along, in metres per second.

    Slower than the hand carries an object: the fingers only hold the handle by friction, and the
    fixture's own drag works against them the whole way.
    """

    pull_overshoot: float = 0.06
    """
    How far past where the fixture stops the hand is drawn, in metres: the fingers lose
    a little of the handle to slip the whole way along, and without it the fixture ends
    up short of where the task wants it.
    """

    handle_grasp_roll: float | None = field(init=False, default=None)
    """
    The roll of :attr:`handle_grasp_rolls` the arm reached the handle with from where
    Tiago stands, once a stand was found.
    """

    disturbance_tolerance: float = 0.05
    """
    How far another fixture's joint may move while Tiago works this one before the
    fixture counts as disturbed, in metres or radians.
    """

    other_fixtures_at_the_start: dict[ActiveConnection1DOF, float] | None = field(
        init=False, default=None
    )
    """
    Where the joint of every other fixture of the kitchen stood before Tiago set to
    work.
    """

    hold: HandleHold | None = field(init=False, default=None)
    """
    How the hand took hold of the handle and where it drew the fixture to, once it has.
    """

    retreat_distance: float = 0.15
    """
    How far the hand backs straight out from where it waited in front of the handle once
    it has let go, in metres, before the arms are parked: parking swings the hand
    sideways, and a fixture's neighbour, an oven's door bar beside a drawer, say, is
    within a hand's breadth of the handle.
    """

    door_waypoint_angle: float = math.radians(10)
    """
    How far a door turns between the waypoints the hand is led through, in radians:

    the handle travels on an arc about the hinge, and a hand led straight to where the
    door ends up would pull across the arc rather than along it.
    """

    impedance_ratio: float = 50.0
    """
    Stiffer friction than carrying an object needs: the fingers have to hold a bar
    against a drawer's drag rather than against an object's weight.
    """

    noslip_iterations: int = 30
    """
    More passes than carrying an object needs, for the same reason as
    :attr:`impedance_ratio`.
    """

    open_ratio: float = 0.95
    """
    The share of its travel from which on RoboCasa counts the fixture open.
    """

    closed_ratio: float = 0.05
    """
    The share of its travel up to which RoboCasa counts the fixture closed.
    """

    @property
    @abstractmethod
    def handle(self) -> Handle:
        """
        :return: The handle the fixture is moved by.
        """

    @property
    @abstractmethod
    def fixture(self) -> Body:
        """
        :return: The fixture the task works, which the moving part sits in.
        """

    @property
    @abstractmethod
    def goal_position(self) -> float:
        """
        :return: Where the mechanism's joint is to end up.
        """

    @property
    def pull_goal(self) -> float:
        """
        :return: Where the hand draws the mechanism's joint to, which is
            :attr:`pull_overshoot` past where the fixture stops.
        """
        goal = self.goal_position
        return goal + math.copysign(self.pull_overshoot, goal - self.mechanism.position)

    @property
    def pull_waypoints(self) -> list[float]:
        """
        :return: The positions of the mechanism's joint the hand is led through on its
            way to :attr:`pull_goal`: the goal alone for a drawer, whose handle travels
            in a straight line, and one every :attr:`door_waypoint_angle` for a door.
        """
        start, goal = self.mechanism.position, self.pull_goal
        if not isinstance(self.mechanism, RevoluteConnection):
            return [goal]
        count = max(1, math.ceil(abs(goal - start) / self.door_waypoint_angle))
        return [start + (goal - start) * step / count for step in range(1, count + 1)]

    # %% the fixture

    @property
    def mechanism(self) -> ActiveConnection1DOF:
        """
        :return: The joint the fixture moves on.
        """
        return self.handle.root.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )

    @property
    def moving_part(self) -> Body:
        """
        :return: The part of the fixture that moves on its joint, carrying the handle.
        """
        return self.mechanism.child

    @property
    def travel(self) -> float:
        """
        :return: How far along its joint the moving part stands, as a share of the
            travel its limits leave it.
        """
        limits = self.mechanism.dof.limits
        return float(
            (self.mechanism.position - limits.lower.position)
            / (limits.upper.position - limits.lower.position)
        )

    @property
    def handle_box(self) -> tuple[np.ndarray, np.ndarray]:
        """
        :return: The lowest and highest corner of the handle, in the kitchen's frame.
        """
        return self.box_around([self.handle.root])

    @property
    def handle_middle(self) -> np.ndarray:
        """
        :return: The middle of :attr:`handle_box`.
        """
        lowest, highest = self.handle_box
        return (lowest + highest) / 2

    @property
    def handle_is_upright(self) -> bool:
        """
        :return: Whether the handle stands taller than it is wide.
        """
        lowest, highest = self.handle_box
        size = highest - lowest
        return bool(size[2] > max(size[0], size[1]))

    @property
    def handle_grasp_rolls(self) -> tuple[float, ...]:
        """
        :return: The rolls of the hand about its approach the handle is tried with, in
            radians, the first that the arm reaches being taken: a quarter turn on a
            level handle, which closes the fingers across it from above and below;
            on an upright one no roll, three eighths of a turn and half a turn, which
            close them across it from the sides.

        Which roll the arm reaches an upright handle with depends on where the handle
        stands: measured at RoboCasa's sink lever, only the unrolled hand arrives;
        at its microwave, only the hand rolled three eighths of a turn does.
        """
        if self.handle_is_upright:
            return 0.0, 3 * math.pi / 4, math.pi
        return (math.pi / 2,)

    @property
    def first_target(self) -> np.ndarray:
        return self.handle_middle

    @property
    def handled_bodies(self) -> list[Body]:
        """
        :return: The whole fixture: not only the part that moves and the handle it
            carries, but the housing around them, which Tiago stands against while it
            works the mechanism.
        """
        return self.colliding_bodies_below(self.fixture)

    @property
    def bodies_under_the_handle(self) -> list[Body]:
        """
        :return: The bodies of the kitchen that span the handle's place on the floor
            and start below it: the counter a fixture on it stands on, and the cabinet
            under that. Their tops cannot be read off their boxes, which RoboCasa builds
            to take in fixtures far above.
        """
        handle = self.handle_middle
        robot_bodies = set(self.robot.bodies_with_collision)
        under = []
        for body in self.world.bodies:
            if not body.has_collision() or body in robot_bodies:
                continue
            lowest, highest = self.box_around([body])
            if (
                lowest[0] <= handle[0] <= highest[0]
                and lowest[1] <= handle[1] <= highest[1]
                and lowest[2] < handle[2]
            ):
                under.append(body)
        return under

    @property
    def headings_to_try(self) -> tuple[float, ...]:
        """
        :return: The one way that faces the fixture, read off which way its handle
            stands out from the part that moves.
        """
        outward = self.handle_middle - self.extent(self.moving_part).center.to_np()[:3]
        return (float(np.arctan2(-outward[1], -outward[0])),)

    @property
    def disturbed_fixtures(self) -> list[str]:
        """
        :return: The joints of the other fixtures that moved while Tiago worked this
            one, by name.
        """
        if self.other_fixtures_at_the_start is None:
            return []
        return [
            joint.name.name
            for joint, start in self.other_fixtures_at_the_start.items()
            if abs(joint.position - start) > self.disturbance_tolerance
        ]

    @property
    def outcome(self) -> str:
        """
        :return: How far along its joint the fixture now stands, and which other
            fixtures moved.
        """
        report = (
            f"{self.moving_part.name.name} stands at {self.travel:.0%} of its travel"
        )
        if self.disturbed_fixtures:
            report += f"; also moved: {', '.join(self.disturbed_fixtures)}"
        return report

    # %% where things stand once the fixture has moved

    def shift_to(self, position: float) -> np.ndarray:
        """
        :param position: Where the mechanism's joint is to be.
        :return: The transform, in the kitchen's frame, that takes what moves with the
            fixture from where it stands now to where it stands then.
        """
        joint = self.mechanism
        travel = position - joint.position
        movement = np.eye(4)
        match joint:
            case RevoluteConnection():
                movement[:3, :3] = Rotation.from_rotvec(
                    joint.axis.to_np()[:3] * travel
                ).as_matrix()
            case PrismaticConnection():
                movement[:3, 3] = joint.axis.to_np()[:3] * travel
        world_T_connection = joint.child.global_pose.to_np() @ np.linalg.inv(
            joint.connection_T_child_expression.to_np()
        )
        return world_T_connection @ movement @ np.linalg.inv(world_T_connection)

    def shifted(self, pose: Pose, position: float) -> Pose:
        """
        :param pose: A pose that moves with the fixture.
        :param position: Where the mechanism's joint is to be.
        :return: Where that pose stands then.
        """
        return HomogeneousTransformationMatrix(
            self.shift_to(position) @ pose.to_np(), reference_frame=self.world.root
        ).pose

    # %% taking hold of the handle

    @property
    def square_position(self) -> float:
        """
        :return: Where the mechanism stands when its handle faces Tiago squarely: shut,
            for a drawer or a door.
        """
        return float(self.mechanism.dof.limits.lower.position)

    def handle_grasp(self) -> GraspCandidate:
        """
        The grasp that takes the handle at its middle, approaching it from where Tiago
        stands, the fingers closing across the bar from above and below.

        It is worked out where the fixture stands at :attr:`square_position` and carried
        back to where the fixture stands now, so that a handle on an open door is taken
        at the angle the door has turned to.

        :return: The grasp.
        """
        handle = self.handle
        closed = self.square_position
        closed_middle = (self.shift_to(closed) @ np.append(self.handle_middle, 1.0))[:3]
        world_T_closed_grasp = HomogeneousTransformationMatrix.from_xyz_rpy(
            *closed_middle,
            roll=(
                self.handle_grasp_roll
                if self.handle_grasp_roll is not None
                else self.handle_grasp_rolls[0]
            ),
            pitch=self.handle_grasp_pitch,
            yaw=self.heading,
            reference_frame=self.world.root,
        ).pose
        return GraspCandidate(
            handle,
            self.world.transform(
                self.shifted(world_T_closed_grasp, self.mechanism.position),
                handle.root,
            ),
        )

    @property
    def handle_grasp_pitch(self) -> float:
        """
        :return: How far the hand is tilted from level as it takes the handle, in
            radians, negative upwards: not at all.
        """
        return 0.0

    def reaches_the_handle(self) -> bool:
        """
        :return: Whether the arm gets the hand onto the handle from where Tiago stands
            with any of :attr:`handle_grasp_rolls`, and to where it waits in front of it
            if it :attr:`waits_in_joint_space`, keeping the first roll it does, worked
            out afresh because the grasp follows Tiago as Tiago is moved about.
        """
        for roll in self.handle_grasp_rolls:
            self.handle_grasp_roll = roll
            if self.can_reach(self.handle_grasp().world_T_grasp) and (
                not self.waits_in_joint_space or self.can_reach(self.pre_grasp())
            ):
                return True
        self.handle_grasp_roll = None
        return False

    def taking_hold_of(self, grasp: GraspCandidate) -> GraspingAction:
        """
        :param grasp: The grasp the handle is taken by.
        :return: Reaching onto the handle and closing the fingers on it, which stall on
            the bar rather than shutting.
        """
        return GraspingAction(
            grasp,
            self.arm,
            approach_clearance=self.approach_clearance,
            grasp_closing_velocity=self.closing_velocity,
            grasp_stall_minimum_time=self.stall_minimum_time,
            tolerate_grasp_stall=True,
            position_threshold=self.tool_frame_tolerance,
        )

    @property
    def approach_clearance(self) -> float:
        """
        :return: How far in front of the handle the hand waits, in metres: far enough
            for the open fingers to pass the handle, and no further, because the arm's
            envelope at a handle's height is only centimetres deep.
        """
        return float(
            self.arm.end_effector.fingers_reach_beyond_the_grasp + self.finger_clearance
        )

    def pre_grasp(self) -> Pose:
        """
        :return: Where the hand waits before it moves onto the handle, read afresh so it
            follows Tiago as Tiago is moved about.
        """
        grasp = self.handle_grasp()
        return (
            self.taking_hold_of(grasp)
            .grasp_pose_sequence(grasp.world_T_grasp, self.arm.end_effector, grasp)
            .pre_grasp
        )

    # %% the plan

    def build_plan(self, context: Context) -> Plan:
        """
        Stand where the hand closes on the handle, bring the hand in front of it, take
        hold of it, draw the fixture to where the task wants it and let go.

        The stand has to leave the handle itself within reach of the arm, not merely the
        pose the hand waits at in front of it: those lie a hand's breadth apart and the
        arm's envelope at a handle's height is narrower than that, so a stand chosen for
        the waiting pose brings the arm there and stalls. Covering the rest, and the
        fixture's own travel, is what the base is along for.

        :param context: The context to plan in.
        :return: The plan.
        """
        self.stand_at_the_handle()
        return self.working_the_handle(context)

    def stand_at_the_handle(self) -> None:
        """
        Note where every other fixture stands and put Tiago where the hand closes on the
        handle.
        """
        robot_joints = set(self.robot.connections)
        self.other_fixtures_at_the_start = {
            joint: joint.position
            for joint in self.world.connections
            if isinstance(joint, ActiveConnection1DOF)
            and joint is not self.mechanism
            and joint not in robot_joints
        }
        self.stand_where_the_hand_arrives(
            self.reaches_the_handle, self.handle_middle, self.handle_reach
        )

    def backed_out_from(
        self, grasp: GraspCandidate, pre_grasp: Pose, position: float
    ) -> Pose:
        """
        :param grasp: The grasp the handle was taken by.
        :param pre_grasp: Where the hand waited in front of the handle.
        :param position: Where the mechanism stands once the hand has let go.
        :return: The waiting pose carried :attr:`retreat_distance` further out, along
            the line the hand came in on.
        """
        held = self.shifted(grasp.world_T_grasp, position).to_np()
        waiting = self.shifted(pre_grasp, position).to_np()
        outward = waiting[:3, 3] - held[:3, 3]
        waiting[:3, 3] += outward / np.linalg.norm(outward) * self.retreat_distance
        return HomogeneousTransformationMatrix(
            waiting, reference_frame=self.world.root
        ).pose

    def working_the_handle(self, context: Context) -> Plan:
        """
        Bring the hand in front of the handle from where Tiago stands, take hold of it,
        draw the fixture to where the task wants it and let go.

        :param context: The context to plan in.
        :return: The plan.
        """
        return sequential(
            self.drawing_the_fixture() + self.letting_go_of_the_handle(),
            context=context,
        ).plan

    def drawing_the_fixture(self) -> list[BaseMotion | ActionDescription]:
        """
        Note in :attr:`hold` how the hand is to take hold of the handle.

        :return: Bringing the hand in front of the handle from where Tiago stands,
            taking hold of it and drawing the fixture to where the task wants it.
        """
        self.hold = HandleHold(
            grasp=self.handle_grasp(),
            pre_grasp=self.pre_grasp(),
            drawn_to=self.pull_goal,
        )
        return [
            ParkArmsAction(self.robot.all_arms),
            *self.reaching_the_waiting_pose(),
            self.taking_hold_of(self.hold.grasp),
            *[
                MoveToolCenterPointMotion(
                    self.shifted(self.hold.grasp.world_T_grasp, waypoint),
                    self.arm,
                    allow_gripper_collision=True,
                    max_linear_velocity=self.pull_velocity,
                    position_threshold=self.tool_frame_tolerance,
                )
                for waypoint in self.pull_waypoints
            ],
        ]

    def reaching_the_waiting_pose(self) -> list[BaseMotion]:
        """
        :return: Bringing the hand to where :attr:`hold` has it wait in front of the
            handle, in joint space if it :attr:`waits_in_joint_space`.
        """
        if self.waits_in_joint_space:
            return self.arm_motions_to(self.hold.pre_grasp)
        return [
            MoveToolCenterPointMotion(
                self.hold.pre_grasp,
                self.arm,
                allow_gripper_collision=True,
                position_threshold=self.tool_frame_tolerance,
            )
        ]

    def letting_go_of_the_handle(self) -> list[BaseMotion | ActionDescription]:
        """
        :return: Opening the fingers on the handle :attr:`hold` took, backing the hand
            out to where it waited and :attr:`retreat_distance` further, and parking
            the arms.
        """
        hold = self.hold
        return [
            MoveGripperMotion(
                GripperState.OPEN,
                self.arm.end_effector,
                allow_gripper_collision=True,
            ),
            MoveToolCenterPointMotion(
                self.shifted(hold.pre_grasp, hold.drawn_to),
                self.arm,
                allow_gripper_collision=True,
                position_threshold=self.tool_frame_tolerance,
            ),
            MoveToolCenterPointMotion(
                self.backed_out_from(hold.grasp, hold.pre_grasp, hold.drawn_to),
                self.arm,
                allow_gripper_collision=True,
                position_threshold=self.tool_frame_tolerance,
            ),
            ParkArmsAction(self.robot.all_arms),
        ]
