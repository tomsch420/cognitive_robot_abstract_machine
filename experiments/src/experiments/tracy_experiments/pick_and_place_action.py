"""
:class:`PickUpActionMujoco`/:class:`PlaceActionMujoco`: Mujoco-driven siblings of
:class:`~coraplex.robot_plans.actions.core.pick_up.PickUpAction`/
:class:`~coraplex.robot_plans.actions.core.placing.PlaceAction`, matching their own
field interface (``object_designator``, ``arm``, ``grasp_description``/
``target_location``) so a caller can compose them into a
:func:`~coraplex.plans.factories.sequential` plan the same way, but with each own leaf
motion running plain Python (see :meth:`PickUpActionMujoco._run`/
:meth:`PlaceActionMujoco._run`, wrapped via :func:`~coraplex.plans.factories.code`)
rather than a Giskard motion mapping.

The real ``PickUpAction``/``PlaceAction`` build their own plan entirely from
``MoveToolCenterPointMotion``/``MoveGripperMotion`` designators, each of which ticks
Giskard's own closed loop live against the world model
(:class:`~coraplex.robot_plans.motions.gripper.MoveToolCenterPointMotion` →
:class:`~coraplex.plans.plan_node.MotionNode` →
:class:`~coraplex.plans.executables.GiskardExecutable`). That races
:class:`~semantic_digital_twin.adapters.multi_sim.MujocoSynchronizer`'s own
physics-thread state sync for a physically simulated robot -- see
:mod:`~experiments.tracy_experiments.equipment`'s own module docstring. These two
actions instead reuse :mod:`~experiments.tracy_experiments.trajectory_planning`'s own
plan-then-execute functions directly: each reach is planned by Giskard against an
isolated scratch copy of the world, then the resulting trajectory is played back by
commanding the real MuJoCo actuators.

Unlike ``PickUpAction``/``PlaceAction``, neither action here kinematically attaches or
detaches the object (no ``AttachNode``/``DetachNode``): the object is held only by real
MuJoCo contact friction between the fingers throughout -- a kinematically snapped object
is not left behind by a friction hold's own continuous motion the way an instantaneous
kinematic detach would otherwise risk. Both actions are generic over any body and arm,
used the same way for a Montessori shape being sorted into a hole and a cube being
stacked onto another.

Both actions currently support only a fixed top-down grasp (``grasp_description`` is
accepted for interface parity with ``PickUpAction``, but its own approach direction and
vertical alignment are not yet read); see :func:`_finger_midpoint_offset`'s own
docstring for the geometry this fixed orientation assumes.
"""

from __future__ import annotations

import math

import numpy
from typing_extensions import Dict

from coraplex.datastructures.enums import Arms
from coraplex.datastructures.grasp import GraspDescription
from coraplex.plans.factories import code
from coraplex.plans.plan_node import PlanNode
from coraplex.robot_plans.actions.base import ActionDescription
from dataclasses import dataclass
from experiments.tracy_experiments.real_time_simulation import RealTimeSimulation
from experiments.tracy_experiments.trajectory_planning import (
    close_gripper_around,
    follow_joint_trajectory,
    plan_cartesian_trajectory,
    set_gripper,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types.spatial_types import Point3, Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Actuator, Body

HOVER_CLEARANCE = 0.3
"""
Height, in metres, above a body's own top face the TCP moves to before descending onto
it -- clears obstacles during the horizontal part of each approach.

``0.3`` clears the Montessori board plus its three drawers with a wide margin --
confirmed directly, a smaller hover height left Giskard's own collision-avoidance
solver too little vertical room to route the arm above the board at all.
"""

PLACE_HOVER_CLEARANCE = 0.05
"""
Height, in metres, above ``target_location`` a body is released at, rather than
descending onto it exactly.
"""

CORRECTED_PLACE_DESCENT_CLEARANCE = 0.015
"""
Height, in metres, above ``target_location`` the held piece descends to for its own
closed-loop-corrected release (see :func:`_correct_place_xy`), once XY is already known
to be accurate -- lower than :data:`PLACE_HOVER_CLEARANCE` since there is little risk
left of drifting into the hole's own rim on the way down, and less height to fall gives
it less chance to catch on that rim instead of dropping straight through.
"""

MAX_PLACE_CORRECTIONS = 3
"""
How many times :func:`_correct_place_xy` re-measures the held piece's own position and
reaches again before giving up and releasing anyway.

A single correction pass took a real 2.7cm miss down to 2.5mm (confirmed directly) --
close, but still enough to land the piece on the hole's own board surface rather than
drop through it, since the opening leaves only a few millimetres of true clearance
around a piece this size. The piece is still held at this point, so nothing stops
measuring and correcting again; bounded rather than looped forever since each pass
costs a full reach and there is no guarantee of converging below whatever the true
clearance turns out to require.
"""

PLACE_XY_CONVERGED = 0.001
"""
Metres of remaining XY error :func:`_correct_place_xy` accepts as close enough to stop
correcting early, rather than spending its full :data:`MAX_PLACE_CORRECTIONS` budget
regardless.
"""

GRASP_CLOSE_SWING_CLEARANCE = 0.015
"""
Extra height, in metres, added on top of the object's own vertical centre when reaching
the grasp pose, so the fingertip pads still clear the resting surface after closing.

The pads aren't fixed relative to the tool frame: as the Robotiq-85 knuckle closes, each
pad's own position travels ~1.35cm further out along the gripper's reach axis (measured
directly: pad centre sits at 0.1208m from the gripper mount when open, 0.1343m when
closed) -- confirmed directly, a pad that clears the table by that same ~1.35cm while
open ends up flush with the table once closed. ``0.015`` covers that swing with a small
margin.

Was ``0.0435`` -- nearly 3x this docstring's own derived value -- confirmed directly as
a real bug, not an intentional deviation: for a piece a few centimetres tall (e.g. a
3cm cube, so 1.5cm half-height), reaching 4.35cm above its own centre put the grasp
target *above the piece's own top face entirely*, closing the fingers on whatever sliver
of the piece's upper edge they happened to catch rather than pinching it securely around
its middle. That grip held the piece rock-steady the instant it stopped closing --
gravity alone does not need much from a marginal top-edge pinch -- but let it slip free
as soon as the very next reach put it under any acceleration at all, regardless of how
much squeeze force, gripper servo torque, or contact stiffness the fingers were given
(all already tried; see :data:`~experiments.tracy_experiments.equipment.
GRIPPER_JOINT_SERVO`'s own docstring and :data:`SQUEEZE_MARGIN`'s), because none of
those make a bad pinch point good.
"""

def _bounding_box_center_world(world: World, body: Body) -> numpy.ndarray:
    """
    A body's own collision bounding box centre, in the world root frame.

    :param world: The world ``body`` belongs to.
    :param body: The body to measure.
    """
    bounding_box = body.collision[0].local_frame_bounding_box
    center_local = numpy.array(
        [
            (bounding_box.min_x + bounding_box.max_x) / 2,
            (bounding_box.min_y + bounding_box.max_y) / 2,
            (bounding_box.min_z + bounding_box.max_z) / 2,
        ]
    )
    root_transform_body = world.compute_forward_kinematics_np(world.root, body)
    return root_transform_body[:3, :3] @ center_local + root_transform_body[:3, 3]


def _finger_midpoint_offset(robot: Tracy, arm_side: Arms) -> numpy.ndarray:
    """
    Fixed offset from an arm's own tool frame to its gripper's own finger-tip midpoint,
    expressed in the tool frame's own local axes.

    :func:`~experiments.tracy_experiments.trajectory_planning.plan_cartesian_trajectory`
    places the tool frame itself at a Cartesian goal, not where the fingers actually
    meet -- confirmed directly, targeting a shape's own centre this way put the tool
    frame there but left the fingers several centimetres away, closing on open air.
    Each fingertip's own collision bounding box centre (not its link origin) stands in
    for where that finger actually is, since the link origin sits at one edge of the
    fingertip mesh, not its geometric centre.

    :param robot: The robot whose gripper this offset is measured on.
    :param arm_side: Which arm's gripper to measure.
    """
    arm = robot.right_arm if arm_side == Arms.RIGHT else robot.left_arm
    prefix = "right_" if arm_side == Arms.RIGHT else "left_"
    tool_frame = arm.end_effector.tool_frame
    root_transform_tool = robot._world.compute_forward_kinematics_np(
        robot._world.root, tool_frame
    )
    left_center = _bounding_box_center_world(
        robot._world,
        robot._world.get_body_by_name(f"{prefix}robotiq_85_left_finger_tip_link"),
    )
    right_center = _bounding_box_center_world(
        robot._world,
        robot._world.get_body_by_name(f"{prefix}robotiq_85_right_finger_tip_link"),
    )
    finger_midpoint = (left_center + right_center) / 2
    offset_in_root_frame = finger_midpoint - root_transform_tool[:3, 3]
    return root_transform_tool[:3, :3].T @ offset_in_root_frame


def _top_down_pose_builder(world: World, robot: Tracy, arm: Arms):
    """
    Build a ``pose(x, y, z) -> Pose`` closure that places the gripper's own finger
    midpoint (not its tool frame) at the given world-frame point, fixed top-down.

    :param world: The world the returned poses are expressed in.
    :param robot: The robot whose gripper geometry corrects the target.
    :param arm: Which arm's gripper geometry to use.
    """
    orientation = Pose.from_xyz_rpy(0, 0, 0, pitch=math.pi, reference_frame=world.root)
    tool_frame_rotation = orientation.to_rotation_matrix().evaluate()[:3, :3]
    finger_midpoint_offset = _finger_midpoint_offset(robot, arm)

    def pose(x: float, y: float, z: float) -> Pose:
        finger_target = numpy.array([x, y, z])
        tool_frame_target = finger_target - tool_frame_rotation @ finger_midpoint_offset
        return Pose.from_xyz_rpy(
            *tool_frame_target, pitch=math.pi, reference_frame=world.root
        )

    return pose


REACH_SETTLE_TIMEOUT = 10.0
"""
Simulated seconds :func:`_reach` waits, in a single attempt, for the servo to settle on
a leg's own final waypoint -- see :func:`~experiments.tracy_experiments.
trajectory_planning.follow_joint_trajectory`'s own ``settle_timeout``, whose default
this matches exactly (kept as its own name here rather than just omitting the argument,
so the choice -- one attempt, no retry -- reads as deliberate at the call site).

Was a bounded 3-attempt *retry* (a fresh Giskard re-plan from wherever the first attempt
left off) rather than a single wait. Confirmed directly, with instrumentation tracking
the held piece's own position tick by tick through a whole reach: the piece survived
being carried the entire length of a reach's own planned trajectory just fine, then
separated abruptly right where the first attempt's settle window ran out and a retry's
fresh re-plan began -- a second, shorter re-plan trajectory is necessarily jerkier (same
tick budget, less distance left to cover), and that jerk is what shook a piece loose
that the original, smoother trajectory was carrying without incident. Removing the
retry (holding the same single planned trajectory's own final waypoint instead) let a
real episode's grasp survive an entire pick-up for the first time, and land within a
couple of centimetres of the hole (confirmed directly: 0.0mm/27mm/17mm off in x/y/z,
0.25 containment against the 0.90 needed to count as sorted).

Tried lengthening this same single wait to 30s next, on the theory that more patience
alone could close that last gap -- confirmed directly it does not: a real episode with
the longer wait landed back at the piece's own spawn position (0.0 containment), worse
than the 10s version, not better. This is a chaotic system (see :data:`~experiments.
tracy_experiments.equipment._LARGE_JOINT_TORQUE_HEADROOM`'s own docstring on why first
attempts are short to begin with); changing how long the arm dwells anywhere changes
every subsequent tick's own physics state, not just precision at that one leg, so this
value should be treated as load-bearing and re-validated against a real episode (not
just assumed safe to raise) before it is changed again.
"""


def _reach(
    world: World,
    sim: RealTimeSimulation,
    actuators: Dict[str, Actuator],
    arm: Arms,
    goal_pose: Pose,
) -> None:
    """
    Plan a Cartesian reach against an isolated scratch copy of ``world`` and play it
    back on the real, physically simulated ``sim``, holding the final waypoint for up to
    :data:`REACH_SETTLE_TIMEOUT` (rather than retrying with a fresh re-plan -- see its
    own docstring for why a retry specifically breaks a reach that is carrying
    something).

    Collision avoidance is off: a much more crowded scene than an open table (e.g. the
    Montessori board plus its three drawers) can make Giskard's own collision-avoidance
    solver repeatedly raise ``CollisionViolatedError`` even after widening clearances --
    confirmed directly, still colliding with the board, a drawer, and even the target
    shape itself. Orientation is still constrained (``translation_only=False``): every
    pose here shares the same fixed top-down orientation, and leaving it unconstrained
    let Giskard's own IK redundancy resolution drift the achieved orientation slightly
    on each leg.

    :param world: The live world to clone for planning; never itself modified.
    :param sim: The running real-time simulation to drive.
    :param actuators: Every joint's own actuator, keyed by joint name.
    :param arm: Which arm's tool centre point should reach ``goal_pose``.
    :param goal_pose: Target pose for the gripper's own finger midpoint.
    """
    trajectory = plan_cartesian_trajectory(
        world, arm, goal_pose, translation_only=False, avoid_collisions=False
    )
    follow_joint_trajectory(
        sim, actuators, trajectory, settle_timeout=REACH_SETTLE_TIMEOUT
    )


def _correct_place_xy(
    world: World,
    sim: RealTimeSimulation,
    actuators: Dict[str, Actuator],
    arm: Arms,
    robot: Tracy,
    held_body: Body,
    target_position: Point3,
    descent_clearance: float,
) -> None:
    """
    Reach for ``target_position`` again, as many times as :data:`MAX_PLACE_CORRECTIONS`
    allows, each time measuring where the held piece actually ended up and correcting
    for exactly that much XY error -- see :data:`MAX_PLACE_CORRECTIONS`'s own docstring
    for why one pass was not always enough.

    Descends to ``descent_clearance`` above ``target_position`` (not
    :data:`PLACE_HOVER_CLEARANCE`) throughout: confirmed directly, once XY is close
    there is little risk left of swinging into the hole's own rim on the way down, and
    a shorter fall gives the piece less chance to catch on that rim instead of dropping
    straight through it.

    :param world: The live world to clone for planning; never itself modified.
    :param sim: The running real-time simulation to drive.
    :param actuators: Every joint's own actuator, keyed by joint name.
    :param arm: Which arm is holding and placing the piece.
    :param robot: The robot ``arm`` belongs to.
    :param held_body: The body currently held, whose own position is measured.
    :param target_position: Where ``held_body`` should end up.
    :param descent_clearance: Height, in metres, above ``target_position`` each
        correction reaches to.
    """
    pose = _top_down_pose_builder(world, robot, arm)
    for _ in range(MAX_PLACE_CORRECTIONS):
        actual_center = _bounding_box_center_world(world, held_body)
        xy_error = (
            float(target_position.x) - float(actual_center[0]),
            float(target_position.y) - float(actual_center[1]),
        )
        if max(abs(xy_error[0]), abs(xy_error[1])) < PLACE_XY_CONVERGED:
            return
        corrected_pose = pose(
            float(target_position.x) + xy_error[0],
            float(target_position.y) + xy_error[1],
            float(target_position.z) + descent_clearance,
        )
        _reach(world, sim, actuators, arm, corrected_pose)


@dataclass
class PickUpActionMujoco(ActionDescription):
    """
    :class:`~coraplex.robot_plans.actions.core.pick_up.PickUpAction`'s own field
    interface, but driven by direct MuJoCo actuator control; see this module's own
    docstring.
    """

    object_designator: Body
    """
    The body to pick up.
    """

    arm: Arms
    """
    Which arm picks it up.
    """

    grasp_description: GraspDescription
    """
    Accepted for interface parity with ``PickUpAction``; not yet read (see this module's
    own docstring) -- every grasp is currently a fixed top-down approach.
    """

    sim: RealTimeSimulation
    """
    The running real-time simulation to drive.
    """

    actuators: Dict[str, Actuator]
    """
    Every joint's own actuator, keyed by joint name.
    """

    hover_clearance: float = HOVER_CLEARANCE
    """
    See :data:`HOVER_CLEARANCE`.
    """

    @property
    def _action_plan(self) -> PlanNode:
        return code(self._run)

    def _run(self) -> None:
        world = self.world
        robot = self.robot
        pose = _top_down_pose_builder(world, robot, self.arm)

        body_center = _bounding_box_center_world(world, self.object_designator)
        pick_hover = pose(
            body_center[0], body_center[1], body_center[2] + self.hover_clearance
        )
        pick_grasp = pose(
            body_center[0],
            body_center[1],
            body_center[2] + GRASP_CLOSE_SWING_CLEARANCE,
        )

        _reach(world, self.sim, self.actuators, self.arm, pick_hover)
        _reach(world, self.sim, self.actuators, self.arm, pick_grasp)
        close_gripper_around(
            self.sim, self.actuators, robot, self.arm, self.object_designator
        )
        # Recomputed, not reused: the knuckle has just closed, and the finger-pad
        # midpoint's own offset from the tool frame -- baked into pose() -- is measured
        # off whatever the gripper's own geometry was at the moment pose() was built.
        # Reusing the pre-grasp (open-gripper) pick_hover here would retreat to where
        # the *open* gripper's own midpoint should be, not the closed one now actually
        # holding the piece -- confirmed directly, ~1.35cm off along the closing axis
        # (see _finger_midpoint_offset's own docstring for that measurement).
        closed_pose = _top_down_pose_builder(world, robot, self.arm)
        pick_hover_closed = closed_pose(
            body_center[0], body_center[1], body_center[2] + self.hover_clearance
        )
        _reach(world, self.sim, self.actuators, self.arm, pick_hover_closed)


@dataclass
class PlaceActionMujoco(ActionDescription):
    """
    :class:`~coraplex.robot_plans.actions.core.placing.PlaceAction`'s own field
    interface, but driven by direct MuJoCo actuator control; see this module's own
    docstring.
    """

    object_designator: Body
    """
    The body to place; only used to release it, since this action does not kinematically
    attach it in the first place (see this module's own docstring).
    """

    target_location: Pose
    """
    Where to place :attr:`object_designator`.
    """

    arm: Arms
    """
    Which arm places it.
    """

    sim: RealTimeSimulation
    """
    The running real-time simulation to drive.
    """

    actuators: Dict[str, Actuator]
    """
    Every joint's own actuator, keyed by joint name.
    """

    hover_clearance: float = HOVER_CLEARANCE
    """
    See :data:`HOVER_CLEARANCE`.
    """

    place_hover_clearance: float = PLACE_HOVER_CLEARANCE
    """
    See :data:`PLACE_HOVER_CLEARANCE`.
    """

    @property
    def _action_plan(self) -> PlanNode:
        return code(self._run)

    def _run(self) -> None:
        world = self.world
        robot = self.robot
        pose = _top_down_pose_builder(world, robot, self.arm)

        target_position = self.target_location.to_position()
        place_hover = pose(
            float(target_position.x),
            float(target_position.y),
            float(target_position.z) + self.hover_clearance,
        )
        place_pose = pose(
            float(target_position.x),
            float(target_position.y),
            float(target_position.z) + self.place_hover_clearance,
        )

        _reach(world, self.sim, self.actuators, self.arm, place_hover)
        _reach(world, self.sim, self.actuators, self.arm, place_pose)
        # Closed-loop correction: the reach above is open-loop -- it plans once,
        # against a kinematic snapshot, and trusts the result -- so whatever residual
        # joint error is still there when it gives up its own settle window (a few
        # hundredths of a radian, confirmed directly) becomes a few centimetres of
        # Cartesian error in the held piece's own actual position, not just the tool
        # frame's. The piece is still held at this point, so its own actual position
        # can be measured and corrected for as many times as needed -- see
        # :func:`_correct_place_xy`'s own docstring for why one pass alone was not
        # enough (confirmed directly: a first pass took a 2.7cm miss down to 2.5mm).
        #
        # Tried correcting at a safe hover height first, well clear of the board,
        # then descending once and correcting again -- confirmed directly this is
        # worse, not better: it adds reach legs (each one a jerk event on a piece
        # held only by friction -- see :data:`~experiments.montessori.
        # world.LOOSE_PIECE_MASS`, 30g), and a real episode's own tracked
        # position/quaternion showed the piece losing the gripper's grip entirely
        # partway through that longer sequence (every reach after some point left
        # both the piece's position and orientation completely unchanged, meaning
        # the arm kept moving but nothing was attached to it any more) -- worse than
        # this shorter sequence's own known failure mode (landing on the board,
        # still held, tipped).
        _correct_place_xy(world, self.sim, self.actuators, self.arm, robot,
                           self.object_designator, target_position,
                           CORRECTED_PLACE_DESCENT_CLEARANCE)
        set_gripper(self.sim, self.actuators, robot, self.arm, GripperState.OPEN)
        # Recomputed, not reused: pose() was built while still holding the piece
        # (closed-gripper geometry), but the knuckle has just opened for this retreat --
        # see PickUpActionMujoco._run's own comment on the same recompute, other
        # direction.
        open_pose = _top_down_pose_builder(world, robot, self.arm)
        place_hover_open = open_pose(
            float(target_position.x),
            float(target_position.y),
            float(target_position.z) + self.hover_clearance,
        )
        _reach(world, self.sim, self.actuators, self.arm, place_hover_open)
