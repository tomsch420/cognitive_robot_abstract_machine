"""
Solve a Cartesian reach for a GraspCandidate.approach_pose via Giskard, in-process.

This copies the pattern from coraplex's real pickup machinery
(coraplex/src/coraplex/plans/executables.py, GiskardExecutable._execute_simulation)
instead of driving Giskard through GiskardWrapperNode's ROS action-client/world-fetch
round trip: build a Ros2Executor directly against our own live semdt World and tick it
to completion. Client and "server" are the same Python process and the same World
object, so there is nothing to serialize or synchronize over ROS.

Collision handling was inspired by cram2 PR #589 ("Fix collision avoidance in motion
charts and reachability validation"), not a blunt on/off toggle. An earlier version of
this file solved the reach in two phases -- collision avoidance ON to a pre-grasp pose,
then OFF for a short final descent -- because turning ExternalCollisionAvoidance off
entirely for the final approach was the only tool available at the time. That still let
the *fully unprotected* final phase clip the cube sideways with the open fingers
(verified: slip 0.22, one fewer contact, non-deterministically depending on aperture).
PR #589 adds AllowCollisionForEndEffector for exactly this (exempt specific bodies from
the collision matrix while collision avoidance keeps checking everything else), but that
PR is unmerged -- not present on this branch. Its actual mechanism, AllowCollisionRule
subclasses that edit the collision matrix, already exists here as
AllowCollisionForBodies (semantic_digital_twin.collision_checking.collision_rules), so
this uses that directly: a single collision-avoiding solve for the whole reach, with the
two finger bodies exempted -- everything else about the arm stays protected, only the
fingers are free to approach and straddle the cube. See solve_reach_trajectory's
`end_effector_bodies` parameter.

(The one thing PR #589's EndEffector-specific version adds beyond this -- re-reading the
exempted bodies on every world update so an object grasped mid-plan is automatically
included -- doesn't matter here: this is a single one-shot reach solve, not a multi-step
plan that picks something up partway through.)
"""
import os
import sys

sys.path.insert(0, "experiments/src")

from typing import Dict, List, Optional

from giskardpy.middleware.ros2 import rospy
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
    UpdateTemporaryCollisionRules,
)
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy.ros_executor import Ros2Executor

from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForBodies,
)
from semantic_digital_twin.robots.minimal_robot import MinimalRobot
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

ARM_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]

_registered_worlds = set()


def _ensure_robot_registered(world: World, arm_root: Body) -> None:
    """
    Giskard's QP controller needs an AbstractRobot semantic annotation to know which
    connections are actuated; register the minimal one once per world.
    """
    if id(world) in _registered_worlds:
        return
    MinimalRobot.from_branch_in_world(branch_root=arm_root)
    _registered_worlds.add(id(world))


def _read_arm_joints(world: World) -> Dict[str, float]:
    return {
        name: world.state[world.get_connection_by_name(name).dof.id].position
        for name in ARM_JOINT_NAMES
    }


def solve_reach_trajectory(
    world: World,
    tip: Body,
    goal_pose: Pose,
    root: Body = None,
    timeout: int = 2000,
    collision_avoidance: bool = True,
    end_effector_bodies: Optional[List[Body]] = None,
) -> List[Dict[str, float]]:
    """
    Solve a Cartesian IK reach of ``tip`` to ``goal_pose`` directly against ``world``'s
    own state (mutates world.state in place to the solved configuration).

    :param collision_avoidance: Whether to add ``ExternalCollisionAvoidance()`` to the
        motion statechart (mirroring coraplex's ``GiskardExecutable.collision_avoidance``
        flag -- same node, added the same way ``GiskardExecutable.prepare_for_execution``
        does when that flag is set).
    :param end_effector_bodies: If given (together with ``collision_avoidance=True``),
        exempts exactly these bodies from the collision matrix for this solve, via
        ``UpdateTemporaryCollisionRules([AllowCollisionForBodies(...)])`` -- inspired by
        cram2 PR #589's AllowCollisionForEndEffector, mirroring coraplex's
        ``allow_gripper_collision`` flag on
        ``MoveToolCenterPointMotion``/``MoveGripperMotion``. This is what actually lets a
        single collision-avoiding solve reach all the way in and straddle an object:
        everything else stays protected, only these bodies (e.g. the two gripper
        fingers) are allowed to approach/touch the goal object.

    Unlike a single final-pose IK solve, this returns *every* intermediate joint
    configuration Giskard passes through on the way to the goal, one per control tick.
    That matters here: collision avoidance only protects Giskard's own solve -- if a
    caller discards the intermediate ticks and only replays the final configuration
    (e.g. by jumping a separate physics engine's PD controller straight from a hover
    pose to that final target), the *replay* path is a straight joint-space interpolation
    that was never collision-checked, and can still knock over anything in between. Feed
    every returned waypoint into the physics executor instead of just the last one.

    :return: One ``{joint_name: position}`` dict per control tick, in order.
    """
    root = root or world.get_body_by_name("link0")
    _ensure_robot_registered(world, root)
    if rospy.node is None:
        # PID-suffixed: parallel dataset-generation workers (see generate_dataset_worker.py)
        # are separate OS processes sharing one ROS2 domain -- a fixed node name here
        # would collide across them.
        rospy.init_node(f"graspability_reach_{os.getpid()}")

    executor = Ros2Executor(
        context=MotionStatechartContext(
            world=world,
            qp_controller_config=QPControllerConfig(target_frequency=50),
        ),
        ros_node=rospy.node,
    )
    motion_statechart = MotionStatechart()
    if collision_avoidance:
        motion_statechart.add_node(ExternalCollisionAvoidance())
        if end_effector_bodies:
            motion_statechart.add_node(
                UpdateTemporaryCollisionRules(
                    temporary_rules=[
                        AllowCollisionForBodies(
                            allowed_collision_bodies=set(end_effector_bodies)
                        )
                    ]
                )
            )
    motion_statechart.add_node(
        cartesian_goal := CartesianPose(root_link=root, tip_link=tip, goal_pose=goal_pose)
    )
    motion_statechart.add_node(EndMotion.when_true(cartesian_goal))
    executor.compile(motion_statechart)

    trajectory = []
    for _ in range(timeout):
        executor.tick()
        trajectory.append(_read_arm_joints(world))
        if executor.motion_statechart.is_end_motion():
            break
    else:
        raise TimeoutError("Timeout reached while waiting for end of motion.")

    return trajectory


def solve_reach(
    world: World, tip: Body, goal_pose: Pose, root: Body = None, timeout: int = 2000
) -> dict:
    """
    Same as :func:`solve_reach_trajectory`, but returns only the final joint
    configuration. Kept for callers that only need the destination, not the path there
    (e.g. anything that doesn't drive a separate physics engine through the reach).
    """
    return solve_reach_trajectory(world, tip, goal_pose, root=root, timeout=timeout)[-1]
