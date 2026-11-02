from enum import StrEnum

import numpy as np
import pytest

from coraplex.robot_plans.actions.core.container import CloseAction, OpenAction
from coraplex.robot_plans.actions.core.pick_up import GraspingAction
from cramph.node import StatechartNode
from giskardpy.motion_statechart.goals.grasped_mechanism import (
    ArticulateGraspedMechanism,
)
from giskardpy.motion_statechart.goals.gripper import MoveGripper
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import (
    Pose,
    RotationMatrix,
    Vector3,
)
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    PrismaticConnection,
)
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from .conftest import left_or_only_arm
from ..plan_running import simulated_executor, statechart_of

# %% fixtures


class ApartmentBody(StrEnum):
    """
    The apartment's own bodies the tests act on.
    """

    DRAWER_HANDLE = "handle_cab10_m"
    """
    The handle of the apartment's middle cabinet drawer.
    """

    DRAWER = "cabinet10_drawer_middle"
    """
    The middle cabinet drawer the handle pulls out.
    """


@pytest.fixture
def pr2_at_drawer(pr2_apartment_context):
    """
    A PR2 in the apartment, with the drawer handle it works on annotated as such.
    """
    world, robot, extensions = pr2_apartment_context
    body = world.get_body_by_name(ApartmentBody.DRAWER_HANDLE)
    annotated = [
        handle
        for handle in world.get_semantic_annotations_by_type(Handle)
        if handle.root is body
    ]
    if not annotated:
        with world.modify_world():
            world.add_semantic_annotation_recursively(handle := Handle(root=body))
        annotated = [handle]
    return world, robot, extensions, annotated[0]


def goals_of(step: StatechartNode) -> list[StatechartNode]:
    """
    :param step: A step of an expanded action.
    :return: The step and every node below it.
    """
    return [step, *step.descendants]


# %% backing off the handle


@pytest.mark.parametrize("action_type", [OpenAction, CloseAction])
def test_gripper_backs_off_the_handle_after_letting_go(pr2_at_drawer, action_type):
    """
    The gripper still straddles the handle once it opens, so the action ends by
    clearing it along the direction it approached from.
    """
    world, robot, extensions, handle = pr2_at_drawer
    arm = left_or_only_arm(robot)
    grasp = GraspCandidate.from_body_origin(handle)
    action = action_type(grasp=grasp, arm=arm)
    statechart_of(simulated_executor(extensions), action)

    *_, letting_go, back_off = action.nodes

    assert [
        node.width for node in goals_of(letting_go) if isinstance(node, MoveGripper)
    ] == [arm.end_effector.separation_in(GripperState.OPEN)]
    [back_off_goal] = [
        node for node in goals_of(back_off) if isinstance(node, CartesianPose)
    ]
    np.testing.assert_allclose(
        back_off_goal.goal_pose.to_np(),
        action.back_off_pose(grasp.grasp_pose, arm.end_effector).to_np(),
        atol=1e-9,
    )


def test_back_off_pose_clears_the_grasp_by_the_release_clearance(pr2_at_drawer):
    """
    The gripper backs straight out along the way it came in, by the clearance it is
    given and no further.
    """
    world, robot, extensions, handle = pr2_at_drawer
    end_effector = left_or_only_arm(robot).end_effector
    action = OpenAction(
        grasp=GraspCandidate.from_body_origin(handle), arm=left_or_only_arm(robot)
    )
    grasp_pose = Pose(reference_frame=handle.root)

    tool_goal = end_effector.tool_frame_goal_for_grasp(grasp_pose)
    back_off = action.back_off_pose(grasp_pose, end_effector)

    np.testing.assert_allclose(
        back_off.to_np()[:3, :3], tool_goal.to_np()[:3, :3], atol=1e-9
    )
    np.testing.assert_allclose(
        back_off.to_np()[:3, 3] - tool_goal.to_np()[:3, 3],
        -action.release_clearance * np.asarray(grasp_pose.to_np()[:3, 0]),
        atol=1e-9,
    )


def test_back_off_pose_clears_the_drawer_of_a_grasp_from_above(pr2_at_drawer):
    """
    A grasp from above closes with one finger between the handle and the drawer front,
    so the gripper backs off until it is clear of the drawer, and by the release
    clearance beyond its top.
    """
    world, robot, extensions, handle = pr2_at_drawer
    arm = left_or_only_arm(robot)
    world_T_handle = world.transform(Pose(reference_frame=handle.root), world.root)
    downward = Vector3(0, 0, -1, reference_frame=world.root)
    pull_axis = Vector3(1, 0, 0, reference_frame=world.root)
    grasp_pose = world.transform(
        Pose(
            position=world_T_handle.position,
            orientation=RotationMatrix.from_vectors(x=downward, y=pull_axis).quaternion,
            reference_frame=world.root,
        ),
        handle.root,
    )
    action = OpenAction(grasp=GraspCandidate(handle, grasp_pose), arm=arm)

    tool_goal = world.transform(
        arm.end_effector.tool_frame_goal_for_grasp(grasp_pose), world.root
    )
    back_off = world.transform(
        action.back_off_pose(grasp_pose, arm.end_effector), world.root
    )
    drawer = world.get_body_by_name(ApartmentBody.DRAWER)
    drawer_top = (
        drawer.collision.as_bounding_box_collection_in_frame(world.root)
        .bounding_box()
        .z_interval.upper
    )
    grasp_height = world_T_handle.to_np()[2, 3]

    assert back_off.to_np()[2, 3] - tool_goal.to_np()[2, 3] == pytest.approx(
        drawer_top - grasp_height + action.release_clearance, abs=1e-3
    )


# %% a handle hanging from its joint through a body without geometry


@pytest.fixture
def pr2_at_a_handle_below_an_empty_body(simple_pr2_context):
    """
    A PR2 next to a handle whose joint moves a body without geometry, from which the
    handle hangs; it is how a part that turns about two joints in a row is built.
    """
    world, robot, extensions = simple_pr2_context
    with world.modify_world():
        empty = Body(name=PrefixedName("between_the_joints"))
        handle_body = Body(
            name=PrefixedName("handle_below_the_empty_body"),
            collision=ShapeCollection([Box(scale=Scale(0.04, 0.2, 0.04))]),
        )
        world.add_connection(
            PrismaticConnection.create_with_dofs(
                world,
                world.root,
                empty,
                axis=Vector3.X(),
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    1, 0, 1, reference_frame=world.root
                ),
            )
        )
        world.add_connection(FixedConnection(parent=empty, child=handle_body))
        world.add_semantic_annotation(handle := Handle(root=handle_body))
    return world, robot, handle


def test_back_off_pose_clears_a_handle_below_a_body_without_geometry(
    pr2_at_a_handle_below_an_empty_body,
):
    """
    The part the joint moves is everything that hangs from it, so the gripper backs
    out of the handle even though the body the joint moves directly has no geometry.
    """
    world, robot, handle = pr2_at_a_handle_below_an_empty_body
    end_effector = left_or_only_arm(robot).end_effector
    action = OpenAction(
        grasp=GraspCandidate.from_body_origin(handle), arm=left_or_only_arm(robot)
    )
    grasp_pose = Pose(reference_frame=handle.root)

    tool_goal = end_effector.tool_frame_goal_for_grasp(grasp_pose)
    back_off = action.back_off_pose(grasp_pose, end_effector)

    [handle_box] = handle.root.collision.shapes
    np.testing.assert_allclose(
        back_off.to_np()[:3, 3] - tool_goal.to_np()[:3, 3],
        -(handle_box.scale.x / 2 + action.release_clearance)
        * np.asarray(grasp_pose.to_np()[:3, 0]),
        atol=1e-9,
    )


# %% taking hold of the handle


@pytest.mark.parametrize("action_type", [OpenAction, CloseAction])
def test_the_tuning_the_action_is_given_reaches_taking_hold_of_the_handle(
    pr2_at_drawer, action_type
):
    """
    A container action is tuned the way taking hold of a grasp is tuned, and a hold
    that a physically simulated hand can keep needs that tuning passed on to it.
    """
    world, robot, extensions, handle = pr2_at_drawer
    action = action_type(
        grasp=GraspCandidate.from_body_origin(handle),
        arm=left_or_only_arm(robot),
        grasp_closing_velocity=0.05,
        grasp_stall_minimum_time=0.3,
        tolerate_grasp_stall=True,
        position_threshold=0.03,
    )
    statechart_of(simulated_executor(extensions), action)

    [taking_hold] = [node for node in action.nodes if isinstance(node, GraspingAction)]
    assert (
        taking_hold.grasp_closing_velocity,
        taking_hold.grasp_stall_minimum_time,
        taking_hold.tolerate_grasp_stall,
        taking_hold.position_threshold,
    ) == (
        action.grasp_closing_velocity,
        action.grasp_stall_minimum_time,
        action.tolerate_grasp_stall,
        action.position_threshold,
    )


# %% how far closed


def test_a_close_action_shuts_the_container_as_far_as_it_is_asked_to(pr2_at_drawer):
    """
    A container that has to shut tighter than the default is closed to the position
    and within the tolerance the action is given.
    """
    world, robot, extensions, handle = pr2_at_drawer
    action = CloseAction(
        grasp=GraspCandidate.from_body_origin(handle),
        arm=left_or_only_arm(robot),
        closed_joint_position=0.0,
        closing_tolerance=0.002,
    )
    statechart_of(simulated_executor(extensions), action)

    [closing] = [
        node
        for step in action.nodes
        for node in goals_of(step)
        if isinstance(node, ArticulateGraspedMechanism)
    ]
    assert closing.threshold == action.closing_tolerance
