import numpy as np
import pytest

from coraplex.exceptions import ObjectIsNotHeld
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from cramph.composites import Sequence
from giskardpy.motion_statechart.goals.cartesian_goals import CartesianPoseStraight
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from coraplex.robot_plans.plan_transformations import MoveStraightAroundObjects
from giskardpy.motion_statechart.goals.gripper import MoveGripper
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types.spatial_types import Pose

from ..conftest import SAMPLING_SEED
from .conftest import expand
from ..plan_running import simulated_executor, statechart_of

# %% fixtures

HELD_AT = (0.03, -0.02, 0.05)
"""
Where the held object's origin sits relative to the tool frame, in meters; off-centre,
as a rim grasp would hold it.
"""

HELD_YAW = np.pi / 3
"""
How far the held object is turned about the tool frame's z-axis.
"""


@pytest.fixture
def pr2_holding_milk(simple_pr2_context):
    """
    A PR2 whose left tool frame holds the milk off-centre, at :data:`HELD_AT`, and the
    extensions it acts in.
    """
    world, robot, extensions = simple_pr2_context
    milk_body = world.get_body_by_name("milk.stl")
    milk = Milk(root=milk_body)
    tool_frame = robot.left_arm.end_effector.tool_frame
    with world.modify_world():
        world.move_branch(milk_body, tool_frame)
        world.add_semantic_annotation(milk)
    milk_body.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        *HELD_AT, yaw=HELD_YAW, reference_frame=tool_frame
    )
    world.notify_state_change()
    return world, robot, milk, extensions


@pytest.fixture
def pr2_gripping_milk_by_contact(simple_pr2_context):
    """
    A PR2 whose left fingers grip the milk where :data:`HELD_AT` puts it, while the
    milk stays attached to the world rather than to the tool frame, as a physically
    simulated grip leaves it, and the extensions it acts in.
    """
    world, robot, extensions = simple_pr2_context
    milk_body = world.get_body_by_name("milk.stl")
    milk = Milk(root=milk_body)
    tool_frame = robot.left_arm.end_effector.tool_frame
    with world.modify_world():
        world.add_semantic_annotation(milk)
    milk_body.parent_connection.origin = world.transform(
        HomogeneousTransformationMatrix.from_xyz_rpy(
            *HELD_AT, yaw=HELD_YAW, reference_frame=tool_frame
        ),
        milk_body.parent_connection.parent,
    )
    world.notify_state_change()
    return world, robot, milk, extensions


def _tool_goals_of(place: PlaceAction) -> list[CartesianPose]:
    """
    :return: The tool center point goals the expanded `place` runs, in order.
    """
    return [node for node in place.descendants if isinstance(node, CartesianPose)]


# %% releasing what is actually held


def test_a_place_puts_the_held_object_on_the_target_however_it_is_held(
    pr2_holding_milk,
):
    """
    Sending the tool frame to the target pose itself would place the object wherever
    the grasp happens to hold it, which is beside the target unless the object is held
    at its own origin. The release has to account for the transform the gripper
    actually holds it at.
    """
    world, robot, milk, extensions = pr2_holding_milk
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, yaw=np.pi / 4, reference_frame=world.root)
    place = PlaceAction(object_designator=milk, target_location=target)
    expand(Sequence(nodes=[place]), extensions)

    end_effector = robot.left_arm.end_effector
    tool_goal = end_effector.tool_frame_goal_for_grasp(
        place._grasp_on_the_held_object().moved_to(target)
    )

    tool_T_milk = world.transform(milk.root.global_transform, end_effector.tool_frame)
    placed_milk = tool_goal.homogeneous_matrix @ tool_T_milk

    np.testing.assert_allclose(
        placed_milk.to_np(), target.homogeneous_matrix.to_np(), atol=1e-9
    )


def test_a_place_runs_the_grasp_backwards(pr2_holding_milk):
    """
    A release comes down onto the target from where a pick-up would lift the object to,
    and leaves the way a pick-up would approach it.
    """
    world, robot, milk, extensions = pr2_holding_milk
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)
    place = PlaceAction(object_designator=milk, target_location=target)
    expand(Sequence(nodes=[place]), extensions)
    grasp = place._grasp_on_the_held_object()
    poses = place.grasp_pose_sequence(
        grasp.moved_to(target), robot.left_arm.end_effector, grasp
    )

    tool_goals = [goal.goal_pose for goal in _tool_goals_of(place)]

    # The grasp pose is commanded twice: to come down onto the target, and to stay there
    # while the gripper opens.
    assert len(tool_goals) == 4
    for expected, actual in zip(
        [poses.retreat, poses.grasp, poses.grasp, poses.pre_grasp], tool_goals
    ):
        np.testing.assert_allclose(expected.to_np(), actual.to_np(), atol=1e-9)


def test_a_place_that_rises_off_the_object_withdraws_straight_up(pr2_holding_milk):
    """
    Withdrawing the way the grasp came in draws the open fingers past a free-standing
    object at the height they held it, so a place may lift off it instead.
    """
    world, robot, milk, extensions = pr2_holding_milk
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)
    place = PlaceAction(
        object_designator=milk, target_location=target, rises_off_the_object=True
    )
    expand(Sequence(nodes=[place]), extensions)
    grasp = place._grasp_on_the_held_object()
    poses = place.grasp_pose_sequence(
        grasp.moved_to(target), robot.left_arm.end_effector, grasp
    )

    *_, withdrawal = _tool_goals_of(place)

    np.testing.assert_allclose(
        withdrawal.goal_pose.to_np(), poses.retreat.to_np(), atol=1e-9
    )


def test_a_place_moving_straight_around_objects_comes_down_and_withdraws_straight(
    pr2_holding_milk,
):
    """
    The object is between the fingers on the way down and still between them on the way
    out: a free path, or fingers left to drift, sweeps it off its place.
    """
    world, robot, milk, extensions = pr2_holding_milk
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)
    place = PlaceAction(object_designator=milk, target_location=target)
    executor = simulated_executor(
        extensions, plan_adaptations=[MoveStraightAroundObjects()]
    )
    statechart_of(executor, Sequence(nodes=[place]))

    _, descent, _, withdrawal = _tool_goals_of(place)

    assert (type(descent), type(withdrawal)) == (
        CartesianPoseStraight,
        CartesianPoseStraight,
    )
    assert [
        type(node) for node in withdrawal.nodes if isinstance(node, MoveGripper)
    ] == [MoveGripper]


def test_a_place_leaves_the_paths_around_the_object_free_by_default(pr2_holding_milk):
    world, robot, milk, extensions = pr2_holding_milk
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)
    place = PlaceAction(object_designator=milk, target_location=target)
    expand(Sequence(nodes=[place]), extensions)

    assert [type(goal) for goal in _tool_goals_of(place)] == [CartesianPose] * 4


# %% releasing what has not been picked up yet


def test_place_uses_the_grasp_its_pick_up_will_take(pr2_apartment_context):
    """
    A plan is built before it runs, so a place that follows a pick-up in the same plan
    is expanded while the object is still on its shelf, nowhere near the gripper. The
    grasp then has to come from the pick-up that is going to take it, not from where
    the object happens to lie.
    """
    world, robot, extensions = pr2_apartment_context
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)

    pick_up = PickUpAction(grasp=milk.grasp_candidates()[0], arm=robot.left_arm)
    place = PlaceAction(object_designator=milk, target_location=target)
    expand(Sequence(nodes=[pick_up, place]), extensions)

    np.testing.assert_allclose(
        place._grasp_on_the_held_object().grasp_pose.to_np(),
        pick_up.grasp.grasp_pose.to_np(),
        atol=1e-9,
    )


def test_a_place_of_an_object_nothing_holds_is_refused(pr2_apartment_context):
    """
    Nothing in the gripper and no pick-up before it leaves no arm to place with.
    """
    world, robot, extensions = pr2_apartment_context
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)

    place = PlaceAction(object_designator=milk, target_location=target)

    with pytest.raises(ObjectIsNotHeld):
        expand(Sequence(nodes=[place]), extensions)


# %% the arm that places


def _arms_moved_by(place: PlaceAction, robot) -> set[Arm]:
    """
    :return: The arms of `robot` whose tool frames the expanded `place` moves.
    """
    moved_tips = {goal.tip_link for goal in _tool_goals_of(place)}
    return {arm for arm in robot.all_arms if arm.end_effector.tool_frame in moved_tips}


def test_place_takes_the_arm_that_holds_the_object(pr2_holding_milk):
    world, robot, milk, extensions = pr2_holding_milk
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)
    place = PlaceAction(object_designator=milk, target_location=target)
    expand(Sequence(nodes=[place]), extensions)

    assert _arms_moved_by(place, robot) == {robot.left_arm}


def test_place_takes_the_arm_its_pick_up_will_use(pr2_apartment_context):
    """
    A plan is built before it runs, so a place that follows a pick-up in the same plan
    places with the arm that pick-up is going to hold the object in.
    """
    world, robot, extensions = pr2_apartment_context
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)
    pick_up = PickUpAction(grasp=milk.grasp_candidates()[0], arm=robot.right_arm)
    place = PlaceAction(object_designator=milk, target_location=target)
    expand(Sequence(nodes=[pick_up, place]), extensions)

    assert _arms_moved_by(place, robot) == {robot.right_arm}


# %% releasing what the fingers grip by contact alone


def test_place_takes_the_arm_whose_fingers_grip_the_object(
    pr2_gripping_milk_by_contact,
):
    world, robot, milk, extensions = pr2_gripping_milk_by_contact
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, reference_frame=world.root)
    place = PlaceAction(object_designator=milk, target_location=target)
    expand(Sequence(nodes=[place]), extensions)

    assert _arms_moved_by(place, robot) == {robot.left_arm}


def test_a_place_puts_an_object_gripped_by_contact_on_the_target(
    pr2_gripping_milk_by_contact,
):
    """
    The grasp is read off where the fingers hold the object, so the release accounts
    for it as it does for an object hanging off the tool frame.
    """
    world, robot, milk, extensions = pr2_gripping_milk_by_contact
    target = Pose.from_xyz_rpy(1.2, 0.4, 0.9, yaw=np.pi / 4, reference_frame=world.root)
    place = PlaceAction(object_designator=milk, target_location=target)
    expand(Sequence(nodes=[place]), extensions)

    end_effector = robot.left_arm.end_effector
    tool_goal = end_effector.tool_frame_goal_for_grasp(
        place._grasp_on_the_held_object().moved_to(target)
    )

    tool_T_milk = world.transform(milk.root.global_transform, end_effector.tool_frame)
    placed_milk = tool_goal.homogeneous_matrix @ tool_T_milk

    np.testing.assert_allclose(
        placed_milk.to_np(), target.homogeneous_matrix.to_np(), atol=1e-9
    )
