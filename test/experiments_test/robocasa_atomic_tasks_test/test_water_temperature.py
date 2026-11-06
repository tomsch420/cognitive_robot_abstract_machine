"""
The robot setting how hot a mixer tap runs by swinging its lever with one fingertip.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.robocasa_atomic_tasks.joints import FixtureJoints
from experiments.robocasa_atomic_tasks.panda_omron import PandaOmron
from experiments.robocasa_atomic_tasks.robocasa_names import WaterTemperature
from experiments.robocasa_atomic_tasks.skills import (
    FingerSideApproach,
    FingerSidePressAction,
    FingerSideWithdrawAction,
    PushAction,
    StandAction,
)
from experiments.robocasa_atomic_tasks.water_temperature import (
    AdjustWaterTemperatureAction,
    StandAndAdjustWaterTemperatureAction,
)
from giskardpy.motion_statechart.goals.gripper import MovesGripperToConfiguration
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
    Vector3,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import MixerLever
from semantic_digital_twin.world import World
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    RevoluteConnection,
)
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
    LimitSide,
)
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% a mixer tap


@dataclass
class MixerTapWithAPost:
    """
    A mixer tap whose lever is a thin upright post, swivelling about the world's x-axis
    and tilting about its y-axis at the post's foot, as RoboCasa's sink handle does.
    """

    lever: MixerLever
    """
    The lever, the hot water at the upper end of the swivel's range.
    """

    pivot: np.ndarray
    """
    Where the swivel and the hinge cross, at the post's foot, in the world's root frame.
    """

    post_scale: Scale
    """
    The post's extents.
    """

    post_center: np.ndarray
    """
    Where the post's middle lies, in the world's root frame.
    """

    obstacle: Body
    """
    A block standing 5 cm to the left of the post, as a faucet's spout does.
    """


@pytest.fixture
def mixer_tap_with_a_post() -> MixerTapWithAPost:
    world = World()
    pivot = np.array([1.0, 0.0, 0.95])
    post_scale = Scale(0.012, 0.012, 0.076)
    post_height = 0.06
    with world.modify_world():
        tap = Body(name=PrefixedName("tap"))
        world.add_kinematic_structure_entity(tap)
        swivel_body = Body(name=PrefixedName("swivel"))
        lever_body = Body(
            name=PrefixedName("lever"),
            collision=ShapeCollection(
                [
                    Box(
                        origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                            z=post_height
                        ),
                        scale=post_scale,
                    )
                ]
            ),
        )
        world.add_connection(
            RevoluteConnection.create_with_dofs(
                world=world,
                parent=tap,
                child=swivel_body,
                axis=Vector3.X(),
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    *pivot, reference_frame=tap
                ),
                dof_limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                    lower_position=-0.785, upper_position=0.785, maximum_speed=1.0
                ),
            )
        )
        world.add_connection(
            RevoluteConnection.create_with_dofs(
                world=world,
                parent=swivel_body,
                child=lever_body,
                axis=Vector3.Y(),
                dof_limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                    lower_position=0.0, upper_position=0.5, maximum_speed=1.0
                ),
            )
        )
        lever = MixerLever(root=lever_body, hot_side=LimitSide.UPPER)
        world.add_semantic_annotation(lever)
        obstacle = Body(
            name=PrefixedName("obstacle"),
            collision=ShapeCollection([Box(scale=Scale(0.03, 0.3, 0.3))]),
        )
        world.add_connection(
            FixedConnection(
                parent=tap,
                child=obstacle,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    *(pivot + np.array([-0.05, -0.15, 0.1])), reference_frame=tap
                ),
            )
        )
    return MixerTapWithAPost(
        lever=lever,
        pivot=pivot,
        post_scale=post_scale,
        post_center=pivot + np.array([0.0, 0.0, post_height]),
        obstacle=obstacle,
    )


def adjustment(
    tap: MixerTapWithAPost, temperature: WaterTemperature, robot: PandaOmron
) -> AdjustWaterTemperatureAction:
    """
    :return: The action setting the tap to ``temperature``, coming in along the first
        of its finger approaches.
    """
    [arm] = robot.all_arms
    [finger_side_approach, *_] = AdjustWaterTemperatureAction.finger_approaches(
        tap.lever, temperature, [tap.obstacle]
    )
    return AdjustWaterTemperatureAction(
        lever=tap.lever,
        temperature=temperature,
        arm=arm,
        finger_side_approach=finger_side_approach,
        wrist=[arm.end_effector.root],
        surroundings=[tap.obstacle],
    )


# %% where the lever is swung to


def test_hot_water_runs_at_the_hottest_position_of_the_lever(mixer_tap_with_a_post):
    lever = mixer_tap_with_a_post.lever

    assert (
        WaterTemperature.HOT.temperature_joint_position(lever) == lever.hottest_position
    )


def test_cold_water_runs_at_the_coldest_position_of_the_lever(mixer_tap_with_a_post):
    lever = mixer_tap_with_a_post.lever

    assert (
        WaterTemperature.COLD.temperature_joint_position(lever)
        == lever.coldest_position
    )


# %% how the hand comes in


def test_the_hand_comes_in_level_and_square_to_the_push():
    push = np.array([0.0, -1.0, 0.1]) / np.linalg.norm([0.0, -1.0, 0.1])

    for finger_side_approach in FingerSideApproach.beside(push):
        assert finger_side_approach.approach[2] == pytest.approx(0.0)
        assert finger_side_approach.approach @ push == pytest.approx(0.0)


def test_the_fingers_close_upwards_square_to_the_approach_and_the_push():
    push = np.array([0.0, -1.0, 0.1]) / np.linalg.norm([0.0, -1.0, 0.1])

    for finger_side_approach in FingerSideApproach.beside(push):
        closing = finger_side_approach.closing
        assert closing[2] > 0.0
        assert closing @ finger_side_approach.approach == pytest.approx(0.0)
        assert closing @ push == pytest.approx(0.0)


def test_the_side_with_room_for_the_hand_is_tried_first(mixer_tap_with_a_post):
    tap = mixer_tap_with_a_post
    first, second = AdjustWaterTemperatureAction.finger_approaches(
        tap.lever, WaterTemperature.HOT, [tap.obstacle]
    )

    obstacle_side = np.sign(tap.obstacle.global_pose.to_np()[0, 3] - tap.post_center[0])
    assert np.sign(first.approach[0]) == obstacle_side
    assert np.sign(second.approach[0]) == -obstacle_side


# %% how the fingertip pushes the lever


def test_the_lower_fingertip_rests_its_side_on_the_point(panda_omron):
    [arm] = panda_omron.all_arms
    point = np.array([1.0, 0.0, 1.0])
    press = FingerSidePressAction(
        point=point,
        approach=np.array([-1.0, 0.0, 0.0]),
        closing=np.array([0.0, 0.0, 1.0]),
        push=np.array([0.0, -1.0, 0.0]),
        arm=arm,
    )
    world = arm._world
    world_T_tool = world.transform(
        press.fingertips_goal(0.0), world.root
    ).homogeneous_matrix.to_np()
    tool_P_edge = press.pushing_edge_of_the_lower_fingertip().to_np()

    np.testing.assert_allclose((world_T_tool @ tool_P_edge)[:3], point, atol=1e-9)
    corners = [
        world_T_tool[:3, :3] @ corner + world_T_tool[:3, 3]
        for box in press.open_fingertip_boxes()
        for corner in (box.lower, box.upper)
    ]
    lowest = min(corner[2] for corner in corners)
    assert lowest == pytest.approx(point[2])


def test_the_push_swings_the_temperature_joint_and_holds_the_base(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    [push, _] = adjustment(tap, WaterTemperature.COLD, panda_omron).create_steps()

    assert isinstance(push, PushAction)
    assert isinstance(push.press, FingerSidePressAction)
    assert push.joint is tap.lever.temperature_joint
    assert push.part is tap.lever.root
    assert push.goal_position == tap.lever.coldest_position
    assert push.hold_base


def test_the_fingertip_side_rests_on_the_post_where_it_is_pushed_from(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    [push, _] = adjustment(tap, WaterTemperature.HOT, panda_omron).create_steps()

    behind = -np.sign(push.press.push[1])
    np.testing.assert_allclose(
        push.press.point,
        tap.post_center + [0.0, behind * tap.post_scale.y / 2.0, 0.0],
        # the push direction is probed by turning the swivel a milliradian
        atol=1e-5,
    )


def test_the_wrist_may_touch_what_stands_around_the_lever(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    action = adjustment(tap, WaterTemperature.HOT, panda_omron)

    [rule] = action.create_temporary_collision_rules()
    assert isinstance(rule, AllowCollisionBetweenGroups)
    assert set(action.wrist) <= set(rule.body_group_a)
    assert rule.body_group_b == [*action.surroundings, tap.lever.root]


def test_the_fingers_may_touch_the_lever_and_what_stands_around_it(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    action = adjustment(tap, WaterTemperature.HOT, panda_omron)

    [rule] = action.create_temporary_collision_rules()
    fingertips = {finger.tip for finger in action.arm.end_effector.fingers}
    assert fingertips <= set(rule.body_group_a)


def test_the_gripper_withdraws_from_where_the_push_leaves_the_post(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    action = adjustment(tap, WaterTemperature.HOT, panda_omron)
    [push, withdraw] = action.create_steps()

    swing = Rotation.from_rotvec([tap.lever.hottest_position, 0.0, 0.0]).as_matrix()
    assert isinstance(withdraw, FingerSideWithdrawAction)
    np.testing.assert_allclose(
        withdraw.point, tap.pivot + swing @ (push.press.point - tap.pivot), atol=1e-9
    )
    np.testing.assert_allclose(withdraw.push, swing @ push.press.push, atol=1e-9)


def test_the_gripper_withdraws_with_the_base_held(mixer_tap_with_a_post, panda_omron):
    [_, withdraw] = adjustment(
        mixer_tap_with_a_post, WaterTemperature.HOT, panda_omron
    ).create_steps()

    assert withdraw.hold_base


# %% walking to a stand first


def stand_and_adjustment(
    tap: MixerTapWithAPost, robot: PandaOmron, through_free_space: bool
) -> StandAndAdjustWaterTemperatureAction:
    """
    :return: The action walking to a stand in front of the tap and setting it to hot.
    """
    return StandAndAdjustWaterTemperatureAction(
        navigate=StandAction(target_location=Pose(reference_frame=robot._world.root)),
        adjustment=adjustment(tap, WaterTemperature.HOT, robot),
        through_free_space=through_free_space,
    )


def test_the_robot_stands_and_opens_the_gripper_before_it_swings_the_lever(
    mixer_tap_with_a_post, panda_omron
):
    action = stand_and_adjustment(mixer_tap_with_a_post, panda_omron, False)

    [navigate, gripper, adjustment_step] = action.create_steps()

    assert navigate is action.navigate
    assert isinstance(gripper, MovesGripperToConfiguration)
    assert adjustment_step is action.adjustment


def test_the_gripper_may_be_led_through_free_space_to_where_the_press_begins(
    mixer_tap_with_a_post, panda_omron
):
    action = stand_and_adjustment(mixer_tap_with_a_post, panda_omron, True)

    [navigate, gripper, reach, adjustment_step] = action.create_steps()

    assert adjustment_step is action.adjustment
