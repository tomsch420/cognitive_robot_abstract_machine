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
    FingerApproach,
    FingerWithdrawAction,
    PushAction,
)
from experiments.robocasa_atomic_tasks.water_temperature import (
    AdjustWaterTemperatureAction,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import MixerLever
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import RevoluteConnection
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
    return MixerTapWithAPost(
        lever=lever,
        pivot=pivot,
        post_scale=post_scale,
        post_center=pivot + np.array([0.0, 0.0, post_height]),
    )


def adjustment(
    tap: MixerTapWithAPost, temperature: WaterTemperature, robot: PandaOmron
) -> AdjustWaterTemperatureAction:
    """
    :return: The action setting the tap to ``temperature``, coming in along the first
        of its finger approaches.
    """
    [arm] = robot.all_arms
    [finger_approach, *_] = AdjustWaterTemperatureAction.finger_approaches(
        tap.lever, temperature
    )
    return AdjustWaterTemperatureAction(
        lever=tap.lever,
        temperature=temperature,
        arm=arm,
        finger_approach=finger_approach,
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


# %% how the fingertip pushes the lever


def test_the_fingertip_rests_on_the_post_where_it_is_pushed_from(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    [push, _] = adjustment(tap, WaterTemperature.HOT, panda_omron).create_steps()

    swinging_towards = np.sign(push.press.closing[1])
    np.testing.assert_allclose(
        push.press.point,
        tap.post_center - [0.0, swinging_towards * tap.post_scale.y / 2.0, 0.0],
        # the push direction is probed by turning the swivel a milliradian
        atol=1e-5,
    )


def test_the_fingers_close_along_the_way_the_post_is_pushed(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    point = FixtureJoints.point_for_a_fingertip(
        tap.lever.temperature_joint, tap.lever.root, tap.lever.hottest_position
    )
    push = FixtureJoints.push_direction(
        tap.lever.temperature_joint, tap.lever.hottest_position, point=point
    )

    for finger_approach in AdjustWaterTemperatureAction.finger_approaches(
        tap.lever, WaterTemperature.HOT
    ):
        np.testing.assert_allclose(finger_approach.closing, push, atol=1e-12)


def test_the_push_swings_the_temperature_joint_with_the_fingertip_on_the_lever(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    action = adjustment(tap, WaterTemperature.COLD, panda_omron)
    [push, _] = action.create_steps()

    assert isinstance(push, PushAction)
    assert push.joint is tap.lever.temperature_joint
    assert push.part is tap.lever.root
    assert push.goal_position == tap.lever.coldest_position


def test_the_gripper_withdraws_from_where_the_push_leaves_the_post(
    mixer_tap_with_a_post, panda_omron
):
    tap = mixer_tap_with_a_post
    action = adjustment(tap, WaterTemperature.HOT, panda_omron)
    [push, withdraw] = action.create_steps()

    swing = Rotation.from_rotvec([tap.lever.hottest_position, 0.0, 0.0]).as_matrix()
    assert isinstance(withdraw, FingerWithdrawAction)
    np.testing.assert_allclose(
        withdraw.point, tap.pivot + swing @ (push.press.point - tap.pivot), atol=1e-9
    )
    np.testing.assert_allclose(
        withdraw.approach, swing @ push.press.approach, atol=1e-9
    )
