"""
The positions of RoboCasa's joints as RoboCasa states them.
"""

from __future__ import annotations

import numpy as np
import pytest

from experiments.robocasa_atomic_tasks.joints import FixtureJoints, JointRange
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Vector3,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import RevoluteConnection
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
)
from semantic_digital_twin.world_description.world_entity import Body

# %% RoboCasa's ranges


def test_a_fraction_of_a_range_lies_that_far_from_its_lower_limit():
    joint_range = JointRange(lower=-1.0, upper=3.0)

    assert joint_range.at_fraction(0.25) == 0.0


def test_a_door_opening_towards_positive_positions_is_shut_at_its_lower_limit():
    joint_range = JointRange(lower=0.0, upper=2.0)

    assert joint_range.at_share(0.0) == joint_range.lower
    assert joint_range.at_share(1.0) == joint_range.upper


def test_a_door_opening_towards_negative_positions_is_shut_at_its_upper_limit():
    joint_range = JointRange(lower=-2.0, upper=0.0)

    assert joint_range.at_share(0.0) == joint_range.upper
    assert joint_range.at_share(1.0) == joint_range.lower


def test_a_range_ending_at_zero_is_held_mirrored():
    assert JointRange(lower=-2.0, upper=0.0).mirrored
    assert not JointRange(lower=0.0, upper=2.0).mirrored


# %% a lever turning about an axis


@pytest.fixture
def lever() -> RevoluteConnection:
    """
    A lever turning between -1 and 1 rad about an axis along the world's x-axis through
    (1, 2, 3), standing at 0.2 rad.
    """
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("kitchen"))
        world.add_kinematic_structure_entity(root)
        lever = Body(name=PrefixedName("lever"))
        joint = RevoluteConnection.create_with_dofs(
            world=world,
            parent=root,
            child=lever,
            axis=Vector3.X(),
            parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                1.0, 2.0, 3.0, yaw=np.pi / 2, reference_frame=root
            ),
            dof_limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                lower_position=-1.0, upper_position=1.0, maximum_speed=1.0
            ),
        )
        world.add_connection(joint)
    joint.position = 0.2
    return joint


def test_a_lever_turns_about_its_axis_as_placed_in_the_world(lever):
    axis = FixtureJoints.axis_of(lever)

    assert axis.distance_to(np.array([1.0, 2.0, 3.0])) == pytest.approx(0.0)
    assert abs(axis.direction @ [0.0, 1.0, 0.0]) == pytest.approx(1.0)


def test_a_lever_is_pushed_to_the_limit_on_the_side_of_a_position(lever):
    limits = lever.dof.limits

    assert FixtureJoints.end_towards(lever, 0.5) == limits.upper.position
    assert FixtureJoints.end_towards(lever, -0.5) == limits.lower.position
