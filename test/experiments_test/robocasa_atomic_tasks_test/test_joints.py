"""
The positions of RoboCasa's joints as RoboCasa states them.
"""

from __future__ import annotations

from experiments.robocasa_atomic_tasks.joints import JointRange


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
