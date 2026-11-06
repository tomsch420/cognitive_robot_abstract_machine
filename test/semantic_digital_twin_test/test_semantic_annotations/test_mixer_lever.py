"""
The lever of a single lever mixer tap, read off the hinge it tilts on and the swivel
that hinge sits on.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import NotAMixerLever
from semantic_digital_twin.semantic_annotations.semantic_annotations import MixerLever
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    RevoluteConnection,
)
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
    LimitSide,
)
from semantic_digital_twin.world_description.world_entity import Body

# %% a mixer tap


@dataclass
class MixerTap:
    """
    A lever tilting on a hinge about y that sits on a swivel about x, as RoboCasa's sink
    faucet handle does.
    """

    lever: MixerLever
    """
    The lever, the hot water at the upper end of the swivel's range.
    """

    hinge: RevoluteConnection
    """
    The hinge the lever tilts on.
    """

    swivel: RevoluteConnection
    """
    The swivel the hinge sits on.
    """


@pytest.fixture
def mixer_tap() -> MixerTap:
    world = World()
    with world.modify_world():
        tap = Body(name=PrefixedName("tap"))
        world.add_kinematic_structure_entity(tap)
        swivel_body = Body(name=PrefixedName("swivel"))
        lever_body = Body(name=PrefixedName("lever"))
        swivel = RevoluteConnection.create_with_dofs(
            world=world,
            parent=tap,
            child=swivel_body,
            axis=Vector3.X(),
            dof_limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                lower_position=-0.785, upper_position=0.785, maximum_speed=1.0
            ),
        )
        world.add_connection(swivel)
        hinge = RevoluteConnection.create_with_dofs(
            world=world,
            parent=swivel_body,
            child=lever_body,
            axis=Vector3.Y(),
            dof_limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                lower_position=0.0, upper_position=0.5, maximum_speed=1.0
            ),
        )
        world.add_connection(hinge)
        lever = MixerLever(root=lever_body, hot_side=LimitSide.UPPER)
        world.add_semantic_annotation(lever)
    return MixerTap(lever=lever, hinge=hinge, swivel=swivel)


# %% the joints


def test_the_lever_tilts_on_its_hinge_to_let_the_water_flow(mixer_tap):
    assert mixer_tap.lever.flow_joint is mixer_tap.hinge


def test_the_lever_swings_on_the_swivel_to_set_the_temperature(mixer_tap):
    assert mixer_tap.lever.temperature_joint is mixer_tap.swivel


def test_the_water_runs_hottest_at_the_hot_side_of_the_swivel(mixer_tap):
    limits = mixer_tap.swivel.dof.limits

    assert mixer_tap.lever.hottest_position == limits.upper.position
    assert mixer_tap.lever.coldest_position == limits.lower.position


def test_a_lever_fixed_in_place_is_not_a_mixer_lever():
    world = World()
    with world.modify_world():
        tap = Body(name=PrefixedName("tap"))
        lever_body = Body(name=PrefixedName("lever"))
        world.add_kinematic_structure_entity(tap)
        world.add_connection(
            FixedConnection(
                parent=tap,
                child=lever_body,
                parent_T_connection_expression=HomogeneousTransformationMatrix(
                    reference_frame=tap
                ),
            )
        )
        lever = MixerLever(root=lever_body, hot_side=LimitSide.UPPER)
        world.add_semantic_annotation(lever)

    with pytest.raises(NotAMixerLever):
        lever.flow_joint


# %% the sides of a range


@pytest.mark.parametrize("side", list(LimitSide))
def test_the_opposite_side_of_a_range_is_the_other_one(side):
    assert side.opposite is not side
    assert side.opposite.opposite is side


def test_each_side_of_a_range_is_at_its_own_limit(mixer_tap):
    limits = mixer_tap.swivel.dof.limits

    assert LimitSide.LOWER.position_in(limits) == limits.lower.position
    assert LimitSide.UPPER.position_in(limits) == limits.upper.position
