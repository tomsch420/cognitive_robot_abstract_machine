"""
Finding an object's handle or rim in its shape and splitting it off.
"""

import pytest
import trimesh

from semantic_digital_twin.pipeline.handle_finding import (
    NarrowEndHandleFinder,
    ProtrudingHandleFinder,
)
from semantic_digital_twin.pipeline.rim_finding import RimFinder

# %% shapes

BODY_OUTER_RADIUS = 0.04
"""
Outer radius of the round body of the synthetic mug.
"""

HANDLE_REACH = 0.02
"""
How far the synthetic mug's handle sticks out of its body.
"""

TOOL_HANDLE_LENGTH = 0.1
"""
Length of the synthetic tool's handle, which runs from x = -0.1 to x = 0.
"""

TOOL_HANDLE_WIDTH = 0.01
"""
Width of the synthetic tool's handle.
"""


def cup_without_handle() -> trimesh.Trimesh:
    """
    :return: A round body with a wall, open at the top.
    """
    return trimesh.creation.annulus(
        r_min=BODY_OUTER_RADIUS - 0.005, r_max=BODY_OUTER_RADIUS, height=0.1
    )


def mug() -> trimesh.Trimesh:
    """
    :return: The cup with a box-shaped handle sticking out towards positive x.
    """
    handle = trimesh.creation.box(extents=[HANDLE_REACH, 0.01, 0.06])
    handle.apply_translation([BODY_OUTER_RADIUS + HANDLE_REACH / 2, 0.0, 0.0])
    return trimesh.util.concatenate([cup_without_handle(), handle])


def tool() -> trimesh.Trimesh:
    """
    :return: A flat tool, its narrow handle towards negative x and its wide head
        towards positive x.
    """
    handle = trimesh.creation.box(
        extents=[TOOL_HANDLE_LENGTH, TOOL_HANDLE_WIDTH, 0.005]
    )
    handle.apply_translation([-TOOL_HANDLE_LENGTH / 2, 0.0, 0.0])
    head = trimesh.creation.box(extents=[0.05, 0.04, 0.005])
    head.apply_translation([0.025, 0.0, 0.0])
    return trimesh.util.concatenate([handle, head])


# %% handles sticking out of a round body


def test_the_handle_of_a_mug_is_what_sticks_out_of_its_round_body():
    split = ProtrudingHandleFinder().split(mug())

    lowest, highest = split.part.bounds
    assert lowest[0] >= BODY_OUTER_RADIUS - 1e-9
    assert highest[0] == pytest.approx(BODY_OUTER_RADIUS + HANDLE_REACH)
    assert split.rest.bounds[1][0] == pytest.approx(BODY_OUTER_RADIUS)
    assert len(split.part.faces) + len(split.rest.faces) == len(mug().faces)


def test_a_cup_without_a_handle_has_none():
    assert ProtrudingHandleFinder().split(cup_without_handle()) is None


# %% handles at the narrow end of a tool


def test_the_handle_of_a_tool_is_its_narrow_end_cut_off_where_it_widens():
    split = NarrowEndHandleFinder().split(tool())

    lowest, highest = split.part.bounds
    assert lowest[0] == pytest.approx(-TOOL_HANDLE_LENGTH)
    assert highest[0] == pytest.approx(0.0)
    assert highest[1] - lowest[1] == pytest.approx(TOOL_HANDLE_WIDTH)
    assert split.rest.bounds[0][0] == pytest.approx(0.0)
    assert split.part.is_watertight and split.rest.is_watertight


def test_a_bar_of_even_width_has_no_handle():
    bar = trimesh.creation.box(extents=[0.2, 0.02, 0.02])

    assert NarrowEndHandleFinder().split(bar) is None


# %% rims of open containers


def test_the_rim_of_a_cup_is_the_band_below_its_top():
    finder = RimFinder()

    split = finder.split(cup_without_handle())

    top = cup_without_handle().bounds[1][2]
    assert split.part.bounds[0][2] == pytest.approx(top - finder.depth)
    assert split.part.bounds[1][2] == pytest.approx(top)
    assert split.rest.bounds[1][2] == pytest.approx(top - finder.depth)
    assert split.part.is_watertight and split.rest.is_watertight
    assert split.part.volume + split.rest.volume == pytest.approx(
        cup_without_handle().volume
    )
