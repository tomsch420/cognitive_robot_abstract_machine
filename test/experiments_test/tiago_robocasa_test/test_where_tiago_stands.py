"""
Where Tiago is put to work in a RoboCasa kitchen: the stands a task tries, and the
places over a receptacle it tries letting an object go at.
"""

import pytest

pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

import numpy as np

from experiments.tiago_robocasa.pick_place import PickPlaceCounterToSink
from experiments.tiago_robocasa.robocasa_task import StandCandidate

# %% the stands a task tries


def test_the_nearest_stand_facing_the_counter_is_tried_first():
    """
    Nothing is stepped back or along, and the first heading is the one a kitchen is
    usually worked from, so a kitchen Tiago can simply walk up to costs no search.
    """
    task = PickPlaceCounterToSink()

    assert task.stands_to_try()[0] == StandCandidate(
        back=0.0, along=0.0, heading=task.headings_to_try[0]
    )


def test_every_heading_is_tried_at_every_stand():
    """
    Which way Tiago has to face is not known before the kitchen is composed, so each way
    round is tried with the same steps back and along.
    """
    task = PickPlaceCounterToSink()
    stands = task.stands_to_try()

    steps_of_each_heading = {
        heading: {
            (stand.back, stand.along) for stand in stands if stand.heading == heading
        }
        for heading in task.headings_to_try
    }
    assert len(stands) == len(task.headings_to_try) * len(
        steps_of_each_heading[task.headings_to_try[0]]
    )
    assert all(
        steps == steps_of_each_heading[task.headings_to_try[0]]
        for steps in steps_of_each_heading.values()
    )


def test_the_stands_step_back_no_further_than_the_search_reaches():
    task = PickPlaceCounterToSink()

    stands = task.stands_to_try()

    assert max(stand.back for stand in stands) == pytest.approx(task.stand_search)
    assert max(abs(stand.along) for stand in stands) == pytest.approx(task.stand_search)


# %% the places a task tries letting go at


def test_letting_go_is_tried_across_the_whole_floor_of_the_receptacle():
    """
    The places lie a step apart, inside the floor by what keeps the object clear of its
    walls.
    """
    task = PickPlaceCounterToSink()
    inset = 0.04

    places = task.places_across(0.0, 0.5, inset)

    assert places[0] == pytest.approx(inset)
    assert max(places) <= 0.5 - inset
    assert np.allclose(np.diff(places), task.release_step)


def test_a_floor_the_object_barely_fits_on_is_let_go_at_over_its_middle():
    """
    A receptacle no wider than the object leaves nowhere to inset to, and its middle is
    the one place the object can go.
    """
    task = PickPlaceCounterToSink()

    assert task.places_across(0.2, 0.3, inset=0.2) == [pytest.approx(0.25)]
