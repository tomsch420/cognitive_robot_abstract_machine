"""
The HSR opening and closing a drawer in a RoboCasa kitchen, simulated in MuJoCo with
the drawer moved by nothing but the HSR's hold on its handle, as SegMind sees it.
"""

import pytest

pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.hsr_robocasa.drawers import CloseDrawer, OpenDrawer
from experiments.hsr_robocasa.handles import HSRHandleTask
from segmind.datastructures.events import ClosedEvent, DetectionEvent, OpenedEvent

from .test_pick_place import runs_in_ci

# %% what SegMind saw


def last_event_of_the_moving_part(experiment: HSRHandleTask) -> DetectionEvent:
    """
    :return: The last event SegMind detected for the part of the fixture that moves.
    """
    return [
        event
        for event in experiment.detected_events
        if experiment.moving_part in event.tracked_objects
    ][-1]


# %% the runs


@runs_in_ci
def test_the_hsr_opens_the_drawer():
    experiment = OpenDrawer(headless=True)
    experiment.build_world()

    experiment.run()

    assert type(last_event_of_the_moving_part(experiment)) is OpenedEvent


@runs_in_ci
def test_the_hsr_closes_the_drawer():
    experiment = CloseDrawer(headless=True)
    experiment.build_world()

    experiment.run()

    assert type(last_event_of_the_moving_part(experiment)) is ClosedEvent
