"""
The HSR opening and closing the oven's door in a RoboCasa kitchen, simulated in MuJoCo with the
door moved by nothing but the HSR's hold on its bar, as SegMind sees it.
"""

import pytest

pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.hsr_robocasa.doors import CloseOven, OpenOven
from segmind.datastructures.events import ClosedEvent, OpenedEvent

from .test_drawers import last_event_of_the_moving_part
from .test_pick_place import runs_in_ci

# %% taking hold of the bar


def test_the_bar_is_taken_hold_of_as_the_door_turns():
    """
    The grasp is level where the door is closed and turns with the door where it
    stands open, so that it stands the same on the door however far the door is open.
    """
    experiment = OpenOven(headless=True)
    experiment.build_world()
    hinge = experiment.door.movable_joint
    closed = experiment.handle_grasp().grasp_pose.to_np()

    hinge.position = hinge.dof.limits.upper.position
    opened = experiment.handle_grasp().grasp_pose.to_np()

    assert opened == pytest.approx(closed, abs=1e-6)


# %% the run


@runs_in_ci
def test_the_hsr_opens_the_oven():
    experiment = OpenOven(headless=True)
    experiment.build_world()

    experiment.run()

    assert type(last_event_of_the_moving_part(experiment)) is OpenedEvent


@runs_in_ci
def test_the_hsr_closes_the_oven():
    experiment = CloseOven(headless=True)
    experiment.build_world()

    experiment.run()

    assert type(last_event_of_the_moving_part(experiment)) is ClosedEvent
