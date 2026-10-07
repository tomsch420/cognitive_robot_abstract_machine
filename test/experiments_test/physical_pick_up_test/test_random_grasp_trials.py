"""
The PR2 tries grasps drawn from the statement of where the object's annotation may be
grasped.
"""

from __future__ import annotations

import pytest

from experiments.physical_pick_up.objects import PickUpObject
from experiments.physical_pick_up.pr2_pick_up import PR2PickUpExperiment
from experiments.physical_pick_up.random_grasp_trials import PR2GraspTrier
from experiments.physical_pick_up.scene import ObjectOnTableScene
from semantic_digital_twin.grasping.grasp_trials import GraspTrials
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle

# %% fixtures


def trials_on(pick_up_object: PickUpObject) -> GraspTrials:
    """
    :return: Trials of the PR2 grasping ``pick_up_object``.
    """
    experiment = PR2PickUpExperiment(
        scene=ObjectOnTableScene(object_description=pick_up_object.value)
    )
    return GraspTrials(
        graspable=experiment.scene.graspable,
        trier=PR2GraspTrier(experiment=experiment),
        number_of_trials=30,
    )


@pytest.fixture(scope="module")
def milk_trials() -> GraspTrials:
    return trials_on(PickUpObject.MILK)


@pytest.fixture(scope="module")
def mug_trials() -> GraspTrials:
    return trials_on(PickUpObject.YCB_MUG)


# %% where the grasps are placed


def test_an_object_without_a_handle_is_grasped_as_a_whole(milk_trials):
    assert milk_trials.grasped_part is milk_trials.graspable


def test_a_mug_is_grasped_at_the_handle_found_in_its_shape(mug_trials):
    mug = mug_trials.graspable

    assert isinstance(mug.handle, Handle)
    assert mug_trials.grasped_part is mug.handle


def test_grasps_drawn_for_a_mug_reach_its_handle(mug_trials):
    handle = mug_trials.grasped_part

    reaching = [
        grasp for grasp in mug_trials.drawn_grasps() if grasp.reaches_surface_of(handle)
    ]

    assert reaching
    for grasp in reaching:
        assert grasp.grasp_candidate(handle).graspable is handle
