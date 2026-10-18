"""
The graspable objects of RoboCasa, described for the pick-up experiment.
"""

from __future__ import annotations

import argparse
import inspect
from dataclasses import replace

import pytest

from experiments.physical_pick_up.exceptions import NoDefaultGraspError
from experiments.physical_pick_up.objects import (
    GraspableRoboCasaObject,
    ObjectChoice,
    PickUpObject,
)
from experiments.physical_pick_up.pick_up_experiment import PickUpExperiment
from experiments.physical_pick_up.scene import PickUpScene
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
)
from semantic_digital_twin.part_finding.annotation_part_finder import (
    AnnotationPartFinder,
)
from semantic_digital_twin.part_finding.locating import KnownPartLocator

requires_robocasa_assets = pytest.mark.skipif(
    not (RoboCasaDatasetLoader().directory / "objects").is_dir(),
    reason="the RoboCasa object assets are not downloaded",
)

# %% describing a graspable RoboCasa object


@pytest.mark.parametrize(
    "graspable", list(GraspableRoboCasaObject), ids=lambda graspable: graspable.name
)
def test_every_graspable_robocasa_object_is_given_an_annotation_it_can_have(graspable):
    assert not inspect.isabstract(graspable.value.semantic_annotation_type)


@requires_robocasa_assets
def test_a_graspable_robocasa_object_weighs_its_density_times_its_volume():
    graspable = GraspableRoboCasaObject.APPLE.value
    density = 250.0

    description = graspable.description(instance_index=1, density=density)

    volume = sum(
        part.convex_hull.volume for part in description.load_geometry().collision_parts
    )
    assert description.category == graspable.category
    assert description.instance_index == 1
    assert description.semantic_annotation_type is graspable.semantic_annotation_type
    assert description.mass == pytest.approx(density * volume)


@requires_robocasa_assets
def test_the_command_line_takes_a_graspable_robocasa_object():
    parser = argparse.ArgumentParser()
    object_choice = ObjectChoice(parser)
    object_choice.add_arguments()

    description = object_choice.description(
        parser.parse_args(
            [
                "--robocasa-object",
                GraspableRoboCasaObject.APPLE.name.lower(),
                "--instance",
                "2",
            ]
        )
    )

    expected = GraspableRoboCasaObject.APPLE.value.description(instance_index=2)
    assert (
        description.category,
        description.instance_index,
        description.semantic_annotation_type,
        description.mass,
    ) == (
        expected.category,
        expected.instance_index,
        expected.semantic_annotation_type,
        expected.mass,
    )


# %% an object without a default grasp


def test_an_attempt_without_a_grasp_needs_a_default_grasp():
    scene = PickUpScene(
        object_description=replace(PickUpObject.MILK.value, default_grasp=None),
        part_finder=AnnotationPartFinder(locator=KnownPartLocator()),
    )

    with pytest.raises(NoDefaultGraspError):
        PickUpExperiment(scene=scene).run()
