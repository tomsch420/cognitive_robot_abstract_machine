"""
Learning the grasp model of a RoboCasa object category from the attempts on its own
objects.
"""

from __future__ import annotations

from dataclasses import replace

import pytest
from typing_extensions import List

from .test_grasp_learning import database, milk_scene, synthetic_attempts

from experiments.grasp_learning.category_models import CategoryGraspModels
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.records import GraspAttempt
from experiments.physical_pick_up.objects import GraspableRoboCasaObject
from experiments.physical_pick_up.scene import PickUpScene

# %% attempts on named objects


def attempts_on(
    scene: PickUpScene, object_name: str, number: int
) -> List[GraspAttempt]:
    """
    :return: Synthetic attempts on the scene's object, stored as if tried on the object
        of that name.
    """
    return [
        replace(attempt, object_name=object_name)
        for attempt in synthetic_attempts(scene, number)
    ]


# %% telling the objects of a category apart


def test_a_category_names_the_bodies_of_its_own_objects_only():
    bar = GraspableRoboCasaObject.BAR.value

    assert bar.is_body_name_of(bar.body_name(3))
    assert not bar.is_body_name_of(GraspableRoboCasaObject.BAR_SOAP.value.body_name(3))


# %% learning per category


def test_a_category_model_is_learned_from_the_attempts_on_its_own_objects_only(
    milk_scene, database
):
    bar = GraspableRoboCasaObject.BAR.value
    bar_soap = GraspableRoboCasaObject.BAR_SOAP.value
    own = attempts_on(milk_scene, bar.body_name(0), 40) + attempts_on(
        milk_scene, bar.body_name(1), 30
    )
    database.add_attempts(own)
    database.add_attempts(attempts_on(milk_scene, bar_soap.body_name(0), 50))

    model = CategoryGraspModels(database=database).model_for(bar.category)

    assert model.number_of_attempts == len(own)
    assert model.task == own[0].task


def test_a_category_without_attempts_has_no_model(milk_scene, database):
    database.add_attempts(
        attempts_on(milk_scene, GraspableRoboCasaObject.BAR_SOAP.value.body_name(0), 50)
    )

    models = CategoryGraspModels(database=database)

    assert models.model_for(GraspableRoboCasaObject.BAR.value.category) is None


def test_a_category_that_is_not_graspable_has_no_model(database: GraspDatabase):
    assert CategoryGraspModels(database=database).model_for("sink") is None
