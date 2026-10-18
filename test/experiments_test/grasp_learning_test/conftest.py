"""
The scene and the database the grasp learning tests share.
"""

from __future__ import annotations

import pytest

from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.workers import IterationObject
from experiments.physical_pick_up.objects import PickUpObject
from experiments.physical_pick_up.scene import PickUpScene

# %% fixtures


@pytest.fixture(scope="module")
def milk_scene() -> PickUpScene:
    return IterationObject(description=PickUpObject.MILK.value).scene()


@pytest.fixture
def database() -> GraspDatabase:
    return GraspDatabase.connect("sqlite:///:memory:")
