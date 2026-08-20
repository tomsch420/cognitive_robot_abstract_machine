import os
from pathlib import Path

import pytest

from semantic_digital_twin.adapters.partnet_mobility_dataset.domain_model import (
    HANDLE_PART_NAME,
    PartNetLink,
    PartNetModel,
    PartNetMotionKind,
    StorageFurnitureLabel,
)
from semantic_digital_twin.adapters.partnet_mobility_dataset.loader import (
    PartNetMobilityDatasetLoader,
    SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME,
)
from semantic_digital_twin.world_description.connections import (
    PrismaticConnection,
    RevoluteConnection,
)

PARTNET_MOBILITY_DATASET_DIRECTORY_ENVIRONMENT_VARIABLE_NAME = (
    "PARTNET_MOBILITY_DATASET_DIRECTORY"
)

SYNTHETIC_MODEL_DIRECTORY = Path(__file__).parent / "partnet_mobility_fixture"
SYNTHETIC_MODEL_ID = 1


@pytest.fixture
def synthetic_model(monkeypatch) -> PartNetModel:
    monkeypatch.delenv(SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME, raising=False)
    loader = PartNetMobilityDatasetLoader(directory=SYNTHETIC_MODEL_DIRECTORY)
    world = loader.load_from_directory(SYNTHETIC_MODEL_ID)
    return PartNetModel.from_world(
        world=world,
        model_directory=SYNTHETIC_MODEL_DIRECTORY / str(SYNTHETIC_MODEL_ID),
        model_id=SYNTHETIC_MODEL_ID,
    )


# %% parsing the two PartNet vocabularies


def test_links_carry_both_vocabularies(synthetic_model: PartNetModel):
    """
    ``semantics.txt``'s label and ``mobility_v2.json``'s name disagree in the real
    corpus, so both are kept rather than one standing in for the other.
    """
    by_index = {link.index: link for link in synthetic_model.links}

    assert sorted(by_index) == [0, 1]
    assert by_index[0].semantic_label == StorageFurnitureLabel.ROTATION_DOOR
    assert by_index[0].part_name == "cabinet_door"
    assert by_index[1].semantic_label == StorageFurnitureLabel.FURNITURE_BODY
    assert by_index[1].part_name == "cabinet_frame"


def test_motion_kind_is_an_enum_member(synthetic_model: PartNetModel):
    by_index = {link.index: link for link in synthetic_model.links}

    assert by_index[0].motion_kind == PartNetMotionKind.HINGE
    assert by_index[1].motion_kind == PartNetMotionKind.HEAVY


def test_parts_are_kept_with_their_identifiers(synthetic_model: PartNetModel):
    [door] = [link for link in synthetic_model.links if link.index == 0]

    assert [(part.identifier, part.name) for part in door.parts] == [
        (10, "cabinet_door_surface"),
        (11, HANDLE_PART_NAME),
    ]


def test_handle_presence_is_derived_from_the_parts(synthetic_model: PartNetModel):
    by_index = {link.index: link for link in synthetic_model.links}

    assert by_index[0].has_handle
    assert not by_index[1].has_handle


# %% wiring to the loaded world


def test_links_resolve_to_their_world_body(synthetic_model: PartNetModel):
    by_index = {link.index: link for link in synthetic_model.links}

    assert by_index[0].body.name.name == "link_0"
    assert by_index[1].body.name.name == "link_1"


def test_links_resolve_to_their_parent_connection(synthetic_model: PartNetModel):
    [door] = [link for link in synthetic_model.links if link.index == 0]

    assert isinstance(door.connection, RevoluteConnection)


def test_parent_link_follows_mobility_v2(synthetic_model: PartNetModel):
    by_index = {link.index: link for link in synthetic_model.links}

    assert by_index[0].parent_link is by_index[1]
    assert by_index[1].parent_link is None


# %% the real corpus


@pytest.mark.skipif(
    os.getenv(PARTNET_MOBILITY_DATASET_DIRECTORY_ENVIRONMENT_VARIABLE_NAME, None)
    is None,
    reason="PartNet-Mobility corpus directory not set",
)
def test_real_drawer_link_has_a_handle_and_a_prismatic_connection():
    """
    Model 45162 has one drawer, one rotation door and the furniture body.
    """
    directory = Path(
        os.environ[PARTNET_MOBILITY_DATASET_DIRECTORY_ENVIRONMENT_VARIABLE_NAME]
    )
    world = PartNetMobilityDatasetLoader(directory=directory).load_from_directory(45162)
    model = PartNetModel.from_world(
        world=world, model_directory=directory / "45162", model_id=45162
    )

    assert len(model.links) == 3
    [drawer] = [
        link
        for link in model.links
        if link.semantic_label == StorageFurnitureLabel.DRAWER
    ]
    assert drawer.motion_kind == PartNetMotionKind.SLIDER
    assert drawer.has_handle
    assert isinstance(drawer.connection, PrismaticConnection)
