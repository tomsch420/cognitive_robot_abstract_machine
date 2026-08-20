import os
import shutil
from pathlib import Path

import pytest

from semantic_digital_twin.adapters.partnet_mobility_dataset.loader import (
    PartNetMobilityDatasetLoader,
    SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME,
)
from semantic_digital_twin.exceptions import MissingSapienAccessTokenError

from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    Connection6DoF,
    RevoluteConnection,
)

PARTNET_MOBILITY_DATASET_DIRECTORY_ENVIRONMENT_VARIABLE_NAME = (
    "PARTNET_MOBILITY_DATASET_DIRECTORY"
)

SYNTHETIC_MODEL_DIRECTORY = Path(__file__).parent / "partnet_mobility_fixture"
SYNTHETIC_MODEL_ID = 1


@pytest.mark.skipif(
    os.getenv(SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME, None) is None,
    reason="SAPIEN access token not set",
)
def test_loader():
    loader = PartNetMobilityDatasetLoader()
    world = loader.load()
    assert len(world.bodies) > 0
    assert len(world.semantic_annotations) > 0

    unique_connection_types = {type(c) for c in world.connections}
    interesting_connection_types = unique_connection_types - {
        FixedConnection,
        Connection6DoF,
    }
    assert interesting_connection_types != {}


# %% loading an already-present corpus, without a token or sapien


def test_loader_is_constructible_without_an_access_token(monkeypatch):
    """
    The token is only needed to download; an on-disk corpus must not require one.
    """
    monkeypatch.delenv(SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME, raising=False)

    loader = PartNetMobilityDatasetLoader(directory=SYNTHETIC_MODEL_DIRECTORY)

    assert loader.token is None


def test_load_from_directory_builds_a_world_from_the_synthetic_model(monkeypatch):
    monkeypatch.delenv(SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME, raising=False)
    loader = PartNetMobilityDatasetLoader(directory=SYNTHETIC_MODEL_DIRECTORY)

    world = loader.load_from_directory(SYNTHETIC_MODEL_ID)

    assert {body.name.name for body in world.bodies} == {"base", "link_1", "link_0"}
    assert {type(connection) for connection in world.connections} == {
        FixedConnection,
        RevoluteConnection,
    }
    assert {type(annotation).__name__ for annotation in world.semantic_annotations} == {
        "PartNetFurniture",
        "PartNetRotationDoor",
    }


def test_load_from_directory_does_not_modify_the_corpus(monkeypatch, tmp_path):
    """
    The corpus is mounted read-only for real runs, so parsing must not write to it.
    """
    monkeypatch.delenv(SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME, raising=False)
    corpus_directory = tmp_path / "corpus"
    shutil.copytree(SYNTHETIC_MODEL_DIRECTORY, corpus_directory)
    urdf_file = corpus_directory / str(SYNTHETIC_MODEL_ID) / "mobility.urdf"
    contents_before = urdf_file.read_bytes()

    PartNetMobilityDatasetLoader(directory=corpus_directory).load_from_directory(
        SYNTHETIC_MODEL_ID
    )

    assert urdf_file.read_bytes() == contents_before


def test_load_applies_the_limit_defaults_without_writing_them_to_disk(monkeypatch):
    monkeypatch.delenv(SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME, raising=False)
    loader = PartNetMobilityDatasetLoader(
        directory=SYNTHETIC_MODEL_DIRECTORY,
        default_effort_for_limit_tags=42.0,
        default_velocity_for_limit_tags=7.0,
    )

    world = loader.load_from_directory(SYNTHETIC_MODEL_ID)

    [revolute_connection] = [
        connection
        for connection in world.connections
        if isinstance(connection, RevoluteConnection)
    ]
    assert revolute_connection.raw_dof.limits.upper.velocity == 7.0


def test_load_without_a_token_explains_that_downloading_needs_one(monkeypatch):
    monkeypatch.delenv(SAPIEN_ACCESS_TOKEN_ENVIRONMENT_VARIABLE_NAME, raising=False)
    loader = PartNetMobilityDatasetLoader(directory=SYNTHETIC_MODEL_DIRECTORY)

    with pytest.raises(MissingSapienAccessTokenError):
        loader.load(SYNTHETIC_MODEL_ID)


# %% the real corpus, over the remote workflow


@pytest.mark.skipif(
    os.getenv(PARTNET_MOBILITY_DATASET_DIRECTORY_ENVIRONMENT_VARIABLE_NAME, None)
    is None,
    reason="PartNet-Mobility corpus directory not set",
)
def test_load_from_directory_on_the_real_corpus():
    """
    Asserts the values measured by hand on neem-4's copy of model 35059.
    """
    directory = Path(
        os.environ[PARTNET_MOBILITY_DATASET_DIRECTORY_ENVIRONMENT_VARIABLE_NAME]
    )
    loader = PartNetMobilityDatasetLoader(directory=directory)

    world = loader.load_from_directory(35059)

    assert len(world.bodies) == 3
    assert {type(connection) for connection in world.connections} == {
        FixedConnection,
        RevoluteConnection,
    }
    assert {type(annotation).__name__ for annotation in world.semantic_annotations} == {
        "PartNetFurniture",
        "PartNetRotationDoor",
    }
