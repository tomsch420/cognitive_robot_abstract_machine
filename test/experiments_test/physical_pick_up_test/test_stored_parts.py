from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import trimesh

from experiments.physical_pick_up.exceptions import NoStoredPartsError
from experiments.physical_pick_up.stored_parts import StoredPartLocator
from semantic_digital_twin.part_finding.locating import PartQuery
from semantic_digital_twin.semantic_annotations.semantic_annotations import Mug

# %% fixtures


@pytest.fixture
def mesh() -> trimesh.Trimesh:
    return trimesh.creation.box(extents=(0.1, 0.1, 0.1))


@pytest.fixture
def directory(tmp_path: Path, mesh: trimesh.Trimesh) -> Path:
    """
    Holds one stored answer: the faces 0 to 3 of :func:`mesh` are a handle.
    """
    np.savez_compressed(
        tmp_path / "mug_0.npz", vertices=mesh.vertices, faces=mesh.faces
    )
    (tmp_path / "mug_0.json").write_text(
        json.dumps(
            {
                "object": "mug_0",
                "parts": [
                    {"relationship": "handle", "faces": [0, 1, 2, 3], "confidence": 0.9}
                ],
            }
        )
    )
    return tmp_path


# %% answering from stored answers


def test_a_query_of_a_stored_mesh_is_answered_with_the_stored_parts(directory, mesh):
    locator = StoredPartLocator(directory=directory)

    [part] = locator.locate(PartQuery.from_mesh(mesh, Mug, ["handle", "rim"]))

    assert part.relationship_name == "handle"
    np.testing.assert_array_equal(part.faces, [0, 1, 2, 3])
    assert part.confidence == 0.9


def test_a_stored_mesh_is_answered_also_once_it_is_turned(directory, mesh):
    locator = StoredPartLocator(directory=directory)
    turned = mesh.copy().apply_transform(
        trimesh.transformations.rotation_matrix(0.7, [0.0, 0.0, 1.0])
    )

    [part] = locator.locate(PartQuery.from_mesh(turned, Mug, ["handle"]))

    np.testing.assert_array_equal(part.faces, [0, 1, 2, 3])


def test_only_the_parts_asked_for_are_answered(directory, mesh):
    locator = StoredPartLocator(directory=directory)

    assert locator.locate(PartQuery.from_mesh(mesh, Mug, ["rim"])) == []


def test_a_mesh_without_a_stored_answer_cannot_be_answered(directory):
    locator = StoredPartLocator(directory=directory)
    other_mesh = trimesh.creation.box(extents=(0.2, 0.1, 0.1))

    with pytest.raises(NoStoredPartsError):
        locator.locate(PartQuery.from_mesh(other_mesh, Mug, ["handle"]))


def test_a_mesh_without_a_stored_answer_has_no_parts_unless_an_answer_is_required(
    directory,
):
    locator = StoredPartLocator(directory=directory, require_stored_answer=False)
    other_mesh = trimesh.creation.box(extents=(0.2, 0.1, 0.1))

    assert locator.locate(PartQuery.from_mesh(other_mesh, Mug, ["handle"])) == []
