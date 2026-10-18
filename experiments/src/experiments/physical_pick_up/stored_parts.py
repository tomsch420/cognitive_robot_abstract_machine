"""
Answering part queries with the parts a part finding server located before, so that an
experiment splits its objects as the server would without asking it again.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Any, Dict, List

from experiments.physical_pick_up.exceptions import NoStoredPartsError
from semantic_digital_twin.part_finding.locating import (
    LocatedPart,
    PartLocator,
    PartQuery,
)

# %% the stored answers


@dataclass
class StoredPart:
    """
    One part a part finding server located, as an answer file stores it.
    """

    relationship: str
    """
    The part-whole relationship the part fills.
    """

    faces: List[int]
    """
    The faces of the part, as indices into the mesh the answer was located in.
    """

    confidence: float
    """
    How sure the server is of the part.
    """

    @classmethod
    def from_json(cls, data: Dict[str, Any]) -> StoredPart:
        """
        :param data: The part, as an answer file stores it.
        :return: The part.
        """
        return cls(
            relationship=data["relationship"],
            faces=data["faces"],
            confidence=data["confidence"],
        )

    def located_part(self) -> LocatedPart:
        """
        :return: The part as a locator answers it.
        """
        return LocatedPart(
            relationship_name=self.relationship,
            faces=np.asarray(self.faces, dtype=np.int64),
            confidence=self.confidence,
        )


# %% answering from the stored answers


@dataclass
class StoredPartLocator(PartLocator):
    """
    Answers a query with the parts stored for the mesh it asks about.

    The directory holds, for every object, ``<object>.json``, the server's answer, whose
    ``parts`` are :class:`StoredPart`, and ``<object>.npz``, the ``vertices`` and
    ``faces`` of the mesh the answer was located in. A query is answered for that mesh
    also when it is moved or turned.
    """

    directory: Path
    """
    Where the stored answers are.
    """

    parts_by_mesh: Dict[str, List[StoredPart]] = field(init=False, repr=False)
    """
    The stored parts, by :meth:`mesh_key` of the mesh they were located in.
    """

    def __post_init__(self):
        self.parts_by_mesh = {}
        for mesh_file in sorted(self.directory.glob("*.npz")):
            answer_file = mesh_file.with_suffix(".json")
            if not answer_file.is_file():
                continue
            mesh = np.load(mesh_file)
            answer = json.loads(answer_file.read_text())
            self.parts_by_mesh[self.mesh_key(mesh["vertices"], mesh["faces"])] = [
                StoredPart.from_json(part) for part in answer["parts"]
            ]

    def locate(self, query: PartQuery) -> list[LocatedPart]:
        """
        :raises NoStoredPartsError: If no answer is stored for the query's mesh.
        """
        key = self.mesh_key(query.vertices, query.faces)
        if key not in self.parts_by_mesh:
            raise NoStoredPartsError(
                whole_type=query.whole_type, directory=self.directory
            )
        return [
            part.located_part()
            for part in self.parts_by_mesh[key]
            if part.relationship in query.relationship_names
        ]

    @staticmethod
    def mesh_key(
        vertices: NDArray[np.float64],
        faces: NDArray[np.int64],
        decimals: int = 6,
    ) -> str:
        """
        :param vertices: The vertices of a mesh.
        :param faces: Its faces.
        :param decimals: To how many decimals of a meter the lengths of the edges are
            compared.
        :return: A name that is the same for the same mesh, also once it is moved or
            turned.
        """
        faces = np.asarray(faces, dtype=np.int64)
        corners = np.asarray(vertices, dtype=np.float64)[faces]
        edge_lengths = np.linalg.norm(corners - np.roll(corners, 1, axis=1), axis=2)
        content = faces.tobytes() + np.round(edge_lengths, decimals).tobytes()
        return hashlib.sha256(content).hexdigest()
