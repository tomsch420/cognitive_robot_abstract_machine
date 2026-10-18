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
class StoredAnswer:
    """
    The parts stored for one mesh, with what the mesh is recognized by.
    """

    edge_lengths: NDArray[np.float64]
    """
    The length of every edge of every face of the mesh, which stays the same when the
    mesh is moved or turned.
    """

    parts: List[StoredPart]
    """
    The parts located in the mesh.
    """


@dataclass
class StoredPartLocator(PartLocator):
    """
    Answers a query with the parts stored for the mesh it asks about.

    The directory holds, for every object, ``<object>.json``, the server's answer, whose
    ``parts`` are :class:`StoredPart`, and ``<object>.npz``, the ``vertices`` and
    ``faces`` of the mesh the answer was located in. A query is answered for a mesh with
    the same faces whose edges are as long, so also once the mesh is moved or turned.
    """

    directory: Path
    """
    Where the stored answers are.
    """

    require_stored_answer: bool = True
    """
    Whether a mesh needs a stored answer; one that does not is taken to have no parts.
    """

    edge_length_tolerance: float = 1e-5
    """
    How much the edges of a queried mesh may differ in length from those of a stored
    one, in meters, for the two to be the same mesh.
    """

    answers_by_faces: Dict[str, List[StoredAnswer]] = field(init=False, repr=False)
    """
    The stored answers, by :meth:`faces_key` of the mesh they were located in.
    """

    def __post_init__(self):
        self.answers_by_faces = {}
        for mesh_file in sorted(self.directory.glob("*.npz")):
            answer_file = mesh_file.with_suffix(".json")
            if not answer_file.is_file():
                continue
            mesh = np.load(mesh_file)
            answer = json.loads(answer_file.read_text())
            self.answers_by_faces.setdefault(self.faces_key(mesh["faces"]), []).append(
                StoredAnswer(
                    edge_lengths=self.edge_lengths(mesh["vertices"], mesh["faces"]),
                    parts=[StoredPart.from_json(part) for part in answer["parts"]],
                )
            )

    def locate(self, query: PartQuery) -> list[LocatedPart]:
        """
        :raises NoStoredPartsError: If no answer is stored for the query's mesh and
            :attr:`require_stored_answer` says it has to be.
        """
        edge_lengths = self.edge_lengths(query.vertices, query.faces)
        answers = [
            answer
            for answer in self.answers_by_faces.get(self.faces_key(query.faces), [])
            if np.allclose(
                answer.edge_lengths,
                edge_lengths,
                rtol=0.0,
                atol=self.edge_length_tolerance,
            )
        ]
        if not answers and not self.require_stored_answer:
            return []
        if not answers:
            raise NoStoredPartsError(
                whole_type=query.whole_type, directory=self.directory
            )
        return [
            part.located_part()
            for part in answers[0].parts
            if part.relationship in query.relationship_names
        ]

    @staticmethod
    def faces_key(faces: NDArray[np.int64]) -> str:
        """
        :param faces: The faces of a mesh.
        :return: A name that is the same exactly for the same faces.
        """
        return hashlib.sha256(np.asarray(faces, dtype=np.int64).tobytes()).hexdigest()

    @staticmethod
    def edge_lengths(
        vertices: NDArray[np.float64], faces: NDArray[np.int64]
    ) -> NDArray[np.float64]:
        """
        :param vertices: The vertices of a mesh.
        :param faces: Its faces.
        :return: The length of every edge of every face, face by face.
        """
        corners = np.asarray(vertices, dtype=np.float64)[np.asarray(faces)]
        return np.linalg.norm(corners - np.roll(corners, 1, axis=1), axis=2)
