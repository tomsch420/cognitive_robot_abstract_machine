"""
A PartNet-Mobility model expressed as flat, minable dataclasses.

The relational structure of a loaded :class:`World` is not reachable by rule mining:
attribute discovery follows declared dataclass fields, and a
:class:`~semantic_digital_twin.world_description.world_entity.Body` reaches its
connections only through properties over its world. These classes restate one model so
every relation mining needs is a declared field.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from xml.etree import ElementTree

from typing_extensions import Dict, List, Optional

from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection
from semantic_digital_twin.world_description.world_entity import Body

# %% the dataset's own vocabularies

HANDLE_PART_NAME = "handle"
"""
The part name PartNet uses for a graspable handle.
"""

MOBILITY_FILE_NAME = "mobility_v2.json"
"""
The per-model file describing each link's parts and motion.
"""

URDF_FILE_NAME = "mobility.urdf"
"""
The per-model URDF, which declares each joint's type.
"""

SEMANTICS_FILE_NAME = "semantics.txt"
"""
The per-model file labelling each link.
"""

LINK_NAME_PREFIX = "link_"
"""
The prefix URDF link names carry, followed by the link's index.
"""

NO_PARENT_LINK_INDEX = -1
"""
The parent index a root link carries.
"""


class PartNetMotionKind(StrEnum):
    """
    How a link moves, as named by the dataset.
    """

    HINGE = "hinge"
    SLIDER = "slider"
    SLIDER_PLUS = "slider+"
    FREE = "free"
    HEAVY = "heavy"
    STATIC = "static"


class UrdfJointType(StrEnum):
    """
    The joint types a URDF may declare.
    """

    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    FLOATING = "floating"
    PLANAR = "planar"


class StorageFurnitureLabel(StrEnum):
    """
    The labels ``semantics.txt`` uses within the ``StorageFurniture`` category.

    .. note::
        The dataset's label vocabulary is open — 107 distinct labels across all 46
        categories — so a link's label is kept as plain text. This enum exists so code
        naming one of these labels does not spell it as a bare string.
    """

    DRAWER = "drawer"
    ROTATION_DOOR = "rotation_door"
    TRANSLATION_DOOR = "translation_door"
    FURNITURE_BODY = "furniture_body"
    WHEEL = "wheel"
    CASTER = "caster"
    BOARD = "board"


# %% one model


@dataclass
class PartNetPart:
    """
    A fine-grained part of a link, from the dataset's own part hierarchy.
    """

    identifier: int
    """
    The part's id within its model.
    """

    name: str
    """
    The part's name in the dataset's part hierarchy.
    """


@dataclass
class PartNetLink:
    """
    One movable or static link of a model, carrying both of the dataset's vocabularies.
    """

    index: int
    """
    The link's index, matching its URDF name.
    """

    semantic_label: str
    """
    The label ``semantics.txt`` gives the link.
    """

    motion_kind: PartNetMotionKind
    """
    How the link moves.
    """

    part_name: str
    """
    The name the part hierarchy gives the link, which often disagrees with
    :attr:`semantic_label`.
    """

    parts: List[PartNetPart] = field(default_factory=list)
    """
    The fine-grained parts composing the link.
    """

    joint_type: Optional[UrdfJointType] = None
    """
    The type of the joint attaching the link to its parent, absent for a root link.
    """

    body: Optional[Body] = field(default=None, init=False)
    """
    The loaded world's body for this link, absent when the model was read without one.

    Declared ``init=False`` so attribute discovery skips it: a body reaches whole mesh
    collections, and a rule miner walking declared fields would search through geometry
    that says nothing about annotation.
    """

    connection: Optional[Connection] = field(default=None, init=False)
    """
    The connection joining the link to its parent, absent when the model was read
    without a world.

    Excluded from attribute discovery for the same reason as
    :attr:`body`, which it reaches through its own endpoints.
    """

    parent_link: Optional[PartNetLink] = None
    """
    The link this one hangs from, absent for a root link.
    """

    @property
    def has_handle(self) -> bool:
        """
        :return: Whether any of the link's parts is a handle.
        """
        return any(part.name == HANDLE_PART_NAME for part in self.parts)


@dataclass
class PartNetModel:
    """
    A single PartNet-Mobility model, ready to be mined.
    """

    model_id: int
    """
    The model's id within the dataset.
    """

    links: List[PartNetLink] = field(default_factory=list)
    """
    The model's links.
    """

    @classmethod
    def from_dataset(cls, model_directory: Path, model_id: int) -> PartNetModel:
        """
        Build a model from the dataset files alone.

        No meshes are read, so this is the path to use when mining over many models:
        loading a world runs convex decomposition over every mesh, which dominates the
        runtime and contributes nothing a rule can be about.

        :param model_directory: The model's own directory in the dataset.
        :param model_id: The model's id.
        :return: The model, without bodies or connections.
        """
        return cls(
            model_id=model_id, links=cls._build_links(model_directory, world=None)
        )

    @classmethod
    def from_world(
        cls, world: World, model_directory: Path, model_id: int
    ) -> PartNetModel:
        """
        Build a model from a loaded world and the dataset files beside it.

        :param world: The world the model's URDF was parsed into.
        :param model_directory: The model's own directory in the dataset.
        :param model_id: The model's id.
        :return: The model, with every link wired to its body and connection.
        """
        return cls(model_id=model_id, links=cls._build_links(model_directory, world))

    @classmethod
    def _build_links(
        cls, model_directory: Path, world: Optional[World]
    ) -> List[PartNetLink]:
        """
        :param model_directory: The model's own directory in the dataset.
        :param world: The loaded world, when there is one.
        :return: The model's links, wired to each other and to ``world`` if given.
        """
        labels = cls._read_labels(model_directory / SEMANTICS_FILE_NAME)
        joint_types = cls._read_joint_types(model_directory / URDF_FILE_NAME)
        entries = json.loads((model_directory / MOBILITY_FILE_NAME).read_text())

        links_by_index: Dict[int, PartNetLink] = {}
        for entry in entries:
            index = entry["id"]
            links_by_index[index] = PartNetLink(
                index=index,
                semantic_label=labels[index],
                motion_kind=PartNetMotionKind(entry["joint"]),
                part_name=entry["name"],
                parts=[
                    PartNetPart(identifier=part["id"], name=part["name"])
                    for part in entry["parts"]
                ],
                joint_type=joint_types.get(index),
            )

        for entry in entries:
            link = links_by_index[entry["id"]]
            parent_index = entry["parent"]
            if parent_index != NO_PARENT_LINK_INDEX:
                link.parent_link = links_by_index[parent_index]
            if world is not None:
                link.body = world.get_body_by_name(f"{LINK_NAME_PREFIX}{link.index}")
                link.connection = cls._parent_connection(world, link.body)

        return list(links_by_index.values())

    @staticmethod
    def _read_joint_types(urdf_file: Path) -> Dict[int, UrdfJointType]:
        """
        :param urdf_file: The model's URDF.
        :return: The joint type attaching each link to its parent, by link index.
        """
        joint_types: Dict[int, UrdfJointType] = {}
        for joint in ElementTree.parse(urdf_file).getroot().findall("joint"):
            child_name = joint.find("child").get("link")
            if not child_name.startswith(LINK_NAME_PREFIX):
                continue
            index = int(child_name.removeprefix(LINK_NAME_PREFIX))
            joint_types[index] = UrdfJointType(joint.get("type"))
        return joint_types

    @staticmethod
    def _read_labels(semantics_file: Path) -> Dict[int, str]:
        """
        :return: Each link's ``semantics.txt`` label, by link index.
        """
        labels: Dict[int, str] = {}
        for line in semantics_file.read_text().splitlines():
            if not line.strip():
                continue
            link_name, _, label = line.split()
            labels[int(link_name.removeprefix(LINK_NAME_PREFIX))] = label
        return labels

    @staticmethod
    def _parent_connection(world: World, body: Body) -> Optional[Connection]:
        """
        :return: The connection whose child is ``body``, absent for the world's root.
        """
        for connection in world.connections:
            if connection.child is body:
                return connection
        return None
