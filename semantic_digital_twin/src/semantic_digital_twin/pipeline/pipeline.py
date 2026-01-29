import logging
import random
import re
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Callable, Dict, Type

import numpy as np

from ..adapters.mesh import STLParser
from ..semantic_annotations.mixins import HasRootKinematicStructureEntity, HasRootBody
from ..semantic_annotations.semantic_annotations import (
    Bottle,
    Apple,
    Potato,
    Orange,
    Tomato,
    Plate,
    Bowl,
    Fork,
    Knife,
    Mug,
    Cup,
    Pan,
    PanLid,
    Pencil,
    Ball,
    Baseball,
    SprayBottle,
)
from ..spatial_types import Point3
from ..spatial_types.spatial_types import HomogeneousTransformationMatrix
from ..world import World
from ..world_description.geometry import TriangleMesh, FileMesh
from ..world_description.world_entity import Body


@dataclass
class Step(ABC):
    """
    A Step is a transformation that takes a World as input and produces a modified World as output.
    Steps are intended to be used in a Pipeline, where the output World of one Step is passed as the input World to the next Step.
    """

    @abstractmethod
    def _apply(self, world: World) -> World: ...

    def apply(self, world: World) -> World:
        with world.modify_world():
            return self._apply(world)


@dataclass
class Pipeline:
    """
    A Pipeline is a sequence of Steps that are applied to a World in order.
    Each Step takes the World as input and produces a modified World as output.
    The output World of one Step is passed as the input World to the next Step.
    """

    steps: List[Step]
    """
    The list of Steps to be applied in the Pipeline.
    """

    def apply(self, world: World) -> World:
        for step in self.steps:
            world = step.apply(world)
        return world


@dataclass
class BodyFilter(Step):
    """
    Filters bodies in the world based on a given condition.
    """

    condition: Callable[[Body], bool]

    def _apply(self, world: World) -> World:
        for body in world.bodies:
            if not self.condition(body):
                world.remove_kinematic_structure_entity(body)
        return world


@dataclass
class CenterLocalGeometryAndPreserveWorldPose(Step):
    """
    Adjusts the vertices of the collision meshes of each body in the world so that the origin is at the center of the
    mesh, and then updates the parent connection of the body to preserve the original world pose.
    An example where this is useful is when parsing FBX files where all bodies in the resulting world have an origin
    at (0, 0, 0), even through the collision meshes are not centered around that point.
    """

    def _apply(self, world: World) -> World:
        for body in world.bodies_topologically_sorted:

            vertices = []

            for coll in body.collision:
                if isinstance(coll, (FileMesh, TriangleMesh)):
                    mesh = coll.mesh
                    if mesh.vertices.shape[0] > 0:
                        vertices.append(mesh.vertices.copy())

            if len(vertices) == 0:
                logging.warning(
                    f"Body {body.name.name} has no vertices in visual or collision shapes, skipping."
                )
                continue

            # Compute the axis-aligned bounding box center of all vertices
            all_vertices = np.vstack(vertices)
            mins = all_vertices.min(axis=0)
            maxs = all_vertices.max(axis=0)
            center = (mins + maxs) / 2.0

            for coll in body.collision:
                if isinstance(coll, (FileMesh, TriangleMesh)):
                    m = coll.mesh
                    if m.vertices.shape[0] > 0:
                        m.vertices -= center

            old_origin_T_new_origin = (
                HomogeneousTransformationMatrix.from_point_rotation_matrix(
                    Point3(*center)
                )
            )

            if body.parent_connection:
                parent_T_old_origin = (
                    body.parent_connection.parent_T_connection_expression
                )

                body.parent_connection.parent_T_connection_expression = (
                    parent_T_old_origin @ old_origin_T_new_origin
                )

            for child in world.compute_child_kinematic_structure_entities(body):
                old_origin_T_child_origin = (
                    child.parent_connection.parent_T_connection_expression
                )
                child.parent_connection.parent_T_connection_expression = (
                    old_origin_T_new_origin.inverse() @ old_origin_T_child_origin
                )
        return world


@dataclass
class BodyFactoryReplace(Step):
    """
    Replace bodies in the world that match a given condition with new structures created by a factory.
    """

    annotation_creator: Callable[[Body, World], HasRootKinematicStructureEntity]
    """
    A callable that takes a Body, and creates a new semantic annotation (including a new body) for it, and adds them to the world.
    """

    body_condition: Callable[[Body], bool] = lambda x: bool(
        re.compile(r"^dresser_\d+.*$").fullmatch(x.name.name)
    )
    """
    Condition to filter bodies that should be replaced. Defaults to matching bodies containing "dresser_" followed by digits in their name.
    """

    def _apply(self, world: World) -> World:
        filtered_bodies = [body for body in world.bodies if self.body_condition(body)]

        for body in filtered_bodies:
            semantic_annotation = self.annotation_creator(body, world)
            new_world = world.move_branch_to_new_world(semantic_annotation.root)
            parent_connection = body.parent_connection
            if parent_connection is None:
                return new_world

            for entity in world.get_kinematic_structure_entities_of_branch(body):
                world.remove_kinematic_structure_entity(entity)

            world.remove_kinematic_structure_entity(body)

            parent_connection.child = new_world.root
            world.merge_world(new_world, parent_connection)

        return world


@dataclass
class BodyGeometryAndAnnotation:
    object_geometry_file: str

    semantic_annotation: Type[HasRootBody]


@dataclass
class BodyGeometryAndAnnotationReplacement(Step):

    object_mappings: Dict[str, List[BodyGeometryAndAnnotation]]
    """
    A list of mappings specifying object names and their corresponding replacement geometry.
    """

    def _apply(self, world: World) -> World:
        for body in reversed(world.bodies_topologically_sorted):

            if not body.collision:
                continue

            body_name = body.name

            replacement_maps = next(
                (
                    replacement_maps
                    for object_name, replacement_maps in self.object_mappings.items()
                    if body_name.name.startswith(object_name)
                ),
                None,
            )
            if replacement_maps is None:
                continue

            replacement_map = random.choice(replacement_maps)

            parent_C_body = body.parent_connection
            print(body_name)
            print(body.collision)

            new_body_world = STLParser(
                file_path=replacement_map.object_geometry_file
            ).parse()
            with new_body_world.modify_world():
                new_semantic_annotation = replacement_map.semantic_annotation(
                    root=new_body_world.bodies[0]
                )
                new_body_world.add_semantic_annotation(new_semantic_annotation)
            with world.modify_world():
                world.remove_connection(parent_C_body)
                world.remove_kinematic_structure_entity(body)
                parent_C_body.child = new_body_world.bodies[0]
                world.merge_world(new_body_world, parent_C_body)

        return world


mapping = {
    "bottle_1": [BodyGeometryAndAnnotation("mustard_bottle.stl", Bottle)],
    "apple": [
        BodyGeometryAndAnnotation("apple.stl", Apple),
        BodyGeometryAndAnnotation("orange.stl", Orange),
    ],
    "plate": [BodyGeometryAndAnnotation("plate.stl", Plate)],
    "bowl": [BodyGeometryAndAnnotation("bowl.stl", Bowl)],
    "fork": [BodyGeometryAndAnnotation("fork.stl", Fork)],
    "robothor_butter_knife": [BodyGeometryAndAnnotation("knife.stl", Knife)],
    "mug": [BodyGeometryAndAnnotation("mug.stl", Mug)],
    "cup": [BodyGeometryAndAnnotation("cup_a.stl", Cup)],
    "pan": [BodyGeometryAndAnnotation("skillet.stl", Pan)],
    "pan_lid": [BodyGeometryAndAnnotation("skillet_lid.stl", PanLid)],
    "robothor_pencil": [
        BodyGeometryAndAnnotation("small_marker.stl", Pencil),
        BodyGeometryAndAnnotation("large_marker.stl", Pencil),
    ],
    "robothor_basketball": [
        BodyGeometryAndAnnotation("softball.stl", Ball),
        BodyGeometryAndAnnotation("baseball.stl", Baseball),
        BodyGeometryAndAnnotation("tennisball.stl", Ball),
        BodyGeometryAndAnnotation("racequetball.stl", Ball),
        BodyGeometryAndAnnotation("golfball.stl", Ball),
        BodyGeometryAndAnnotation("mini_soccerball.stl", Ball),
    ],
    "robothor_spray_bottle": [
        BodyGeometryAndAnnotation("spraybottle.stl", SprayBottle)
    ],
}
