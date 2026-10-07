"""
The scene of the physical pick-up experiment: a table with an object standing on it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import trimesh

from experiments.physical_pick_up.objects import (
    ObjectCannotHaveAHandleError,
    ObjectDescription,
    ObjectGeometry,
    PickUpObject,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.mixins import (
    HasGraspCandidates,
    HasHandle,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
    Table,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    FixedConnection,
)
from semantic_digital_twin.world_description.geometry import Box, Color, Mesh, Scale
from semantic_digital_twin.world_description.inertial_properties import (
    Inertial,
    InertiaTensor,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body


@dataclass
class ObjectOnTableScene:
    """
    A world holding a table and an object that stands loose on it, ready for a robot to
    be spawned at the world's origin, facing the table along the x-axis.

    The object is a free body: nothing but contact holds it on the table, and nothing but
    contact can lift it off.
    """

    object_description: ObjectDescription = field(
        default_factory=lambda: PickUpObject.BOWL.value
    )
    """
    The object standing on the table.
    """

    floor_scale: Scale = field(default_factory=lambda: Scale(4.0, 4.0, 0.1))
    """
    Length, width and thickness of the floor.
    """

    table_scale: Scale = field(default_factory=lambda: Scale(0.6, 1.0, 0.72))
    """
    Depth, width and height of the table.
    """

    table_position: Point3 = field(default_factory=lambda: Point3(0.85, 0.0, 0.0))
    """
    Where the middle of the table stands on the floor.
    """

    object_position: Point3 = field(default_factory=lambda: Point3(0.68, 0.15, 0.0))
    """
    Where on the table the middle of the object stands; only x and y are read, the
    object is put down on the table top.
    """

    drop_height: float = 0.002
    """
    How far above the table top the object starts, so that it is not spawned in contact.
    """

    world: World = field(init=False)
    """
    The world holding the table and the object.
    """

    table: Table = field(init=False)
    """
    The table.
    """

    graspable: HasGraspCandidates = field(init=False)
    """
    The object, as what :attr:`object_description` says it is.
    """

    def __post_init__(self):
        self.world = World()
        with self.world.modify_world():
            self.world.add_kinematic_structure_entity(self._floor())
        self.table = self._add_table()
        self.graspable = self._add_object()

    def _floor(self) -> Body:
        """
        :return: The floor, a slab whose top is at height zero, so that an object
            dropped off the table comes to rest on it.
        """
        body = Body(name=PrefixedName("floor"))
        slab = Box(
            origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                z=-self.floor_scale.z / 2, reference_frame=body
            ),
            scale=self.floor_scale,
            color=Color(0.3, 0.3, 0.3, 1.0),
        )
        body.collision = ShapeCollection([slab], reference_frame=body)
        body.visual = ShapeCollection([slab], reference_frame=body)
        return body

    @property
    def table_top_height(self) -> float:
        """
        :return: The height of the table's surface above the floor.
        """
        return self.table_scale.z

    def _add_table(self) -> Table:
        """
        :return: The table, a box standing fixed on the floor.
        """
        body = Body(name=PrefixedName("table"))
        box = Box(
            origin=HomogeneousTransformationMatrix.from_xyz_rpy(reference_frame=body),
            scale=self.table_scale,
            color=Color(0.6, 0.45, 0.3, 1.0),
        )
        body.collision = ShapeCollection([box], reference_frame=body)
        body.visual = ShapeCollection([box], reference_frame=body)
        table = Table(root=body)
        x, y, _ = self.table_position.to_np()[:3]
        with self.world.modify_world():
            self.world.add_connection(
                FixedConnection(
                    parent=self.world.root,
                    child=body,
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=x,
                        y=y,
                        z=self.table_top_height / 2,
                        reference_frame=self.world.root,
                    ),
                )
            )
            self.world.add_semantic_annotation(table)
        return table

    def _add_object(self) -> HasGraspCandidates:
        """
        :return: The object, a free body resting on the table with the middle of its
            footprint at :attr:`object_position`.
        """
        description = self.object_description
        body = Body(name=PrefixedName(description.body_name))
        geometry = description.load_geometry()
        body.visual = ShapeCollection(
            [self._shape(geometry.visual, body)], reference_frame=body
        )
        body.collision = ShapeCollection(
            [self._shape(part, body) for part in geometry.collision_parts],
            reference_frame=body,
        )
        body.inertial = self._object_inertial(body)
        middle = geometry.visual.bounds.mean(axis=0)
        lowest = body.collision.combined_mesh.bounds[0]
        graspable = description.semantic_annotation_type(root=body)
        x, y, _ = self.object_position.to_np()[:3]
        with self.world.modify_world():
            connection = Connection6DoF.create_with_dofs(
                parent=self.world.root, child=body, world=self.world
            )
            self.world.add_connection(connection)
            connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
                x=x - middle[0],
                y=y - middle[1],
                z=self.table_top_height - lowest[2] + self.drop_height,
                reference_frame=self.world.root,
            )
            self.world.add_semantic_annotation(graspable)
        self._add_handle(graspable, geometry)
        return graspable

    def _add_handle(
        self, graspable: HasGraspCandidates, geometry: ObjectGeometry
    ) -> None:
        """
        Give the object the handle its description's handle finder finds in its shape,
        if it finds one.

        :param graspable: The object's annotation.
        :param geometry: The object's geometry in its body's frame.
        :raises ObjectCannotHaveAHandleError: If the description finds handles for an
            object whose annotation cannot have one.
        """
        finder = self.object_description.handle_finder
        if finder is None:
            return
        if not isinstance(graspable, HasHandle):
            raise ObjectCannotHaveAHandleError(
                object_description=self.object_description
            )
        shape = finder.find(geometry.visual)
        if shape is None:
            return
        Handle.create_from_part_of_shape(graspable, shape)

    @staticmethod
    def _shape(mesh: trimesh.Trimesh, body: Body) -> Mesh:
        """
        :param mesh: A mesh in the frame of ``body``.
        :param body: The body the shape belongs to.
        :return: The mesh as a shape of ``body``.
        """
        return Mesh.from_trimesh(
            mesh=mesh, origin=HomogeneousTransformationMatrix(reference_frame=body)
        )

    def _object_inertial(self, body: Body) -> Inertial:
        """
        :param body: The object's body, with its collision shapes in place.
        :return: The inertial properties of an object of the described mass whose
            material is spread evenly over what it collides as: the convex hull of each
            collision shape.
        """
        mass = self.object_description.mass
        material = trimesh.util.concatenate(
            [shape.mesh.convex_hull for shape in body.collision.shapes]
        )
        material.density = mass / material.volume
        return Inertial(
            mass=mass,
            center_of_mass=Point3.from_iterable(
                material.center_mass, reference_frame=body
            ),
            inertia=InertiaTensor(data=material.moment_inertia),
        )
