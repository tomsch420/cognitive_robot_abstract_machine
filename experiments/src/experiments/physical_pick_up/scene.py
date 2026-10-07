"""
The scene of the physical pick-up experiment: a robot and an object standing where the
robot picks objects up.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh

from experiments.physical_pick_up.robots import (
    ObjectPlacement,
    PickUpArea,
    PR2Setup,
    RobotSetup,
)
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
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Color, Mesh, Scale
from semantic_digital_twin.world_description.inertial_properties import (
    Inertial,
    InertiaTensor,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body


@dataclass
class PickUpScene:
    """
    A world holding a robot and an object that stands loose where the robot picks
    objects up.

    The object is a free body: nothing but contact holds it where it stands, and nothing
    but contact can lift it off.
    """

    object_description: ObjectDescription = field(
        default_factory=lambda: PickUpObject.BOWL.value
    )
    """
    The object to pick up.
    """

    robot_setup: RobotSetup = field(default_factory=PR2Setup)
    """
    The robot that picks the object up, and where the object stands for it.
    """

    floor_scale: Scale = field(default_factory=lambda: Scale(4.0, 4.0, 0.1))
    """
    Length, width and thickness of the floor.
    """

    drop_height: float = 0.002
    """
    How far above its surface the object starts, so that it is not spawned in contact.
    """

    world: World = field(init=False)
    """
    The world holding the robot and the object.
    """

    robot: AbstractRobot = field(init=False)
    """
    The robot, prepared for physical simulation.
    """

    pick_up_area: PickUpArea = field(init=False)
    """
    Where the object stands for the robot to pick it up.
    """

    graspable: HasGraspCandidates = field(init=False)
    """
    The object, as what :attr:`object_description` says it is.
    """

    _footprint_middle: np.ndarray = field(init=False)
    """
    The middle of the object's footprint in its own frame, as x and y.
    """

    _lowest_point: float = field(init=False)
    """
    The height of the object's lowest point in its own frame.
    """

    def __post_init__(self):
        self.world = World()
        with self.world.modify_world():
            self.world.add_kinematic_structure_entity(self._floor())
        self.robot = self.robot_setup.spawn(self.world)
        self.pick_up_area = self.robot_setup.add_pick_up_area(self.world, self.robot)
        self.graspable = self._add_object()
        self.place_object(self.pick_up_area.middle())

    @property
    def arm(self) -> Arm:
        """
        :return: The arm that picks the object up.
        """
        return self.robot_setup.arm(self.robot)

    def place_object(self, placement: ObjectPlacement) -> None:
        """
        Put the object down where ``placement`` says.

        :param placement: Where the object is to stand.
        """
        middle = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=-self._footprint_middle[0], y=-self._footprint_middle[1]
        ).to_np()
        world_T_object = (
            HomogeneousTransformationMatrix.from_xyz_rpy(
                x=placement.x,
                y=placement.y,
                z=self.pick_up_area.height - self._lowest_point + self.drop_height,
                yaw=placement.yaw,
            ).to_np()
            @ middle
        )
        connection = self.graspable.root.parent_connection
        connection.origin = HomogeneousTransformationMatrix(
            world_T_object, reference_frame=self.world.root
        )

    def _floor(self) -> Body:
        """
        :return: The floor, a slab whose top is at height zero, so that an object
            dropped off its surface comes to rest on it.
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

    def _add_object(self) -> HasGraspCandidates:
        """
        :return: The object, a free body, not yet placed.
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
        self._footprint_middle = geometry.visual.bounds.mean(axis=0)[:2]
        self._lowest_point = float(body.collision.combined_mesh.bounds[0][2])
        graspable = description.semantic_annotation_type(root=body)
        with self.world.modify_world():
            connection = Connection6DoF.create_with_dofs(
                parent=self.world.root, child=body, world=self.world
            )
            self.world.add_connection(connection)
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
