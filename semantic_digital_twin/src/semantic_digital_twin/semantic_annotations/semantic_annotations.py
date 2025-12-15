from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from typing import Iterable, Optional, Self, Tuple

import numpy as np
from probabilistic_model.probabilistic_circuit.rx.helper import uniform_measure_of_event
from random_events.interval import closed
from random_events.product_algebra import Event, SimpleEvent
from typing_extensions import List

from krrood.entity_query_language.entity import entity, let
from krrood.entity_query_language.quantify_entity import an
from .mixins import (
    HasBody,
    HasSupportingSurface,
    HasRegion,
    HasDrawers,
    HasDoors,
    HasHandle,
    HasCorpus,
    HasHinge,
    HasActiveConnection,
    HasRevoluteConnection,
)
from ..datastructures.prefixed_name import PrefixedName
from ..datastructures.variables import SpatialVariables
from ..exceptions import InvalidDoorDimensions
from ..reasoning.predicates import InsideOf
from ..spatial_types import Point3, TransformationMatrix, Vector3
from ..spatial_types.derivatives import DerivativeMap
from ..utils import Direction
from ..world import World
from ..world_description.connections import FixedConnection, RevoluteConnection
from ..world_description.degree_of_freedom import DegreeOfFreedom
from ..world_description.geometry import Scale
from ..world_description.shape_collection import BoundingBoxCollection
from ..world_description.world_entity import (
    SemanticAnnotation,
    Body,
    KinematicStructureEntity,
)


@dataclass(eq=False)
class IsPerceivable:
    """
    A mixin class for semantic annotations that can be perceived.
    """

    class_label: Optional[str] = field(default=None, kw_only=True)
    """
    The exact class label of the perceived object.
    """


@dataclass(eq=False)
class Handle(HasBody):

    @classmethod
    def create_with_new_body_in_world(
        cls,
        name: PrefixedName,
        world: World,
        parent: Optional[KinematicStructureEntity] = None,
        parent_T_self: Optional[TransformationMatrix] = None,
        *,
        scale: Scale = Scale(0.1, 0.02, 0.02),
        thickness: float = 0.005,
    ) -> Self:
        handle_event = cls._create_handle_geometry(scale=scale).as_composite_set()

        inner_box = cls._create_handle_geometry(
            scale=scale, thickness=thickness
        ).as_composite_set()

        handle_event -= inner_box

        handle_body = Body(name=name)
        collision = BoundingBoxCollection.from_event(
            handle_body, handle_event
        ).as_shapes()
        handle_body.collision = collision
        handle_body.visual = collision
        return cls._create_with_fixed_connection_in_world(
            world, handle_body, parent, parent_T_self
        )

    @classmethod
    def _create_handle_geometry(
        cls, scale: Scale, thickness: float = 0.0
    ) -> SimpleEvent:
        """
        Create a box event representing the handle.

        :param scale: The scale of the handle.
        :param thickness: The thickness of the handle walls.
        """

        x_interval = closed(0, scale.x - thickness)
        y_interval = closed(
            -scale.y / 2 + thickness,
            scale.y / 2 - thickness,
        )

        z_interval = closed(-scale.z / 2, scale.z / 2)

        return SimpleEvent(
            {
                SpatialVariables.x.value: x_interval,
                SpatialVariables.y.value: y_interval,
                SpatialVariables.z.value: z_interval,
            }
        )


@dataclass(eq=False)
class Fridge(
    HasCorpus,
    HasDoors,
):
    """
    A semantic annotation representing a fridge that has a door and a body.
    """

    @property
    def opening_direction(self) -> Direction:
        return Direction.NEGATIVE_X


@dataclass(eq=False)
class Aperture(HasRegion):
    """
    A semantic annotation that represents an opening in a physical entity.
    An example is like a hole in a wall that can be used to enter a room.
    """


@dataclass(eq=False)
class Hinge(HasBody, HasRevoluteConnection):
    """
    A hinge is a physical entity that connects two bodies and allows one to rotate around a fixed axis.
    """

    @classmethod
    def create_with_new_body_in_world(
        cls,
        name: PrefixedName,
        world: World,
        parent: Optional[KinematicStructureEntity] = None,
        parent_T_self: Optional[TransformationMatrix] = None,
        *,
        opening_axis: Vector3 = Vector3.Z(),
        connection_limits: Optional[
            Tuple[DerivativeMap[float], DerivativeMap[float]]
        ] = None,
        connection_multiplier: float = 1.0,
        connection_offset: float = 0.0,
    ) -> Self:
        hinge_body = Body(name=name)
        parent_world = parent._world

        if connection_limits is not None:
            if connection_limits[0].position <= connection_limits[1].position:
                raise ValueError("Upper limit must be greater than lower limit.")
        else:
            connection_limits = cls.create_default_upper_lower_limits(
                parent_T_self, opening_axis
            )

        dof = DegreeOfFreedom(
            name=PrefixedName(f"{name.name}_hinge_dof", name.prefix),
            upper_limits=connection_limits[0],
            lower_limits=connection_limits[1],
        )

        parent_world.add_degree_of_freedom(dof)
        parent_C_hinge = RevoluteConnection(
            parent=parent_world.root,
            child=hinge_body,
            parent_T_connection_expression=parent_T_self,
            multiplier=connection_multiplier,
            offset=connection_offset,
            axis=opening_axis,
            dof_id=dof.id,
        )

        parent_world.add_connection(parent_C_hinge)

        return hinge_body


@dataclass(eq=False)
class Door(HasBody, HasHandle, HasHinge):
    """
    A door is a physical entity that has covers an opening, has a movable body and a handle.
    """

    @classmethod
    def create_with_new_body_in_world(
        cls,
        name: PrefixedName,
        world: World,
        parent: Optional[KinematicStructureEntity] = None,
        parent_T_self: Optional[TransformationMatrix] = None,
        *,
        scale: Scale = Scale(0.03, 1, 2),
    ) -> Self:
        if not (scale.x < scale.y and scale.x < scale.z):
            raise InvalidDoorDimensions(scale)

        door_event = scale.to_simple_event().as_composite_set()
        door_body = Body(name=name)
        bounding_box_collection = BoundingBoxCollection.from_event(
            door_body, door_event
        )
        collision = bounding_box_collection.as_shapes()
        door_body.collision = collision
        door_body.visual = collision
        return cls._create_with_fixed_connection_in_world(
            world, door_body, parent, parent_T_self
        )


@dataclass(eq=False)
class DoubleDoor(HasDoors):
    left_door: Door
    right_door: Door


@dataclass(eq=False)
class Drawer(HasCorpus, HasHandle):

    @property
    def opening_direction(self) -> Direction:
        return Direction.Z


############################### subclasses to Furniture


@dataclass(eq=False)
class Furniture(SemanticAnnotation, ABC): ...


@dataclass(eq=False)
class Table(Furniture, HasBody):
    """
    A semantic annotation that represents a table.
    """

    def points_on_table(self, amount: int = 100) -> List[Point3]:
        """
        Get points that are on the table.

        :amount: The number of points to return.
        :returns: A list of points that are on the table.
        """
        area_of_table = BoundingBoxCollection.from_shapes(self.body.collision)
        event = area_of_table.event
        p = uniform_measure_of_event(event)
        p = p.marginal(SpatialVariables.xy)
        samples = p.sample(amount)
        z_coordinate = np.full(
            (amount, 1), max([b.max_z for b in area_of_table]) + 0.01
        )
        samples = np.concatenate((samples, z_coordinate), axis=1)
        return [Point3(*s, reference_frame=self.body) for s in samples]


@dataclass(eq=False)
class Cabinet(HasCorpus, Furniture, HasDrawers, HasDoors):
    @property
    def opening_direction(self) -> Direction:
        return Direction.NEGATIVE_X


@dataclass(eq=False)
class Dresser(HasCorpus, Furniture, HasDrawers, HasDoors):
    @property
    def opening_direction(self) -> Direction:
        return Direction.NEGATIVE_X


@dataclass(eq=False)
class Cupboard(HasCorpus, Furniture, HasDoors):
    @property
    def opening_direction(self) -> Direction:
        return Direction.NEGATIVE_X


@dataclass(eq=False)
class Wardrobe(HasCorpus, Furniture, HasDrawers, HasDoors):
    @property
    def opening_direction(self) -> Direction:
        return Direction.NEGATIVE_X


@dataclass(eq=False)
class Floor(HasSupportingSurface):

    @classmethod
    def create_with_new_body_in_world(cls, name: PrefixedName, scale: Scale) -> Self:
        """
        Create a Floor semantic annotation with a new body defined by the given scale.

        :param name: The name of the floor body.
        :param scale: The scale defining the floor polytope.
        """
        polytope = scale.to_bounding_box().get_points()
        return cls.create_with_new_body_fron_polytope(
            name=name, floor_polytope=polytope
        )

    @classmethod
    def create_with_new_body_fron_polytope(
        cls, name: PrefixedName, floor_polytope: List[Point3]
    ) -> Self:
        """
        Create a Floor semantic annotation with a new body defined by the given list of Point3.

        :param name: The name of the floor body.
        :param floor_polytope: A list of 3D points defining the floor poly
        """
        room_body = Body.from_3d_points(name=name, points_3d=floor_polytope)
        return cls(body=room_body)


@dataclass(eq=False)
class Room(SemanticAnnotation):
    """
    A semantic annotation that represents a closed area with a specific purpose
    """

    floor: Floor
    """
    The room's floor.
    """


@dataclass(eq=False)
class Wall(HasBody):

    @classmethod
    def create_with_new_body_in_world(
        cls, name: PrefixedName, scale: Scale, *args, **kwargs
    ) -> Self:
        wall_body = Body(name=name)
        wall_event = cls._create_wall_event(scale)
        wall_collision = BoundingBoxCollection.from_event(
            wall_body, wall_event
        ).as_shapes()

        wall_body.collision = wall_collision
        wall_body.visual = wall_collision

        return cls(body=wall_body, *args, **kwargs)

    @property
    def doors(self) -> Iterable[Door]:
        door = let(Door, self._world.semantic_annotations)
        query = an(entity(door), InsideOf(self.body, door.entry_way.region)() > 0.1)
        return query.evaluate()

    @classmethod
    def _create_wall_event(cls, scale: Scale) -> Event:
        """
        Return a wall event created from its scale. The height origin is on the ground, not in the center of the wall.
        """
        x_interval = closed(-scale.x / 2, scale.x / 2)
        y_interval = closed(-scale.y / 2, scale.y / 2)
        z_interval = closed(0, scale.z)

        wall_event = SimpleEvent(
            {
                SpatialVariables.x.value: x_interval,
                SpatialVariables.y.value: y_interval,
                SpatialVariables.z.value: z_interval,
            }
        )
        return wall_event.as_composite_set()
