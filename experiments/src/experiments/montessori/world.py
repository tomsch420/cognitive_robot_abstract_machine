"""
Build a semantic digital twin world for the Montessori shape-sorting scene: a table
carrying a smaller table, a shape-sorting board with holes and drawers, the loose
shapes that are dropped through the holes, and (optionally) an ICub3 robot standing in
front of it.

The scene is constructed directly with the semantic digital twin API (bodies, regions,
shapes, connections, and semantic annotations); nothing is loaded from a recorded
episode or any other external dataset.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh
from typing_extensions import List

from experiments.montessori.semantics import (
    MontessoriShape,
    MontessoriShapeCategory,
    ShapeSortingBoard,
    ShapeSortingHole,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.icub3 import ICub3
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootKinematicStructureEntity,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Drawer,
    Handle,
    SideTable,
    Table,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import (
    Box,
    Color,
    Cylinder,
    Mesh,
    Scale,
    Shape,
    Sphere,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body, Region

NAME_PREFIX = "montessori"
"""
Prefix given to every :class:`PrefixedName` created by this module.
"""

TABLE_SCALE = Scale(2.0, 1.0, 0.1)
TABLE_POSITION = Point3(0.0, 0.0, 1.0)

SMALL_TABLE_SCALE = Scale(0.5, 1.0, 0.025)
SMALL_TABLE_POSITION = Point3(-0.35, 0.0, 1.3)

BOARD_SCALE = Scale(0.13, 0.30, 0.08)
BOARD_POSITION = Point3(-0.4, 0.0, 1.353)
BOARD_COLOR = Color.BEIGE()

DRAWER_SCALE = Scale(0.09, 0.08, 0.06)
HANDLE_SCALE = Scale(0.03, 0.015, 0.015)

SHAPE_HOVER_HEIGHT = 0.05
"""
Height above its matching hole at which a loose shape is initially placed.
"""

DEFAULT_ICUB3_STANDOFF_DISTANCE = 0.6
"""
Default distance the spawned ICub3 stands in front of the Montessori table's near edge.
"""

_SHAPE_COLORS = {
    MontessoriShapeCategory.CUBE: Color.RED(),
    MontessoriShapeCategory.CYLINDER: Color.BLUE(),
    MontessoriShapeCategory.DISK: Color.YELLOW(),
    MontessoriShapeCategory.SPHERE: Color.MAGENTA(),
    MontessoriShapeCategory.TRIANGULAR_PRISM: Color.GREEN(),
    MontessoriShapeCategory.RECTANGULAR_PRISM: Color.ORANGE(),
}
"""
The color used to render a loose shape and the hole it fits through, keyed by their
shared :class:`~experiments.montessori.semantics.MontessoriShapeCategory`.
"""


@dataclass(frozen=True)
class _HoleSpec:
    """
    Layout of a single hole cut into the lid of the shape-sorting board.
    """

    key: str
    category: MontessoriShapeCategory
    position: Point3
    footprint: Scale


_HOLES: List[_HoleSpec] = [
    _HoleSpec(
        "square_hole",
        MontessoriShapeCategory.CUBE,
        Point3(-0.38, -0.087, 1.3905),
        Scale(0.03, 0.03, 0.005),
    ),
    _HoleSpec(
        "circular_hole_1",
        MontessoriShapeCategory.CYLINDER,
        Point3(-0.4215, -0.087, 1.3905),
        Scale(0.03, 0.03, 0.005),
    ),
    _HoleSpec(
        "triangle_hole",
        MontessoriShapeCategory.TRIANGULAR_PRISM,
        Point3(-0.381, 0.0, 1.3905),
        Scale(0.03, 0.03, 0.005),
    ),
    _HoleSpec(
        "rectangular_hole",
        MontessoriShapeCategory.RECTANGULAR_PRISM,
        Point3(-0.427, 0.0, 1.3905),
        Scale(0.015, 0.03, 0.005),
    ),
    _HoleSpec(
        "circular_hole_2",
        MontessoriShapeCategory.CYLINDER,
        Point3(-0.418, 0.091, 1.3905),
        Scale(0.03, 0.03, 0.005),
    ),
    _HoleSpec(
        "disk_hole",
        MontessoriShapeCategory.DISK,
        Point3(-0.375, 0.091, 1.3905),
        Scale(0.005, 0.04, 0.005),
    ),
]
"""
One hole per :class:`~experiments.montessori.semantics.MontessoriShapeCategory` that has
a matching shape (two holes, both circular, accept the
:attr:`~experiments.montessori.semantics.MontessoriShapeCategory.CYLINDER` shape); the
sphere has no matching hole, mirroring the real Montessori board this scene is modelled
after.
"""

_DRAWER_POSITIONS: List[Point3] = [
    Point3(-0.403, 0.087, 1.353),
    Point3(-0.403, 0.0, 1.353),
    Point3(-0.403, -0.087, 1.353),
]

_HANDLE_OFFSET = Point3(-0.061, 0.0, 0.001)
"""
Position of a drawer's handle relative to the drawer.
"""


def _name(name: str) -> PrefixedName:
    return PrefixedName(name, NAME_PREFIX)


def _body_with_shape(name: PrefixedName, shape: Shape) -> Body:
    """
    Build a :class:`Body` whose collision and visual geometry are a single shape.
    """
    return Body.from_shape_collection(name, ShapeCollection([shape]))


def _triangular_prism_mesh(base: float, height: float, thickness: float) -> Mesh:
    """
    Build a :class:`Mesh` shape of a triangular prism, centered on its own origin.

    :param base: Width of the triangular cross-section's base.
    :param height: Height of the triangular cross-section.
    :param thickness: Extrusion depth of the prism along its own z-axis.
    """
    triangle = np.array([[0.0, 0.0], [base, 0.0], [base / 2, height]])
    triangle -= triangle.mean(axis=0)
    prism = trimesh.creation.extrude_triangulation(
        vertices=triangle, faces=np.array([[0, 1, 2]]), height=thickness
    )
    prism.apply_translation([0.0, 0.0, -thickness / 2])
    return Mesh.from_trimesh(mesh=prism)


def _shape_body(name: PrefixedName, category: MontessoriShapeCategory) -> Body:
    """
    Build the :class:`Body` of a loose Montessori shape, its geometry depending on its
    category.
    """
    color = _SHAPE_COLORS[category]
    match category:
        case MontessoriShapeCategory.CUBE:
            shape = Box(scale=Scale(0.03, 0.03, 0.03), color=color)
        case MontessoriShapeCategory.CYLINDER:
            shape = Cylinder(width=0.03, height=0.03, color=color)
        case MontessoriShapeCategory.DISK:
            shape = Cylinder(width=0.044, height=0.004, color=color)
        case MontessoriShapeCategory.SPHERE:
            shape = Sphere(radius=0.02, color=color)
        case MontessoriShapeCategory.RECTANGULAR_PRISM:
            shape = Box(scale=Scale(0.02, 0.04, 0.03), color=color)
        case MontessoriShapeCategory.TRIANGULAR_PRISM:
            shape = _triangular_prism_mesh(base=0.03, height=0.03, thickness=0.02)
            shape.color = color
    return _body_with_shape(name, shape)


def _spawn(
    world: World,
    annotation: HasRootKinematicStructureEntity,
    position: Point3,
) -> HasRootKinematicStructureEntity:
    """
    Connect a semantic annotation's root entity to the world root at ``position`` with
    a fixed connection, and register the annotation with the world.

    :param world: The world to spawn the annotation into.
    :param annotation: The semantic annotation to spawn.
    :param position: The annotation's position, expressed in the world root frame.
    :return: The spawned annotation.
    """
    world.add_connection(
        FixedConnection(
            parent=world.root,
            child=annotation.root,
            parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=position.x, y=position.y, z=position.z
            ),
        )
    )
    world.add_semantic_annotation(annotation)
    return annotation


def _build_tables(world: World) -> None:
    table = Table(
        name=_name("table"),
        root=_body_with_shape(
            _name("table"), Box(scale=TABLE_SCALE, color=Color.GREY())
        ),
    )
    _spawn(world, table, TABLE_POSITION)

    small_table = SideTable(
        name=_name("small_table"),
        root=_body_with_shape(
            _name("small_table"), Box(scale=SMALL_TABLE_SCALE, color=BOARD_COLOR)
        ),
    )
    _spawn(world, small_table, SMALL_TABLE_POSITION)


def _build_shape_sorting_board(world: World) -> ShapeSortingBoard:
    board = ShapeSortingBoard(
        name=_name("board"),
        root=_body_with_shape(_name("board"), Box(scale=BOARD_SCALE, color=BOARD_COLOR)),
    )
    _spawn(world, board, BOARD_POSITION)

    for hole_spec in _HOLES:
        hole = ShapeSortingHole(
            name=_name(hole_spec.key),
            root=Region(
                name=_name(hole_spec.key),
                area=ShapeCollection(
                    [
                        Box(
                            scale=hole_spec.footprint,
                            color=_SHAPE_COLORS[hole_spec.category],
                        )
                    ]
                ),
            ),
            shape_category=hole_spec.category,
        )
        _spawn(world, hole, hole_spec.position)
        board.add(hole)

    for index, drawer_position in enumerate(_DRAWER_POSITIONS, start=1):
        drawer = Drawer(
            name=_name(f"drawer_{index}"),
            root=_body_with_shape(
                _name(f"drawer_{index}"), Box(scale=DRAWER_SCALE, color=BOARD_COLOR)
            ),
        )
        _spawn(world, drawer, drawer_position)
        board.add(drawer)

        handle = Handle(
            name=_name(f"drawer_{index}_handle"),
            root=_body_with_shape(
                _name(f"drawer_{index}_handle"),
                Box(scale=HANDLE_SCALE, color=Color.GREY()),
            ),
        )
        handle_position = Point3(
            drawer_position.x + _HANDLE_OFFSET.x,
            drawer_position.y + _HANDLE_OFFSET.y,
            drawer_position.z + _HANDLE_OFFSET.z,
        )
        _spawn(world, handle, handle_position)
        drawer.add(handle)

    return board


def _build_shapes(world: World) -> None:
    for hole_spec in _HOLES:
        shape_key = f"{hole_spec.key}_shape"
        shape = MontessoriShape(
            name=_name(shape_key),
            root=_shape_body(_name(shape_key), hole_spec.category),
            shape_category=hole_spec.category,
        )
        hover_position = Point3(
            hole_spec.position.x,
            hole_spec.position.y,
            hole_spec.position.z + SHAPE_HOVER_HEIGHT,
        )
        _spawn(world, shape, hover_position)

    # The sphere has no matching hole in this scene, so it is placed on the main
    # table instead of hovering over a hole.
    sphere = MontessoriShape(
        name=_name("sphere_shape"),
        root=_shape_body(_name("sphere_shape"), MontessoriShapeCategory.SPHERE),
        shape_category=MontessoriShapeCategory.SPHERE,
    )
    _spawn(world, sphere, Point3(-0.55, 0.3, 1.07))


def build_montessori_world() -> World:
    """
    Build the Montessori shape-sorting world from scratch: a table, a smaller table
    carrying a shape-sorting board (with its holes and drawers), and the loose shapes
    dropped through the holes.

    The iCub robot is not spawned by this function; use :func:`spawn_icub3` to add it
    separately.

    :return: The assembled world.
    """
    world = World()
    root = Body(name=PrefixedName(name="root", prefix="world"))
    with world.modify_world():
        world.add_kinematic_structure_entity(root)

    with world.modify_world():
        _build_tables(world)
        _build_shape_sorting_board(world)
        _build_shapes(world)

    return world


def spawn_icub3(
    world: World, standoff_distance: float = DEFAULT_ICUB3_STANDOFF_DISTANCE
) -> ICub3:
    """
    Spawn an ICub3 robot standing in front of the Montessori table, facing it.

    :param world: The world to spawn the robot into, modified in place. Must already
        contain a body named ``"table"``, as built by :func:`build_montessori_world`.
    :param standoff_distance: How far in front of the table's near edge the robot
        stands.
    :return: The spawned robot.
    """
    table_bounding_box = (
        world.get_body_by_name("table")
        .collision.as_bounding_box_collection_in_frame(world.root)
        .bounding_box()
    )
    icub_world = URDFParser.from_file(ICub3.get_ros_file_path()).parse()
    world.merge_world_at_pose(
        icub_world,
        HomogeneousTransformationMatrix.from_xyz_rpy(
            table_bounding_box.min_x - standoff_distance,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            reference_frame=world.root,
        ),
    )
    return ICub3.from_world(world)
