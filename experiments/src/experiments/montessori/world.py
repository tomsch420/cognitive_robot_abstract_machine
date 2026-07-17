"""
Build a semantic digital twin world for the Montessori shape-sorting scene: a floor
carrying a table with a shape-sorting board (with its holes and drawers) and the loose
shapes that are dropped through the holes, and (optionally) an HSRB robot standing in
front of it.

The scene is constructed directly with the semantic digital twin API (bodies, regions,
shapes, connections, and semantic annotations); nothing is loaded from a recorded
episode or any other external dataset.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh
from typing_extensions import List, Optional

from experiments.montessori.hole_geometry import (
    HOLE_MARKER_THICKNESS,
    HoleFootprint,
    cut_board_mesh,
    detect_hole_footprints,
)
from experiments.montessori.semantics import (
    MontessoriShape,
    MontessoriShapeCategory,
    ShapeSortingBoard,
    ShapeSortingHole,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.pipeline.mesh_decomposition.coacd import (
    ApproximationMode,
    COACDMeshDecomposer,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootKinematicStructureEntity,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Drawer,
    Floor,
    Handle,
    Table,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    OmniDrive,
)
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

FLOOR_Z = 0.0
"""
Height of the ground the Montessori scene stands on.
"""

FLOOR_SCALE = Scale(6.0, 4.0, 0.02)
"""
Size of the floor slab; large enough to comfortably fit the table, the row of loose
shapes, and a standing HSRB.
"""

TABLE_SCALE = Scale(0.5, 1.0, 0.025)
TABLE_POSITION = Point3(-0.35, 0.0, 0.5)

TABLE_LEG_FOOTPRINT = 0.05
"""
Cross-sectional width/depth of each of the table's four legs.
"""

_LEG_CORNER_SIGNS = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
"""
The four ``(x, y)`` sign combinations at which a table's legs are placed, relative to
its center.
"""

BOARD_SCALE = Scale(0.13, 0.30, 0.08)
BOARD_POSITION = Point3(-0.4, 0.0, 0.553)
BOARD_COLOR = Color.BEIGE()

DRAWER_SCALE = Scale(0.09, 0.08, 0.06)
HANDLE_SCALE = Scale(0.03, 0.015, 0.015)

TABLE_SHAPE_ROW_X = -0.15
"""
X-coordinate, in the world frame, of the row in which loose shapes are placed on the
table; clear of the shape-sorting board's footprint.
"""

TABLE_SHAPE_ROW_START_Y = -0.45
"""
Y-coordinate of the first loose shape in the row on the table.
"""

TABLE_SHAPE_ROW_SPACING = 0.15
"""
Distance, along y, between adjacent loose shapes in the row on the table.
"""

DEFAULT_HSRB_STANDOFF_DISTANCE = 0.6
"""
Default distance the spawned HSRB stands in front of the Montessori table's near edge.
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
    shape: HoleFootprint
    """
    The hole's true, mesh-detected shape; used to build both the board's cut and this
    hole's own marker region.
    """


_HOLE_KEY_BY_CATEGORY = {
    MontessoriShapeCategory.CUBE: "square_hole",
    MontessoriShapeCategory.TRIANGULAR_PRISM: "triangle_hole",
    MontessoriShapeCategory.RECTANGULAR_PRISM: "rectangular_hole",
    MontessoriShapeCategory.DISK: "disk_hole",
}
"""
Name given to a hole of a given category, for the categories that occur at most once on
the board.

The :attr:`~MontessoriShapeCategory.CYLINDER` category occurs twice and is numbered
instead (``circular_hole_1``, ``circular_hole_2``).
"""


def _hole_spec_from_footprint(footprint: HoleFootprint, key: str) -> _HoleSpec:
    """
    Place a mesh-detected :class:`HoleFootprint` onto the board, flush with its top
    surface, and pair it with a semantic key.
    """
    position = Point3(
        BOARD_POSITION.x + footprint.center[0],
        BOARD_POSITION.y + footprint.center[1],
        BOARD_POSITION.z + BOARD_SCALE.z / 2 - HOLE_MARKER_THICKNESS / 2,
    )
    return _HoleSpec(key, footprint.category, position, footprint)


def _build_hole_specs(footprints: List[HoleFootprint]) -> List[_HoleSpec]:
    """
    Build the board's hole specifications from its mesh-detected hole shapes.
    """
    circular_hole_count = 0
    hole_specs = []
    for footprint in footprints:
        if footprint.category is MontessoriShapeCategory.CYLINDER:
            circular_hole_count += 1
            key = f"circular_hole_{circular_hole_count}"
        else:
            key = _HOLE_KEY_BY_CATEGORY[footprint.category]
        hole_specs.append(_hole_spec_from_footprint(footprint, key))
    return hole_specs


_HOLE_FOOTPRINTS: List[HoleFootprint] = detect_hole_footprints()
"""
The board's hole shapes, detected once from its mesh
(:func:`~experiments.montessori.hole_geometry.detect_hole_footprints`); the single
source both :const:`_HOLES` and the board's cut mesh are built from.
"""

_HOLES: List[_HoleSpec] = _build_hole_specs(_HOLE_FOOTPRINTS)
"""
One hole per :class:`~experiments.montessori.semantics.MontessoriShapeCategory` that has
a matching shape (two holes, both circular, accept the
:attr:`~experiments.montessori.semantics.MontessoriShapeCategory.CYLINDER` shape); the
sphere has no matching hole, mirroring the real Montessori board this scene is modelled
after.

Detected from the board's mesh by
:func:`~experiments.montessori.hole_geometry.detect_hole_footprints`.
"""

_BOARD_MESH: trimesh.Trimesh = cut_board_mesh(BOARD_SCALE, _HOLE_FOOTPRINTS)
"""
The shape-sorting board's mesh, with all of :const:`_HOLE_FOOTPRINTS` cut fully through
it.

Used directly as the board's visual geometry.
"""

_BOARD_COLLISION_DECOMPOSER = COACDMeshDecomposer(
    threshold=0.01, approximation_mode=ApproximationMode.CONVEX_HULL
)
"""
Decomposes :const:`_BOARD_MESH` into convex sub-pieces for collision purposes.

Physics engines that only support convex collision geometry (MuJoCo among them) collide
mesh geometry against its convex hull, which would silently fill in the board's cut
holes if the un-decomposed, concave mesh were used directly as collision geometry; a
convex decomposition keeps each piece individually convex while the pieces together
still leave the holes open.

CoACD was chosen over V-HACD after measuring both: ray-casting a grid of points across
each hole's opening after decomposition (a shape can only fall through if the whole
opening, not just its center, is collision-free) showed CoACD leaving substantially more
of each hole open. ``threshold=0.01`` is CoACD's minimum (finest) supported value, and
``ApproximationMode.CONVEX_HULL`` (true convex hulls) leaves holes open more reliably
than CoACD's default ``ApproximationMode.BOX``.

Even so, this remains an approximation: the smallest hole (for the thin ``disk``
category) is only partially open, since a general-purpose decomposer optimizing for
volume tends to smooth over narrow slot-shaped features. A shape dropped through one of
the other, larger holes settles noticeably closer to (though, at the time of writing,
not yet fully through) the hole, confirmed by physically simulating the drop in MuJoCo.
"""

MINIMUM_COLLISION_PART_VOLUME = 1e-9
"""
Volume (in cubic meters) below which a convex decomposition part is treated as a
degenerate, zero-thickness sliver and dropped.

V-HACD occasionally emits perfectly flat (2-dimensional) pieces along a decomposition
seam; MuJoCo's mesh compiler cannot build a convex hull from a flat point set and fails
to load the model. Such pieces contribute no real collision volume, so removing them is
safe.
"""

_BOARD_COLLISION_PARTS: List[trimesh.Trimesh] = [
    part.mesh
    for part in _BOARD_COLLISION_DECOMPOSER.apply_to_mesh(Mesh.from_trimesh(mesh=_BOARD_MESH))
    if part.mesh.volume > MINIMUM_COLLISION_PART_VOLUME
]
"""
Convex decomposition of :const:`_BOARD_MESH`, computed once at import time rather than
per :class:`MontessoriWorld`.

Each :class:`MontessoriWorld` wraps these in its own fresh :class:`Mesh` shapes:
:class:`ShapeCollection` transforms a shape's origin into its owning body's frame in
place, so the same ``Mesh`` instance must not be shared between worlds.
"""

_DRAWER_POSITIONS: List[Point3] = [
    Point3(-0.403, 0.087, 0.553),
    Point3(-0.403, 0.0, 0.553),
    Point3(-0.403, -0.087, 0.553),
]

_HANDLE_OFFSET = Point3(-0.061, 0.0, 0.001)
"""
Position of a drawer's handle relative to the drawer.
"""


def _name(name: str) -> PrefixedName:
    return PrefixedName(name, NAME_PREFIX)


def _body_with_shapes(name: PrefixedName, shapes: List[Shape]) -> Body:
    """
    Build a :class:`Body` whose collision and visual geometry are the given shapes.
    """
    return Body.from_shape_collection(name, ShapeCollection(shapes))


def _body_with_shape(name: PrefixedName, shape: Shape) -> Body:
    """
    Build a :class:`Body` whose collision and visual geometry are a single shape.
    """
    return _body_with_shapes(name, [shape])


def _body_with_visual_only_shape(name: PrefixedName, shape: Shape) -> Body:
    """
    Build a :class:`Body` with a visual-only shape and no collision geometry.

    Used for the floor: CRAM's navigation reachability costmaps assume the ground
    itself has no collision mesh, and treat any collidable geometry at ground level
    as an obstacle blocking every standing spot.
    """
    return Body(name=name, visual=ShapeCollection([shape]))


def _board_body(name: PrefixedName, board_shape: Mesh) -> Body:
    """
    Build the shape-sorting board's :class:`Body`: ``board_shape`` as visual geometry,
    and a convex decomposition of the board mesh (:const:`_BOARD_COLLISION_PARTS`) as
    collision geometry, so the cut holes stay physically open to physics engines that
    only support convex collision geometry.

    :param name: Name of the body.
    :param board_shape: The board's (single, concave) visual mesh shape.
    """
    return Body(
        name=name,
        visual=ShapeCollection([board_shape]),
        collision=ShapeCollection(
            [Mesh.from_trimesh(mesh=part) for part in _BOARD_COLLISION_PARTS]
        ),
    )


def _table_shapes(
    table_scale: Scale,
    table_center_z: float,
    leg_footprint: float,
    support_z: float,
    color: Color,
) -> List[Shape]:
    """
    Build a tabletop :class:`Box` plus four leg :class:`Box`\\ es that support it from
    ``support_z`` up to its underside, at its four corners.

    :param table_scale: Size of the tabletop.
    :param table_center_z: Height of the tabletop's own center, in the world frame.
    :param leg_footprint: Cross-sectional width/depth of each leg.
    :param support_z: Height of the surface the legs stand on.
    :param color: Color shared by the tabletop and its legs.
    :return: The tabletop shape followed by its four leg shapes, positioned relative to
        the tabletop's own origin.
    """
    half_x = table_scale.x / 2 - leg_footprint / 2
    half_y = table_scale.y / 2 - leg_footprint / 2
    leg_height = table_center_z - table_scale.z / 2 - support_z
    leg_center_local_z = support_z + leg_height / 2 - table_center_z

    shapes = [Box(scale=table_scale, color=color)]
    for sign_x, sign_y in _LEG_CORNER_SIGNS:
        leg_origin = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=sign_x * half_x, y=sign_y * half_y, z=leg_center_local_z
        )
        shapes.append(
            Box(
                scale=Scale(leg_footprint, leg_footprint, leg_height),
                color=color,
                origin=leg_origin,
            )
        )
    return shapes


SHAPE_FOOTPRINT_CLEARANCE_SCALE = 0.85
"""
In-plane scale applied to a hole's true footprint when deriving the matching loose
shape's cross-section from it, so the shape is a smaller copy of the hole rather than an
exact fit, leaving clearance to actually pass through.
"""


def _footprint_shape_mesh(
    footprint: HoleFootprint, thickness: float, color: Color
) -> Mesh:
    """
    Build a solid :class:`Mesh` whose cross-section is a clearance-scaled copy of a
    hole's true footprint, centered on its own origin.

    Deriving the shape from the same :class:`HoleFootprint` its hole is cut from (rather
    than independently hand-authoring a same-category shape, as done for a hole's
    non-rectangular category, e.g. :attr:`MontessoriShapeCategory.TRIANGULAR_PRISM`)
    keeps the two in the same local orientation by construction, since both are read
    from one detected outline instead of risking two independent authors picking
    different reference orientations for the same nominal shape.

    :param footprint: The hole this shape is sized and oriented after.
    :param thickness: Extrusion depth of the solid along its own z-axis.
    :param color: Color of the resulting shape.
    """
    scale = SHAPE_FOOTPRINT_CLEARANCE_SCALE
    solid = footprint.extrude(thickness)
    solid.apply_transform(np.diag([scale, scale, 1.0, 1.0]))
    mesh = Mesh.from_trimesh(mesh=solid)
    mesh.color = color
    return mesh


def _hole_marker_shape(footprint: HoleFootprint, color: Color) -> Mesh:
    """
    Build a thin :class:`Mesh` matching a hole's true cross-section shape, for its
    :class:`~experiments.montessori.semantics.ShapeSortingHole` region.
    """
    marker = Mesh.from_trimesh(mesh=footprint.extrude(HOLE_MARKER_THICKNESS))
    marker.color = color
    return marker


def _shape_body(
    name: PrefixedName,
    category: MontessoriShapeCategory,
    footprint: Optional[HoleFootprint],
) -> Body:
    """
    Build the :class:`Body` of a loose Montessori shape, its geometry depending on its
    category.

    :param footprint: The footprint of the hole this shape is meant to be dropped
        through, used to derive the shape's cross-section for categories whose fit
        depends on orientation (see :func:`_footprint_shape_mesh`). Unused (and may be
        ``None``) for every other category: cube, cylinder, and disk look the same from
        any yaw, and the sphere has no hole at all.
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
            shape = _footprint_shape_mesh(footprint, thickness=0.03, color=color)
        case MontessoriShapeCategory.TRIANGULAR_PRISM:
            shape = _footprint_shape_mesh(footprint, thickness=0.02, color=color)
    return _body_with_shape(name, shape)


@dataclass(eq=False)
class MontessoriWorld:
    """
    The Montessori shape-sorting scene: a semantic digital twin world containing a
    floor, a table carrying a shape-sorting board (with its holes and drawers), and the
    loose shapes dropped through the holes.

    The scene is built as soon as an instance is constructed; use :meth:`spawn_hsrb`
    afterwards to add the HSRB robot, since it is not spawned by default.
    """

    world: World = field(init=False, default_factory=World)
    """
    The assembled semantic digital twin world.
    """

    board: ShapeSortingBoard = field(init=False)
    """
    The shape-sorting board spawned into :attr:`world`.
    """

    hsrb: Optional[HSRB] = field(init=False, default=None)
    """
    The HSRB robot spawned into :attr:`world` by :meth:`spawn_hsrb`, or ``None`` if it
    has not been spawned.
    """

    def __post_init__(self) -> None:
        root = Body(name=PrefixedName(name="root", prefix="world"))
        with self.world.modify_world():
            self.world.add_kinematic_structure_entity(root)

        with self.world.modify_world():
            self._build_floor_and_table()
            self.board = self._build_shape_sorting_board()
            self._build_shapes()

    def spawn_hsrb(
        self, standoff_distance: float = DEFAULT_HSRB_STANDOFF_DISTANCE
    ) -> HSRB:
        """
        Spawn an HSRB robot standing in front of the Montessori table, facing it, and
        store it as :attr:`hsrb`.

        Attached via an :class:`OmniDrive` (a real, hardware-controlled mobile-base
        connection), not a plain 6DoF join: CRAM's motion planner needs a proper drive
        connection to navigate the robot at all.

        :param standoff_distance: How far in front of the table's near edge the robot
            stands.
        :return: The spawned robot.
        """
        table_bounding_box = (
            self.world.get_body_by_name("table")
            .collision.as_bounding_box_collection_in_frame(self.world.root)
            .bounding_box()
        )
        hsrb_world = URDFParser.from_file(HSRB.get_ros_file_path()).parse()
        with self.world.modify_world():
            drive = OmniDrive.create_with_dofs(
                parent=self.world.root, child=hsrb_world.root, world=self.world
            )
            self.world.merge_world(hsrb_world, drive)
            drive.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
                table_bounding_box.min_x - standoff_distance,
                0.0,
                0.0,
                reference_frame=self.world.root,
            )
        self.hsrb = HSRB.from_world(self.world)
        return self.hsrb

    def _spawn(
        self,
        annotation: HasRootKinematicStructureEntity,
        position: Point3,
    ) -> HasRootKinematicStructureEntity:
        """
        Connect a semantic annotation's root entity to the world root at ``position``
        with a fixed connection, and register the annotation with the world.

        :param annotation: The semantic annotation to spawn.
        :param position: The annotation's position, expressed in the world root frame.
        :return: The spawned annotation.
        """
        self.world.add_connection(
            FixedConnection(
                parent=self.world.root,
                child=annotation.root,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=position.x, y=position.y, z=position.z
                ),
            )
        )
        self.world.add_semantic_annotation(annotation)
        return annotation

    def _build_floor_and_table(self) -> None:
        floor = Floor(
            name=_name("floor"),
            root=_body_with_visual_only_shape(
                _name("floor"), Box(scale=FLOOR_SCALE, color=Color.GREY())
            ),
        )
        self._spawn(floor, Point3(0.0, 0.0, FLOOR_Z - FLOOR_SCALE.z / 2))

        table = Table(
            name=_name("table"),
            root=_body_with_shapes(
                _name("table"),
                _table_shapes(
                    TABLE_SCALE,
                    float(TABLE_POSITION.z),
                    TABLE_LEG_FOOTPRINT,
                    FLOOR_Z,
                    BOARD_COLOR,
                ),
            ),
        )
        self._spawn(table, TABLE_POSITION)

    def _build_shape_sorting_board(self) -> ShapeSortingBoard:
        board_shape = Mesh.from_trimesh(mesh=_BOARD_MESH)
        board_shape.color = BOARD_COLOR
        board = ShapeSortingBoard(
            name=_name("board"),
            root=_board_body(_name("board"), board_shape),
        )
        self._spawn(board, BOARD_POSITION)

        for hole_spec in _HOLES:
            hole = ShapeSortingHole(
                name=_name(hole_spec.key),
                root=Region(
                    name=_name(hole_spec.key),
                    area=ShapeCollection(
                        [
                            _hole_marker_shape(
                                hole_spec.shape, _SHAPE_COLORS[hole_spec.category]
                            )
                        ]
                    ),
                ),
                shape_category=hole_spec.category,
            )
            self._spawn(hole, hole_spec.position)
            board.add(hole)

        for index, drawer_position in enumerate(_DRAWER_POSITIONS, start=1):
            drawer = Drawer(
                name=_name(f"drawer_{index}"),
                root=_body_with_shape(
                    _name(f"drawer_{index}"), Box(scale=DRAWER_SCALE, color=BOARD_COLOR)
                ),
            )
            self._spawn(drawer, drawer_position)
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
            self._spawn(handle, handle_position)
            drawer.add(handle)

        return board

    def _build_shapes(self) -> None:
        categories = [hole_spec.category for hole_spec in _HOLES] + [
            MontessoriShapeCategory.SPHERE
        ]
        keys = [hole_spec.key for hole_spec in _HOLES] + ["sphere"]
        footprints = [hole_spec.shape for hole_spec in _HOLES] + [None]

        for index, (key, category, footprint) in enumerate(
            zip(keys, categories, footprints)
        ):
            shape_key = f"{key}_shape"
            body = _shape_body(_name(shape_key), category, footprint)
            shape = MontessoriShape(
                name=_name(shape_key), root=body, shape_category=category
            )
            y = TABLE_SHAPE_ROW_START_Y + index * TABLE_SHAPE_ROW_SPACING
            self._spawn(shape, self._resting_position_on_table(body, y))

    @staticmethod
    def _resting_position_on_table(body: Body, y: float) -> Point3:
        """
        Position, at ``y`` along :const:`TABLE_SHAPE_ROW_X`, at which ``body`` rests
        exactly on the table's surface, given its own local geometry.
        """
        lowest_local_z = body.collision.combined_mesh.bounds[0][2]
        table_top_z = float(TABLE_POSITION.z) + TABLE_SCALE.z / 2
        return Point3(TABLE_SHAPE_ROW_X, y, table_top_z - lowest_local_z)
