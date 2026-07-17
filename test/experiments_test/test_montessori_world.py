import numpy as np
import pytest
from trimesh import Trimesh

from experiments.montessori.semantics import (
    MontessoriShape,
    MontessoriShapeCategory,
    ShapeSortingBoard,
    ShapeSortingHole,
)
from experiments.montessori.world import (
    BOARD_SCALE,
    FLOOR_Z,
    SHAPE_FOOTPRINT_CLEARANCE_SCALE,
    TABLE_POSITION,
    TABLE_SCALE,
    TABLE_SHAPE_ROW_X,
    MontessoriWorld,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor, Table
from semantic_digital_twin.utils import hsrb_installed


def _sorted_xy(mesh: Trimesh) -> np.ndarray:
    """
    A mesh's vertices, projected to (x, y) and sorted into a canonical order, so two
    meshes built from the same footprint can be compared regardless of internal vertex
    ordering.
    """
    xy = np.round(mesh.vertices[:, :2], 6)
    return xy[np.lexsort((xy[:, 1], xy[:, 0]))]


def test_montessori_world_creates_one_board_with_holes_and_drawers():
    montessori = MontessoriWorld()

    [board] = montessori.world.get_semantic_annotations_by_type(ShapeSortingBoard)
    assert board is montessori.board
    assert len(board.apertures) == 6
    assert len(board.drawers) == 3
    assert all(drawer.handle is not None for drawer in board.drawers)


def test_montessori_world_cuts_a_watertight_board_mesh():
    montessori = MontessoriWorld()
    montessori.world.update_forward_kinematics()

    # The board's *visual* geometry is the single, precisely cut mesh; its *collision*
    # geometry is a convex decomposition of that mesh (see
    # test_montessori_world_board_collision_is_a_convex_decomposition), which is only
    # an approximation and does not preserve the exact cut volume.
    board_mesh = montessori.board.root.visual.combined_mesh

    assert board_mesh.is_watertight
    # cutting 6 holes must strictly shrink the solid board's volume
    uncut_volume = BOARD_SCALE.x * BOARD_SCALE.y * BOARD_SCALE.z
    assert 0 < board_mesh.volume < uncut_volume


def test_montessori_world_board_collision_is_a_convex_decomposition():
    montessori = MontessoriWorld()
    montessori.world.update_forward_kinematics()

    board_collision_shapes = montessori.board.root.collision.shapes

    # MuJoCo (and other physics engines) collide mesh geometry against its convex hull,
    # which would silently fill in the board's cut holes if the single, concave visual
    # mesh were used directly as collision geometry.
    assert len(board_collision_shapes) > 1
    assert all(shape.mesh.is_convex for shape in board_collision_shapes)


def test_montessori_world_creates_one_shape_per_hole_category_plus_the_sphere():
    montessori = MontessoriWorld()

    holes = montessori.world.get_semantic_annotations_by_type(ShapeSortingHole)
    hole_categories = {hole.shape_category for hole in holes}
    shape_categories = [
        shape.shape_category
        for shape in montessori.world.get_semantic_annotations_by_type(MontessoriShape)
    ]

    # one shape per hole (some categories repeat, e.g. the two circular holes), plus
    # the sphere, which has no matching hole
    assert set(shape_categories) == hole_categories | {MontessoriShapeCategory.SPHERE}
    assert len(shape_categories) == len(holes) + 1


@pytest.mark.parametrize(
    "category",
    [
        MontessoriShapeCategory.TRIANGULAR_PRISM,
        MontessoriShapeCategory.RECTANGULAR_PRISM,
    ],
)
def test_orientation_sensitive_shape_matches_its_holes_footprint_orientation(category):
    montessori = MontessoriWorld()
    montessori.world.update_forward_kinematics()

    [shape] = [
        shape
        for shape in montessori.world.get_semantic_annotations_by_type(MontessoriShape)
        if shape.shape_category == category
    ]
    hole = montessori.board.hole_for(shape)

    # a shape whose fit through its hole depends on rotation (unlike e.g. a cube or a
    # cylinder) must be a scaled, same-orientation copy of that hole's true footprint,
    # not an independently authored shape that merely shares its category: scaling the
    # shape's cross-section back up must recover the hole's own footprint, not some
    # rotation of it.
    shape_xy = _sorted_xy(shape.root.collision.combined_mesh)
    hole_xy = _sorted_xy(hole.root.area.combined_mesh)

    assert shape_xy.shape == hole_xy.shape
    np.testing.assert_allclose(
        shape_xy / SHAPE_FOOTPRINT_CLEARANCE_SCALE, hole_xy, atol=1e-4
    )


def test_montessori_world_places_loose_shapes_resting_on_the_table():
    montessori = MontessoriWorld()
    montessori.world.update_forward_kinematics()

    shapes = montessori.world.get_semantic_annotations_by_type(MontessoriShape)
    table_top_z = float(TABLE_POSITION.z) + TABLE_SCALE.z / 2
    table_min_y = float(TABLE_POSITION.y) - TABLE_SCALE.y / 2
    table_max_y = float(TABLE_POSITION.y) + TABLE_SCALE.y / 2

    y_positions = []
    for shape in shapes:
        position = shape.global_transform.to_position()
        lowest_local_z = shape.root.collision.combined_mesh.bounds[0][2]

        # the shape's lowest point rests exactly on the table's surface
        assert float(position.z) + lowest_local_z == pytest.approx(table_top_z)
        assert float(position.x) == TABLE_SHAPE_ROW_X
        assert table_min_y <= float(position.y) <= table_max_y
        y_positions.append(float(position.y))

    # every shape sits at its own spot along the row
    assert len(set(y_positions)) == len(shapes)


def test_montessori_world_creates_one_table_and_one_floor():
    montessori = MontessoriWorld()

    [table] = montessori.world.get_semantic_annotations_by_type(Table)
    assert table is not None
    [floor] = montessori.world.get_semantic_annotations_by_type(Floor)
    assert floor is not None


def test_montessori_world_table_stands_on_the_floor():
    montessori = MontessoriWorld()
    montessori.world.update_forward_kinematics()

    [table] = montessori.world.get_semantic_annotations_by_type(Table)
    position = table.global_transform.to_position()
    lowest_local_z = table.root.collision.combined_mesh.bounds[0][2]

    assert float(position.z) + lowest_local_z == pytest.approx(FLOOR_Z)


def test_montessori_world_has_no_hsrb_until_spawned():
    montessori = MontessoriWorld()

    assert montessori.hsrb is None


def test_montessori_world_spawn_hsrb_adds_the_robot_to_the_world():
    if not hsrb_installed():
        pytest.skip("hsr_description is not installed")

    montessori = MontessoriWorld()
    body_count_before_spawn = len(montessori.world.bodies)

    hsrb = montessori.spawn_hsrb()

    assert hsrb is montessori.hsrb
    assert len(montessori.world.bodies) > body_count_before_spawn
