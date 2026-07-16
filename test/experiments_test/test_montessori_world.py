from experiments.montessori.semantics import (
    MontessoriShape,
    MontessoriShapeCategory,
    ShapeSortingBoard,
    ShapeSortingHole,
)
from experiments.montessori.world import BOARD_SCALE, build_montessori_world
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    SideTable,
    Table,
)


def test_build_montessori_world_creates_one_board_with_holes_and_drawers():
    world = build_montessori_world()

    [board] = world.get_semantic_annotations_by_type(ShapeSortingBoard)
    assert len(board.apertures) == 6
    assert len(board.drawers) == 3
    assert all(drawer.handle is not None for drawer in board.drawers)


def test_build_montessori_world_cuts_a_watertight_board_mesh():
    world = build_montessori_world()
    world.update_forward_kinematics()

    [board] = world.get_semantic_annotations_by_type(ShapeSortingBoard)
    board_mesh = board.root.combined_mesh

    assert board_mesh.is_watertight
    # cutting 6 holes must strictly shrink the solid board's volume
    uncut_volume = BOARD_SCALE.x * BOARD_SCALE.y * BOARD_SCALE.z
    assert 0 < board_mesh.volume < uncut_volume


def test_build_montessori_world_creates_one_shape_per_hole_category_plus_the_sphere():
    world = build_montessori_world()

    holes = world.get_semantic_annotations_by_type(ShapeSortingHole)
    hole_categories = {hole.shape_category for hole in holes}
    shape_categories = [
        shape.shape_category for shape in world.get_semantic_annotations_by_type(MontessoriShape)
    ]

    # one shape per hole (some categories repeat, e.g. the two circular holes), plus
    # the sphere, which has no matching hole
    assert set(shape_categories) == hole_categories | {MontessoriShapeCategory.SPHERE}
    assert len(shape_categories) == len(holes) + 1


def test_build_montessori_world_places_each_shape_above_its_matching_hole():
    world = build_montessori_world()
    world.update_forward_kinematics()

    holes = {
        hole.shape_category: hole
        for hole in world.get_semantic_annotations_by_type(ShapeSortingHole)
    }
    shapes = {
        shape.shape_category: shape
        for shape in world.get_semantic_annotations_by_type(MontessoriShape)
        if shape.shape_category in holes
    }

    for category, shape in shapes.items():
        hole_position = holes[category].global_transform.to_position()
        shape_position = shape.global_transform.to_position()
        assert float(shape_position.x) == float(hole_position.x)
        assert float(shape_position.y) == float(hole_position.y)
        assert float(shape_position.z) > float(hole_position.z)


def test_build_montessori_world_creates_table_and_side_table():
    world = build_montessori_world()

    assert len(world.get_semantic_annotations_by_type(Table)) == 2
    [side_table] = world.get_semantic_annotations_by_type(SideTable)
    assert side_table is not None
