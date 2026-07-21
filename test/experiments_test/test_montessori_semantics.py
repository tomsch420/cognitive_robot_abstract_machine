import pytest

from experiments.montessori.semantics import (
    MontessoriShape,
    MontessoriShapeCategory,
    ShapeSortingBoard,
    ShapeSortingHole,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import Drawer
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Body


@pytest.fixture
def world():
    world = World()
    root = Body(name=PrefixedName("root"))
    with world.modify_world():
        world.add_body(root)
    return world


@pytest.mark.parametrize("category", list(MontessoriShapeCategory))
def test_montessori_shape_stores_its_shape_category(world, category):
    body = Body(name=PrefixedName("piece"))
    with world.modify_world():
        world.add_body(body)
        world.add_connection(FixedConnection(parent=world.root, child=body))
        shape = MontessoriShape(
            name=PrefixedName("piece"), root=body, shape_category=category
        )
        world.add_semantic_annotation(shape)

    assert world.get_semantic_annotations_by_type(MontessoriShape) == [shape]
    assert shape.shape_category == category


def test_shape_sorting_hole_stores_its_shape_category(world):
    with world.modify_world():
        hole = ShapeSortingHole.create_with_new_region_in_world(
            name=PrefixedName("square_hole"),
            world=world,
            scale=Scale(0.03, 0.03, 0.001),
        )
        hole.shape_category = MontessoriShapeCategory.CUBE

    assert hole.shape_category == MontessoriShapeCategory.CUBE


def test_shape_sorting_board_collects_drawers_and_shape_holes(world):
    with world.modify_world():
        board = ShapeSortingBoard.create_with_new_body_in_world(
            name=PrefixedName("board"), world=world, scale=Scale(0.3, 0.3, 0.1)
        )
        drawer = Drawer.create_with_new_body_in_world(
            name=PrefixedName("drawer"), world=world
        )
        hole = ShapeSortingHole.create_with_new_region_in_world(
            name=PrefixedName("triangle_hole"),
            world=world,
            scale=Scale(0.03, 0.03, 0.001),
        )
        hole.shape_category = MontessoriShapeCategory.TRIANGULAR_PRISM
        board.add(drawer)
        board.add(hole)

    assert board.drawers == [drawer]
    assert board.apertures == [hole]
