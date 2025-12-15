import unittest

import pytest
import rclpy

from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import InvalidDoorDimensions

from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
    Door,
    Drawer,
    Dresser,
    Wall,
    Hinge,
)
from semantic_digital_twin.spatial_types.spatial_types import (
    TransformationMatrix,
    Vector3,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale


class TestFactories(unittest.TestCase):
    def test_handle_factory(self):
        world = World()
        returned_handle = Handle.create_with_new_body_in_world(
            name=PrefixedName("handle"),
            scale=Scale(0.1, 0.2, 0.03),
            thickness=0.03,
            world=world,
        )
        semantic_handle_annotations = world.get_semantic_annotations_by_type(Handle)
        self.assertEqual(len(semantic_handle_annotations), 1)

        queried_handle: Handle = semantic_handle_annotations[0]
        self.assertEqual(returned_handle, queried_handle)
        self.assertEqual(world.root, queried_handle.body)

        # this belongs into whatever tests merge_world, and with dummy objects, not handles
        for i in range(10):
            returned_handle = Handle.create_with_new_body_in_world(
                name=PrefixedName(f"handle_{i}"),
                scale=Scale(0.1, 0.2, 0.03),
                thickness=0.03,
                parent=returned_handle.body,
                world=world,
            )

        self.assertEqual(world.root.name.name, "handle")
        semantic_handle_annotations = world.get_semantic_annotations_by_type(Handle)
        self.assertEqual(11, len(semantic_handle_annotations))
        self.assertEqual(11, len(world.bodies))

    def test_hinge_factory(self):
        world = World()
        returned_hinge = Hinge.create_with_new_body_in_world(
            name=PrefixedName("hinge"),
            world=world,
        )
        semantic_hinge_annotations = world.get_semantic_annotations_by_type(Hinge)
        self.assertEqual(len(semantic_hinge_annotations), 1)

        queried_hinge: Hinge = semantic_hinge_annotations[0]
        self.assertEqual(returned_hinge, queried_hinge)
        self.assertEqual(world.root, queried_hinge.body)

    def test_door_factory(self):
        world = World()
        returned_door = Door.create_with_new_body_in_world(
            name=PrefixedName("door"), scale=Scale(0.03, 1, 2), world=world
        )
        semantic_door_annotations = world.get_semantic_annotations_by_type(Door)
        self.assertEqual(len(semantic_door_annotations), 1)

        queried_door: Door = semantic_door_annotations[0]
        self.assertEqual(returned_door, queried_door)
        self.assertEqual(world.root, queried_door.body)

    def test_door_factory_invalid(self):
        world = World()
        with pytest.raises(InvalidDoorDimensions):
            Door.create_with_new_body_in_world(
                name=PrefixedName("door"), scale=Scale(1, 1, 2), world=world
            )

        with pytest.raises(InvalidDoorDimensions):
            Door.create_with_new_body_in_world(
                name=PrefixedName("door"), scale=Scale(1, 2, 1), world=world
            )

    def test_door_constructed_world(self):
        world = World()
        hinge = Hinge.create_with_new_body_in_world(
            name=PrefixedName("hinge"), world=world
        )
        door = Door.create_with_new_body_in_world(
            name=PrefixedName("door"), scale=Scale(0.03, 1, 2), world=world
        )
        door.add

    def test_double_door_factory(self):
        handle_factory = HandleFactory(name=PrefixedName("handle"))
        handle_factory_config = handle_factory.get_config_for_parent_factory(
            semantic_handle_position=SemanticPositionDescription(
                horizontal_direction_chain=[
                    HorizontalSemanticDirection.RIGHT,
                    HorizontalSemanticDirection.FULLY_CENTER,
                ],
                vertical_direction_chain=[VerticalSemanticDirection.FULLY_CENTER],
            ),
        )
        door_factory = DoorFactory(
            name=PrefixedName("door"), handle_factory_config=handle_factory_config
        )
        door_factory_config = door_factory.get_config_for_parent_factory(
            parent_T_child=TransformationMatrix.from_xyz_rpy(y=-0.5),
            hinge_axis=Vector3.Z(),
        )

        handle_factory2 = HandleFactory(name=PrefixedName("handle2"))
        handle_factory_config2 = handle_factory.get_config_for_parent_factory(
            semantic_handle_position=SemanticPositionDescription(
                horizontal_direction_chain=[
                    HorizontalSemanticDirection.LEFT,
                    HorizontalSemanticDirection.FULLY_CENTER,
                ],
                vertical_direction_chain=[VerticalSemanticDirection.FULLY_CENTER],
            ),
        )
        door_factory2 = DoorFactory(
            name=PrefixedName("door2"), handle_factory_config=handle_factory_config2
        )
        door_factory_config2 = door_factory2.get_config_for_parent_factory(
            parent_T_child=TransformationMatrix.from_xyz_rpy(y=0.5),
            hinge_axis=Vector3.Z(),
        )

        factory = DoubleDoorFactory(
            name=PrefixedName("double_door"),
            door_like_factory_configs=[door_factory_config, door_factory_config2],
        )
        world = factory.create()
        doors = world.get_semantic_annotations_by_type(Door)
        self.assertEqual(len(doors), 2)
        self.assertEqual(
            set(world.root.child_kinematic_structure_entities),
            {
                doors[0].body.parent_kinematic_structure_entity,
                doors[1].body.parent_kinematic_structure_entity,
            },
        )
        self.assertIsInstance(doors[0].handle, Handle)
        self.assertIsInstance(doors[1].handle, Handle)
        self.assertNotEqual(doors[0].handle, doors[1].handle)

    def test_container_factory(self):
        factory = ContainerFactory(name=PrefixedName("container"))
        world = factory.create()
        semantic_container_annotations = world.get_semantic_annotations_by_type(Corpus)
        self.assertEqual(len(semantic_container_annotations), 1)

        container: Corpus = semantic_container_annotations[0]
        self.assertEqual(world.root, container.body)

    def test_drawer_factory(self):
        container_factory = ContainerFactory(name=PrefixedName("container"))
        container_factory_config = container_factory.get_config_for_parent_factory(
            TransformationMatrix()
        )

        handle_factory = HandleFactory(name=PrefixedName("handle"))
        handle_factory_config = handle_factory.get_config_for_parent_factory(
            semantic_handle_position=SemanticPositionDescription(
                horizontal_direction_chain=[
                    HorizontalSemanticDirection.FULLY_CENTER,
                ],
                vertical_direction_chain=[VerticalSemanticDirection.FULLY_CENTER],
            ),
        )
        factory = DrawerFactory(
            name=PrefixedName("drawer"),
            container_factory_config=container_factory_config,
            handle_factory_config=handle_factory_config,
        )
        world = factory.create()
        semantic_drawer_annotations = world.get_semantic_annotations_by_type(Drawer)
        self.assertEqual(len(semantic_drawer_annotations), 1)

        drawer: Drawer = semantic_drawer_annotations[0]
        self.assertEqual(world.root, drawer.container.body)

    def test_dresser_factory(self):
        container_factory = ContainerFactory(name=PrefixedName("drawer_container"))
        container_factory_config = container_factory.get_config_for_parent_factory(
            TransformationMatrix()
        )

        handle_factory = HandleFactory(name=PrefixedName("drawer_handle"))
        handle_factory_config = handle_factory.get_config_for_parent_factory(
            semantic_handle_position=SemanticPositionDescription(
                horizontal_direction_chain=[
                    HorizontalSemanticDirection.FULLY_CENTER,
                ],
                vertical_direction_chain=[VerticalSemanticDirection.FULLY_CENTER],
            ),
        )
        drawer_factory = DrawerFactory(
            name=PrefixedName("drawer"),
            container_factory_config=container_factory_config,
            handle_factory_config=handle_factory_config,
        )
        drawer_factory_config = drawer_factory.get_config_for_parent_factory(
            parent_T_child=TransformationMatrix.from_xyz_rpy(x=0.5)
        )

        handle_factory2 = HandleFactory(name=PrefixedName("door_handle"))
        handle_factory_config2 = handle_factory2.get_config_for_parent_factory(
            semantic_handle_position=SemanticPositionDescription(
                horizontal_direction_chain=[
                    HorizontalSemanticDirection.LEFT,
                    HorizontalSemanticDirection.FULLY_CENTER,
                ],
                vertical_direction_chain=[
                    # VerticalSemanticDirection.TOP,
                    VerticalSemanticDirection.FULLY_CENTER,
                ],
            ),
        )
        door_factory = DoorFactory(
            name=PrefixedName("door"),
            handle_factory_config=handle_factory_config2,
            scale=Scale(0.03, 1, 1.0),
        )

        door_factory_config = door_factory.get_config_for_parent_factory(
            parent_T_child=TransformationMatrix.from_xyz_rpy(y=-0.5),
            hinge_axis=Vector3.Z(),
        )

        container_factory = ContainerFactory(name=PrefixedName("dresser_container"))
        container_factory_config = container_factory.get_config_for_parent_factory(
            TransformationMatrix()
        )

        dresser_factory = DresserFactory(
            name=PrefixedName("dresser"),
            drawer_factory_configs=[drawer_factory_config],
            door_like_factory_configs=[door_factory_config],
            container_factory_config=container_factory_config,
        )

        world = dresser_factory.create()
        world.state.positions[0] = 1
        world.notify_state_change()

        rclpy.init()
        node = rclpy.create_node("semantic_digital_twin")

        viz = VizMarkerPublisher(world=world, node=node)

        semantic_dresser_annotations = world.get_semantic_annotations_by_type(Dresser)
        semantic_drawer_annotations = world.get_semantic_annotations_by_type(Drawer)
        semantic_door_annotations = world.get_semantic_annotations_by_type(Door)
        self.assertEqual(len(semantic_drawer_annotations), 1)
        self.assertEqual(len(semantic_dresser_annotations), 1)
        self.assertEqual(len(semantic_door_annotations), 1)
        dresser: Dresser = semantic_dresser_annotations[0]
        self.assertEqual(world.root, dresser.container.body)

    def test_wall_factory(self):
        handle_factory = HandleFactory(name=PrefixedName("handle"))
        handle_factory_config = handle_factory.get_config_for_parent_factory(
            semantic_handle_position=SemanticPositionDescription(
                horizontal_direction_chain=[
                    HorizontalSemanticDirection.RIGHT,
                    HorizontalSemanticDirection.FULLY_CENTER,
                ],
                vertical_direction_chain=[VerticalSemanticDirection.FULLY_CENTER],
            ),
        )
        door_factory = DoorFactory(
            name=PrefixedName("door"),
            handle_factory_config=handle_factory_config,
        )
        door_factory_config = door_factory.get_config_for_parent_factory(
            parent_T_child=TransformationMatrix.from_xyz_rpy(y=-0.5),
            hinge_axis=Vector3.Z(),
        )

        handle_factory2 = HandleFactory(name=PrefixedName("handle2"))
        handle_factory_config2 = handle_factory.get_config_for_parent_factory(
            semantic_handle_position=SemanticPositionDescription(
                horizontal_direction_chain=[
                    HorizontalSemanticDirection.LEFT,
                    HorizontalSemanticDirection.FULLY_CENTER,
                ],
                vertical_direction_chain=[VerticalSemanticDirection.FULLY_CENTER],
            ),
        )
        door_factory2 = DoorFactory(
            name=PrefixedName("door2"),
            handle_factory_config=handle_factory_config2,
        )
        door_factory_config2 = door_factory2.get_config_for_parent_factory(
            parent_T_child=TransformationMatrix.from_xyz_rpy(y=0.5),
            hinge_axis=Vector3.Z(),
        )

        double_door_factory = DoubleDoorFactory(
            name=PrefixedName("double_door"),
            door_like_factory_configs=[door_factory_config, door_factory_config2],
        )
        double_door_factory_config = double_door_factory.get_config_for_parent_factory(
            parent_T_child=TransformationMatrix.from_xyz_rpy(x=0.5)
        )

        single_door_handle_factory = HandleFactory(
            name=PrefixedName("single_door_handle")
        )
        single_door_handle_factory_config = (
            single_door_handle_factory.get_config_for_parent_factory(
                semantic_handle_position=SemanticPositionDescription(
                    horizontal_direction_chain=[
                        HorizontalSemanticDirection.RIGHT,
                        HorizontalSemanticDirection.FULLY_CENTER,
                    ],
                    vertical_direction_chain=[VerticalSemanticDirection.FULLY_CENTER],
                ),
            )
        )
        single_door_factory = DoorFactory(
            name=PrefixedName("single_door"),
            handle_factory_config=single_door_handle_factory_config,
        )
        single_door_factory_config = (
            single_door_handle_factory.get_config_for_parent_factory(
                parent_T_child=TransformationMatrix.from_xyz_rpy(y=-1.5),
                hinge_axis=Vector3.Z(),
            )
        )

        factory = WallFactory(
            name=PrefixedName("wall"),
            scale=Scale(0.1, 4, 2),
            door_like_factory_configs=[
                double_door_factory_config,
                single_door_factory_config,
            ],
        )
        world = factory.create()
        semantic_wall_annotations = world.get_semantic_annotations_by_type(Wall)
        self.assertEqual(len(semantic_wall_annotations), 1)

        wall: Wall = semantic_wall_annotations[0]
        self.assertEqual(world.root, wall.body)


if __name__ == "__main__":
    unittest.main()
