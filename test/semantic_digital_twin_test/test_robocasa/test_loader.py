from dataclasses import dataclass, field

import pytest

pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

from semantic_digital_twin.adapters.robocasa_dataset.exceptions import (
    RoboCasaTaskNotFoundError,
)
from semantic_digital_twin.adapters.mjcf import MJCFParser
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
    _parse_robosuite_mjcf,
)
from semantic_digital_twin.adapters.robocasa_dataset.semantics import (
    RoboCasaKitchenApplianceCategory,
    RoboCasaObjectCategory,
)
from semantic_digital_twin.pipeline.pipeline import Pipeline, Step
from semantic_digital_twin.semantic_annotations.natural_language import (
    NaturalLanguageWithTypeDescription,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Dishwasher,
    Drawer,
    Floor,
)
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world import World


@dataclass
class RecordWorlds(Step):
    """
    A preprocessing step that only remembers the worlds it is applied to.
    """

    worlds: list[World] = field(default_factory=list)
    """
    The worlds the step was applied to, oldest first.
    """

    def _apply(self, world: World) -> World:
        self.worlds.append(world)
        return world


@pytest.fixture(scope="session")
def robocasa_loader() -> RoboCasaDatasetLoader:
    loader = RoboCasaDatasetLoader()
    if not loader.directory.exists():
        pytest.skip(
            "RoboCasa assets not downloaded. Run "
            "'python -m robocasa.scripts.download_kitchen_assets' first."
        )
    return loader


def test_load_kitchen(robocasa_loader):
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    world = robocasa_loader.load_kitchen(
        layout_id=next(iter(LayoutType)), style_id=next(iter(StyleType))
    )
    assert len(world.bodies) > 0
    assert len(world.semantic_annotations) > 0


def test_load_kitchen_annotates_its_floors(robocasa_loader):
    """
    The floor slab of the kitchen and the backing beneath it are annotated as floors, so
    a robot can stand on them.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    world = robocasa_loader.load_kitchen(
        layout_id=next(iter(LayoutType)), style_id=next(iter(StyleType))
    )

    assert {
        floor.root.name.name for floor in world.get_semantic_annotations_by_type(Floor)
    } == {"floor_room_main", "floor_backing_room_main"}


def test_load_kitchen_appliance(robocasa_loader):
    world = robocasa_loader.load_kitchen_appliance(
        RoboCasaKitchenApplianceCategory.CABINET
    )
    assert len(world.bodies) > 0
    assert len(world.semantic_annotations) >= 1


def test_load_kitchen_appliance_attaches_matching_annotation(robocasa_loader):
    """
    The loaded appliance is annotated with the semantic type of the requested category.
    """
    world = robocasa_loader.load_kitchen_appliance(
        RoboCasaKitchenApplianceCategory.DISHWASHER
    )
    assert any(
        isinstance(annotation, Dishwasher) for annotation in world.semantic_annotations
    )


def test_load_object(robocasa_loader):
    world = robocasa_loader.load_object(RoboCasaObjectCategory.APPLE)
    assert len(world.bodies) > 0
    annotations = world.semantic_annotations
    assert len(annotations) == 1
    assert not isinstance(annotations[0], NaturalLanguageWithTypeDescription)


def test_load_object_is_scaled_as_robocasa_scales_its_category(robocasa_loader):
    """
    RoboCasa scales every object of a category by that category's scale, so a loaded
    object has the size RoboCasa gives it, not the size of its model file.
    """
    from robocasa.models.objects.kitchen_objects import OBJ_CATEGORIES

    model_file = (
        robocasa_loader.directory / "objects" / "objaverse" / "pan" / "pan_0"
    ) / "model.xml"
    unscaled = _parse_robosuite_mjcf(MJCFParser.from_file(str(model_file)))
    scale = OBJ_CATEGORIES[RoboCasaObjectCategory.PAN]["objaverse"].scale

    loaded = robocasa_loader.load_object(RoboCasaObjectCategory.PAN)

    assert extent_of(loaded) == pytest.approx(
        [length * scale for length in extent_of(unscaled)], abs=1e-6
    )


def extent_of(world: World) -> list[float]:
    """
    :return: The length, width and height of the collision geometry of the only body
        of ``world`` that has any, in that body's own frame.
    """
    [body] = world.bodies_with_collision
    box = body.collision.as_bounding_box_collection_in_frame(body).bounding_box()
    return [box.max_x - box.min_x, box.max_y - box.min_y, box.max_z - box.min_z]


def test_load_object_from_group_without_objaverse_assets(robocasa_loader):
    """
    A category with no objaverse assets still loads from another self-contained group.
    """
    world = robocasa_loader.load_object(RoboCasaObjectCategory.POT)
    assert len(world.bodies) > 0


def test_the_model_files_of_a_category_are_those_load_object_indexes(robocasa_loader):
    model_files = robocasa_loader.object_model_files(RoboCasaObjectCategory.POT)

    assert model_files == sorted(model_files)
    assert all(
        model_file.parts[-1] == "model.xml"
        and RoboCasaObjectCategory.POT.value in model_file.parts
        for model_file in model_files
    )
    assert len(model_files) > 1


def test_load_task_binds_instruction_and_world(robocasa_loader):
    """
    The loaded task carries its natural-language instruction bound to the scene it plays
    out in.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    task = robocasa_loader.load_task(
        "TurnOnMicrowave",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
    )
    assert "microwave" in task.instruction.lower()
    assert len(task.world.bodies) > 0
    assert len(task.world.semantic_annotations) > 0


def test_load_task_omits_robot(robocasa_loader):
    """
    The task world contains no robot, since semantic_digital_twin owns the robot.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    task = robocasa_loader.load_task(
        "TurnOnMicrowave",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
    )
    assert not any(
        body.name.name.startswith(RoboCasaDatasetLoader.robot_body_name_prefixes)
        for body in task.world.bodies
    )


def test_load_task_manipulated_objects_are_in_world(robocasa_loader):
    """
    The task's manipulated objects are resolved to bodies present in the task world.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    task = robocasa_loader.load_task(
        "TurnOnMicrowave",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
    )
    assert len(task.manipulated_objects) >= 1
    for manipulated_object in task.manipulated_objects:
        assert manipulated_object in task.world.bodies


def test_load_task_with_a_seed_places_the_same_objects_every_time(robocasa_loader):
    """
    RoboCasa samples a task's objects and where they stand; seeded, it samples the same
    scene on every load.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    first, second = (
        robocasa_loader.load_task(
            "PickPlaceCounterToStove",
            layout_id=LayoutType.LAYOUT001,
            style_id=StyleType.STYLE001,
            seed=0,
        )
        for _ in range(2)
    )

    assert first.instruction == second.instruction
    assert [body.global_pose.to_np() for body in first.manipulated_objects] == [
        pytest.approx(body.global_pose.to_np()) for body in second.manipulated_objects
    ]


def test_load_task_applies_the_fixture_updates_it_is_given(robocasa_loader):
    """
    Updates to the fixtures of a layout change the kitchen RoboCasa composes: with every
    counter lowered, the plate the task sets on a counter stands at the lowered height.
    """
    import yaml
    from robocasa.models.scenes.scene_registry import (
        LayoutType,
        StyleType,
        get_layout_path,
    )

    with open(get_layout_path(LayoutType.LAYOUT001)) as layout_file:
        layout = yaml.safe_load(layout_file)
    lowered_counters = {
        fixture["name"]: {
            "size": [*fixture["size"][:2], LOWERED_COUNTER_HEIGHT],
            **(
                {"pos": [*fixture["pos"][:2], LOWERED_COUNTER_HEIGHT / 2]}
                if "pos" in fixture
                else {}
            ),
        }
        for fixture in layout["main_group"]["bottom_row"]
        if fixture["type"] == "counter"
    }

    task = robocasa_loader.load_task(
        "PickPlaceCounterToStove",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
        seed=1,
        fixture_updates=lowered_counters,
    )

    [plate] = [
        body
        for body in task.manipulated_objects
        if body.name.name == "obj_container_main"
    ]
    plate_extent = plate.collision.as_bounding_box_collection_in_frame(
        task.world.root
    ).bounding_box()
    assert plate_extent.min_z == pytest.approx(LOWERED_COUNTER_HEIGHT, abs=0.01)


def test_load_task_passes_its_task_parameters_to_the_task(robocasa_loader):
    """
    A task told which object category to sample from takes its object from that
    category.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    task = robocasa_loader.load_task(
        "PickPlaceSinkToCounter",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
        seed=11,
        task_parameters={"obj_groups": PINNED_CATEGORY},
    )

    [food_annotation] = [
        annotation
        for annotation in task.world.semantic_annotations
        if isinstance(annotation, NaturalLanguageWithTypeDescription)
        and annotation.root.name.name == "obj_main"
    ]
    assert food_annotation.type_description == PINNED_CATEGORY


def test_load_task_opens_a_drawer_towards_positive_positions(robocasa_loader):
    """
    A RoboCasa drawer opens towards negative positions; in the loaded world it opens
    towards positive ones, as semantic_digital_twin's drawers do, from closed at 0 to
    RoboCasa's full travel, and stands as far open as RoboCasa set it.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    environment = robocasa_loader.build_task_environment(
        "CloseDrawer",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
        seed=0,
    )
    model, data = environment.sim.model, environment.sim.data
    [joint] = [
        joint
        for joint in range(model.njnt)
        if environment.drawer.name in (model.joint_id2name(joint) or "")
    ]
    robocasa_lower, robocasa_upper = model.jnt_range[joint]
    robocasa_position = float(data.qpos[model.jnt_qposadr[joint]])

    task = robocasa_loader.load_task_from_environment(environment)

    connection = task.world.get_connection_by_name(model.joint_id2name(joint))
    assert (
        connection.dof.limits.lower.position,
        connection.dof.limits.upper.position,
    ) == pytest.approx((-robocasa_upper, -robocasa_lower))
    assert connection.position == pytest.approx(-robocasa_position)


def test_load_task_names_the_fixtures_the_task_refers_to(robocasa_loader):
    """
    The fixtures a task refers to, such as the drawer to open, are given by the name the
    task refers to them by, as their bodies in the loaded world.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    environment = robocasa_loader.build_task_environment(
        "OpenDrawer",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
        seed=0,
    )

    task = robocasa_loader.load_task_from_environment(environment)

    assert task.referenced_fixtures["drawer"].name.name.startswith(
        environment.drawer.name
    )


def test_load_task_puts_a_drawer_on_its_rails_with_its_handle(robocasa_loader):
    """
    A RoboCasa drawer is annotated at the part that slides, and its handle moves with
    it, rather than staying with the drawer's housing.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    task = robocasa_loader.load_task(
        "OpenDrawer",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
        seed=0,
    )
    housing = task.referenced_fixtures["drawer"]
    [drawer] = [
        annotation
        for annotation in task.world.get_semantic_annotations_by_type(Drawer)
        if annotation.root
        in task.world.compute_descendent_child_kinematic_structure_entities(housing)
    ]

    assert drawer.movable_joint is drawer.root.parent_connection
    assert (
        drawer.handle.root.get_first_parent_connection_of_type(ActiveConnection1DOF)
        is drawer.movable_joint
    )


def test_load_task_weighs_a_body_as_robocasa_weighs_it(robocasa_loader):
    """
    A body is given the mass RoboCasa simulates it with, which counts only the geoms
    RoboCasa weighs: a drawer weighs its panels, not the region marking its inside.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    environment = robocasa_loader.build_task_environment(
        "OpenDrawer",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
        seed=0,
    )
    model = environment.sim.model
    [joint] = [
        joint
        for joint in range(model.njnt)
        if environment.drawer.name in (model.joint_id2name(joint) or "")
    ]
    drawer = model.jnt_bodyid[joint]

    task = robocasa_loader.load_task_from_environment(environment)

    inertial = task.world.get_body_by_name(model.body_id2name(drawer)).inertial
    assert inertial.mass == pytest.approx(model.body_mass[drawer])


PINNED_CATEGORY = "yogurt"
"""
The object category a test pins a task's object to.
"""

LOWERED_COUNTER_HEIGHT = 0.75
"""
The height, in metres, a test lowers every counter of a layout to.
"""


def test_load_task_unknown_name_raises(robocasa_loader):
    """
    Requesting a task that is not registered raises a descriptive error.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    with pytest.raises(RoboCasaTaskNotFoundError):
        robocasa_loader.load_task(
            "ThisTaskDoesNotExist",
            layout_id=LayoutType.LAYOUT001,
            style_id=StyleType.STYLE001,
        )


def test_task_environments_built_with_the_same_seed_compose_the_same_scene(
    robocasa_loader,
):
    """
    Two task environments built with the same seed sample the same objects and place
    them alike.
    """
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    first, second = (
        robocasa_loader.build_task_environment(
            "PickPlaceCounterToSink",
            layout_id=LayoutType.LAYOUT001,
            style_id=StyleType.STYLE001,
            seed=7,
        )
        for _ in range(2)
    )

    assert {name: model.mjcf_path for name, model in first.objects.items()} == {
        name: model.mjcf_path for name, model in second.objects.items()
    }
    assert [
        first.sim.data.get_body_xpos(model.root_body).tolist()
        for model in first.objects.values()
    ] == [
        second.sim.data.get_body_xpos(model.root_body).tolist()
        for model in second.objects.values()
    ]


def test_a_loaded_object_is_preprocessed(robocasa_loader):
    step = RecordWorlds()
    loader = RoboCasaDatasetLoader(
        directory=robocasa_loader.directory, preprocessing=Pipeline(steps=[step])
    )

    world = loader.load_object(RoboCasaObjectCategory.APPLE)

    assert step.worlds == [world]


def test_a_loaded_kitchen_appliance_is_preprocessed(robocasa_loader):
    step = RecordWorlds()
    loader = RoboCasaDatasetLoader(
        directory=robocasa_loader.directory, preprocessing=Pipeline(steps=[step])
    )

    world = loader.load_kitchen_appliance(RoboCasaKitchenApplianceCategory.DISHWASHER)

    assert step.worlds == [world]


def test_a_loaded_task_is_preprocessed_before_its_objects_are_resolved(
    robocasa_loader,
):
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

    step = RecordWorlds()
    loader = RoboCasaDatasetLoader(
        directory=robocasa_loader.directory, preprocessing=Pipeline(steps=[step])
    )

    task = loader.load_task(
        "TurnOnMicrowave",
        layout_id=LayoutType.LAYOUT001,
        style_id=StyleType.STYLE001,
    )

    assert step.worlds == [task.world]
    for manipulated_object in task.manipulated_objects:
        assert manipulated_object in task.world.bodies
