from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree as ET

from typing_extensions import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    import numpy
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

from semantic_digital_twin.adapters.mjcf import MJCFParser
from semantic_digital_twin.adapters.robocasa_dataset.semantics import (
    RoboCasaFixtureResolver,
    RoboCasaObjectResolver,
)
from semantic_digital_twin.exceptions import WorldEntityNotFoundError
from semantic_digital_twin.semantic_annotations.natural_language import (
    NaturalLanguageWithTypeDescription,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

try:
    import robocasa
    import robosuite
except ImportError:
    logger.warning(
        "robocasa/robosuite are required for RoboCasa dataset loading. Install robosuite from git "
        "('pip install git+https://github.com/ARISE-Initiative/robosuite.git') and then robocasa "
        "('pip install -e .' from a clone of https://github.com/robocasa/robocasa), then run "
        "'python -m robocasa.scripts.download_kitchen_assets' to fetch the fixture/object assets."
    )


def _mjcf_document_from_element(element: ET.Element) -> str:
    """
    Wrap a single MJCF XML element (for example one fixture's geometry) into a minimal standalone
    MJCF document so it can be parsed on its own.

    :param element: The XML element to wrap, typically a RoboCasa fixture's underlying body element.
    :return: The MJCF document as a string.
    """
    root = ET.Element("mujoco")
    worldbody = ET.SubElement(root, "worldbody")
    worldbody.append(element)
    return ET.tostring(root, encoding="unicode")


@dataclass
class RoboCasaDatasetLoader:
    """
    Loader for objects, fixtures, and full kitchen scenes from the RoboCasa dataset
    (https://github.com/robocasa/robocasa).

    RoboCasa composes its assets with robosuite/MuJoCo. This loader drives RoboCasa's own Python
    composition code to build the MJCF for a requested fixture or kitchen, and parses the resulting
    MJCF with the existing :class:`~semantic_digital_twin.adapters.mjcf.MJCFParser`.

    For this to work, ``robocasa`` and ``robosuite`` (installed from git) must be available, and the
    RoboCasa fixture/object assets must be downloaded via
    ``python -m robocasa.scripts.download_kitchen_assets``.

    .. note::
        RoboCasa does not version its internal Python module layout as a stable public API. The import
        paths used here match the module layout at the time this loader was written; if they no longer
        match the installed version, adjust the imports inside the affected method accordingly.
    """

    directory: Path = field(
        default_factory=lambda: Path.home() / "robocasa-assets",
    )
    """
    The directory where the RoboCasa fixture/object assets were downloaded to via
    ``python -m robocasa.scripts.download_kitchen_assets``.
    """

    fixture_resolver: RoboCasaFixtureResolver = field(
        default_factory=RoboCasaFixtureResolver
    )
    """
    Resolver mapping RoboCasa fixture category names to SemanticAnnotation subclasses.
    """

    object_resolver: RoboCasaObjectResolver = field(
        default_factory=RoboCasaObjectResolver
    )
    """
    Resolver mapping RoboCasa object category names to SemanticAnnotation subclasses.
    """

    def load_kitchen(
        self,
        layout_id: LayoutType,
        style_id: StyleType,
        rng: Optional[numpy.random.Generator] = None,
    ) -> World:
        """
        Compose a full RoboCasa kitchen scene and parse it into a World.

        :param layout_id: A member of ``robocasa.models.scenes.scene_registry.LayoutType``.
        :param style_id: A member of ``robocasa.models.scenes.scene_registry.StyleType``.
        :param rng: The random number generator used for the (deterministic, non-physical) fixture
            placement within the layout. Defaults to RoboCasa's own default generator.
        :return: The composed world, with a SemanticAnnotation attached to each fixture's root body.
        """
        import yaml
        from robocasa.models.arenas.kitchen_arena import KitchenArena
        from robocasa.models.scenes import scene_builder, scene_registry
        from robosuite.models.tasks import ManipulationTask

        with open(scene_registry.get_layout_path(layout_id)) as layout_file:
            layout_config = yaml.safe_load(layout_file)
        with open(scene_registry.get_style_path(style_id)) as style_file:
            style_config = yaml.safe_load(style_file)

        arena = KitchenArena(layout_id=layout_id, style_id=style_id, rng=rng)
        fixtures = scene_builder.create_fixtures(layout_config, style_config, rng=rng)

        task = ManipulationTask(
            mujoco_arena=arena,
            mujoco_robots=[],
            mujoco_objects=list(fixtures.values()),
        )

        world = MJCFParser.from_xml_string(task.get_xml()).parse()
        self._apply_fixture_semantics(world, fixtures)
        return world

    def load_fixture(self, category: str, **fixture_kwargs) -> World:
        """
        Load a single RoboCasa fixture as a standalone world.

        :param category: The fixture category, for example ``"cabinet"`` or ``"microwave"``, a key of
            ``robocasa.models.scenes.scene_builder.FIXTURES``.
        :param fixture_kwargs: Extra keyword arguments forwarded to the fixture's constructor.
        :return: The loaded world, with a SemanticAnnotation attached to the fixture's root body.
        """
        from robocasa.models.scenes.scene_builder import FIXTURES

        fixture_class = FIXTURES[category]
        fixture = fixture_class(name=category, **fixture_kwargs)

        world = MJCFParser.from_xml_string(
            _mjcf_document_from_element(fixture._obj)
        ).parse()
        self._apply_fixture_semantics(world, {category: fixture})
        return world

    def load_object(self, category: str, instance_index: int = 0) -> World:
        """
        Load a single RoboCasa object as a standalone world.

        :param category: The object category, a key of
            ``robocasa.models.objects.kitchen_objects.OBJ_CATEGORIES``.
        :param instance_index: Which of the category's downloaded asset instances to load.
        :return: The loaded world, with a SemanticAnnotation attached to the object's root body.
        """
        from robocasa.models.objects.kitchen_objects import OBJ_CATEGORIES

        if category not in OBJ_CATEGORIES:
            raise ValueError(
                f"Unknown RoboCasa object category '{category}'. "
                f"Known categories: {sorted(OBJ_CATEGORIES)}"
            )

        model_files = sorted((self.directory / "objects" / category).glob("**/model.xml"))
        if not model_files:
            raise FileNotFoundError(
                f"No downloaded assets found for object category '{category}' in "
                f"{self.directory / 'objects' / category}. "
                "Run 'python -m robocasa.scripts.download_kitchen_assets' first."
            )

        world = MJCFParser.from_file(str(model_files[instance_index])).parse()
        self._apply_object_semantics(world, category)
        return world

    def _apply_fixture_semantics(
        self, world: World, fixtures: Dict[str, Any]
    ) -> None:
        """
        Attach a SemanticAnnotation to the root body of each fixture, using the fixture resolver where
        the fixture's category matches a known SemanticAnnotation subclass, and falling back to
        NaturalLanguageWithTypeDescription otherwise.

        :param world: The world the fixtures were parsed into.
        :param fixtures: Mapping from fixture name to the RoboCasa Fixture instance it was built from.
        """
        for fixture_name, fixture in fixtures.items():
            body = self._find_body(world, fixture_name)
            if body is None:
                continue
            category = _category_from_class_name(type(fixture).__name__)
            self._attach_semantic_annotation(world, body, category)

    def _apply_object_semantics(self, world: World, category: str) -> None:
        """
        Attach a SemanticAnnotation to the root body of a loaded object.

        :param world: The world the object was parsed into.
        :param category: The RoboCasa object category the object belongs to.
        """
        body = world.root
        self._attach_semantic_annotation(world, body, category)

    def _attach_semantic_annotation(self, world: World, body: Body, category: str) -> None:
        """
        Attach the SemanticAnnotation matching ``category`` to ``body``, falling back to
        NaturalLanguageWithTypeDescription if no matching SemanticAnnotation subclass is known.

        :param world: The world ``body`` belongs to.
        :param body: The body to annotate.
        :param category: The RoboCasa fixture or object category of ``body``.
        """
        annotation_class = self.fixture_resolver.resolve(
            category
        ) or self.object_resolver.resolve(category)

        with world.modify_world():
            if annotation_class is not None:
                world.add_semantic_annotation(annotation_class(root=body))
            else:
                world.add_semantic_annotation(
                    NaturalLanguageWithTypeDescription(
                        root=body, description=category, type_description=category
                    )
                )

    @staticmethod
    def _find_body(world: World, name: str) -> Optional[Body]:
        """
        Look up a body by name, returning None instead of raising if it is not present.

        :param world: The world to search.
        :param name: The name of the body to look up.
        :return: The body, or None if no body with this name exists in the world.
        """
        try:
            return world.get_body_by_name(name)
        except WorldEntityNotFoundError:
            return None


def _category_from_class_name(class_name: str) -> str:
    """
    Convert a RoboCasa Fixture subclass name (upper camel case, for example ``"HingeCabinet"``) into a
    lower snake case category string (for example ``"hinge_cabinet"``) suitable for
    :meth:`RoboCasaFixtureResolver.resolve`.

    :param class_name: The RoboCasa Fixture subclass name.
    :return: The lower snake case category string.
    """
    snake_case_characters = []
    for index, character in enumerate(class_name):
        if character.isupper() and index > 0:
            snake_case_characters.append("_")
        snake_case_characters.append(character.lower())
    return "".join(snake_case_characters)
