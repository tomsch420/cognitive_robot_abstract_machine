from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Type

from typing_extensions import Dict

from semantic_digital_twin.semantic_annotations.mixins import HasRootBody
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Apple,
    Banana,
    Bottle,
    Bowl,
    Bread,
    Cabinet,
    Carrot,
    CoffeeMachine,
    Cooktop,
    CounterTop,
    Cup,
    Dishwasher,
    Drawer,
    Fridge,
    Hood,
    Kettle,
    Lettuce,
    Microwave,
    Mug,
    Orange,
    Oven,
    Pan,
    Plate,
    Pot,
    Potato,
    Sink,
    Toaster,
    Tomato,
)


@dataclass
class RoboCasaFixtureResolver:
    """
    Maps RoboCasa fixture category names (the module names under ``robocasa.models.fixtures``) to the
    matching SemanticAnnotation subclass already defined in the digital twin.

    Categories with no closely matching semantic concept resolve to ``None``; callers should fall back
    to a generic annotation (for example :class:`~semantic_digital_twin.semantic_annotations.natural_language.NaturalLanguageWithTypeDescription`)
    for those.
    """

    fixture_category_to_annotation_class: Dict[str, Type[HasRootBody]] = field(
        default_factory=lambda: {
            "cabinet": Cabinet,
            "cabinets": Cabinet,
            "drawer": Drawer,
            "fridge": Fridge,
            "microwave": Microwave,
            "oven": Oven,
            "dishwasher": Dishwasher,
            "hood": Hood,
            "counter": CounterTop,
            "sink": Sink,
            "stove": Cooktop,
            "toaster": Toaster,
            "coffee_machine": CoffeeMachine,
        }
    )
    """
    Lookup table from lower-cased RoboCasa fixture category name to the matching SemanticAnnotation subclass.
    """

    def resolve(self, fixture_category: str) -> Optional[Type[HasRootBody]]:
        """
        Resolve a RoboCasa fixture category name to the matching SemanticAnnotation subclass.

        :param fixture_category: The RoboCasa fixture category name, for example ``"cabinet"`` or ``"microwave"``.
        :return: The matching SemanticAnnotation subclass, or None if no close match is registered.
        """
        return self.fixture_category_to_annotation_class.get(fixture_category.lower())


@dataclass
class RoboCasaObjectResolver:
    """
    Maps RoboCasa object category names (the keys of ``robocasa.models.objects.kitchen_objects.OBJ_CATEGORIES``)
    to the matching SemanticAnnotation subclass already defined in the digital twin.

    Categories with no closely matching semantic concept resolve to ``None``; callers should fall back
    to a generic annotation (for example :class:`~semantic_digital_twin.semantic_annotations.natural_language.NaturalLanguageWithTypeDescription`)
    for those.
    """

    object_category_to_annotation_class: Dict[str, Type[HasRootBody]] = field(
        default_factory=lambda: {
            "apple": Apple,
            "banana": Banana,
            "orange": Orange,
            "tomato": Tomato,
            "lettuce": Lettuce,
            "carrot": Carrot,
            "potato": Potato,
            "bottle": Bottle,
            "cup": Cup,
            "mug": Mug,
            "bowl": Bowl,
            "plate": Plate,
            "pan": Pan,
            "pot": Pot,
            "kettle": Kettle,
            "bread": Bread,
        }
    )
    """
    Lookup table from lower-cased RoboCasa object category name to the matching SemanticAnnotation subclass.
    """

    def resolve(self, object_category: str) -> Optional[Type[HasRootBody]]:
        """
        Resolve a RoboCasa object category name to the matching SemanticAnnotation subclass.

        :param object_category: The RoboCasa object category name, for example ``"apple"`` or ``"mug"``.
        :return: The matching SemanticAnnotation subclass, or None if no close match is registered.
        """
        return self.object_category_to_annotation_class.get(object_category.lower())
