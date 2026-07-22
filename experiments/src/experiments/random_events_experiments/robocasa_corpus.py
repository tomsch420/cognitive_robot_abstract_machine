"""
This module extracts the placement specifications of the RoboCasa kitchen task suite and
classifies what each one demands of a product algebra representation.

RoboCasa task definitions state where an object may be placed in a constraint vocabulary
that was authored independently of :mod:`random_events`. They therefore serve as an
external reality check on whether the product algebra can hold the missing knowledge
CRAM needs, free of the bias that arises when the same author writes both the
representation and the queries exercising it.

The specifications are read from the task sources rather than from instantiated
environments, so the classification covers every task without depending on assets,
physics or a sampled scene. Demands whose resolution needs a concrete scene are reported
as such instead of being silently decided.
"""

from __future__ import annotations

import ast
import enum
import importlib.util
import math
import pathlib
from collections import Counter
from dataclasses import dataclass, field

from typing_extensions import Any, Iterator

from experiments.experiment_definitions import (
    ExperimentResult,
    ExperimentsTable,
    TypstRenderer,
)

TASK_PACKAGES = ("atomic", "composite")
"""
Sub packages of the RoboCasa kitchen environments that hold task definitions.
"""

DEFAULT_ORIENTATION_RANGE = (-math.pi / 4, math.pi / 4)
"""
Yaw range RoboCasa samples an object over when a placement does not state one.

Mirrors the default in RoboCasa's placement resolution, where an unstated rotation is a
non degenerate range and therefore leaves the object's orientation underspecified.
"""

RESOLVABLE_CONSTANTS = {"pi": math.pi}
"""
Attribute names this module resolves to numbers, so yaw ranges written as multiples of
pi become literals instead of being reported as scene dependent.
"""


class PlacementKey(enum.Enum):
    """
    Keyword arguments of a RoboCasa placement specification that influence the region an
    object may be placed in.
    """

    FIXTURE = "fixture"
    OBJECT = "object"
    SIZE = "size"
    POSITION = "pos"
    OFFSET = "offset"
    MARGIN = "margin"
    ROTATION = "rotation"
    ROTATION_AXIS = "rotation_axis"
    SAMPLE_REGION_KWARGS = "sample_region_kwargs"
    REFERENCE_OBJECT = "ref_obj"
    REUSE_REGION_FROM = "reuse_region_from"
    TRY_TO_PLACE_IN = "try_to_place_in"
    TRY_TO_PLACE_IN_KWARGS = "try_to_place_in_kwargs"
    ENSURE_OBJECT_BOUNDARY_IN_RANGE = "ensure_object_boundary_in_range"
    ENSURE_VALID_PLACEMENT = "ensure_valid_placement"

    @classmethod
    def contains(cls, keyword: str | None) -> bool:
        """
        :param keyword: The keyword argument name to look up.
        :return: Whether the name is a placement key this module models.
        """
        return keyword in cls._value2member_map_


class RegionKeyword(enum.Enum):
    """
    Keys of a placement's ``sample_region_kwargs``, which select a named sub region of a
    fixture rather than a numeric extent.
    """

    REFERENCE = "ref"
    LOCATION = "loc"
    LOCATIONS = "locs"
    SIDE = "side"
    COMPARTMENT = "compartment"
    REGION_TYPE = "reg_type"
    VERTICAL_RANGE = "z_range"
    RACK_INDEX = "rack_index"
    RACK_LEVEL = "rack_level"
    SLOT_PAIR = "slot_pair"
    TOP_SIZE = "top_size"
    FULL_DEPTH_REGION = "full_depth_region"

    @classmethod
    def contains(cls, keyword: str | None) -> bool:
        """
        :param keyword: The region keyword name to look up.
        :return: Whether the name is a region keyword this module models.
        """
        return keyword in cls._value2member_map_


QUALITATIVE_REGION_KEYWORDS = frozenset(
    {
        RegionKeyword.LOCATION,
        RegionKeyword.LOCATIONS,
        RegionKeyword.SIDE,
        RegionKeyword.COMPARTMENT,
        RegionKeyword.REGION_TYPE,
        RegionKeyword.RACK_INDEX,
        RegionKeyword.RACK_LEVEL,
        RegionKeyword.SLOT_PAIR,
    }
)
"""
Region keywords that name a sub region qualitatively, leaving its extent to be resolved
by the fixture rather than stated in the task.
"""

OBJECT_RELATIVE_EXTENTS = frozenset({"obj", "obj.x", "obj.y"})
"""
Values a placement extent takes when it is derived from the bounding box of whichever
object the scene sampled.
"""


class RepresentationDemand(enum.Enum):
    """
    A property of a placement specification that a product algebra event must
    accommodate.

    Every member's value describes the demand, so reports can explain a classification
    without repeating the description at each use site.
    """

    FIXTURE_FRAME = (
        "Region is axis aligned in a fixture's frame and rigidly transformed by that "
        "fixture's yaw, so it is axis aligned in world coordinates only when the "
        "fixture happens to be."
    )
    SECOND_FRAME = (
        "Region is positioned relative to a second fixture or object, so it cannot be "
        "axis aligned in the same frame as the fixture it rests on."
    )
    ORIENTATION_COUPLING = (
        "Object yaw is sampled over a non degenerate range while its footprint must "
        "stay inside the region, so the admissible positions depend on the orientation."
    )
    SCENE_VALIDITY_COUPLING = (
        "Placement is rejected against the rest of the scene, so the region depends on "
        "where other objects were sampled."
    )
    CONTAINMENT = (
        "Region is the interior of another sampled object rather than a surface extent."
    )
    QUALITATIVE_REGION = (
        "Region is named qualitatively and resolved by the fixture instead of being "
        "stated as a numeric extent."
    )
    VERTICAL_EXTENT = (
        "Region constrains the vertical axis as well, so the event spans three "
        "continuous dimensions."
    )
    OBJECT_RELATIVE_EXTENT = (
        "Region extent is derived from the bounding box of whichever object the scene "
        "sampled."
    )
    SCENE_DEPENDENT_EXTENT = (
        "Region extent or position is computed at scene construction rather than stated "
        "as a literal."
    )

    @property
    def description(self) -> str:
        """:return: Prose describing what this demand asks of the representation."""
        return self.value


@dataclass
class SpecificationValue:
    """
    A value taken from a placement specification.

    Values that RoboCasa computes while building a scene cannot be read from the task
    source, so they are marked scene dependent instead of being guessed.
    """

    literal: Any
    """
    The resolved value, or ``None`` when it is scene dependent.
    """

    is_scene_dependent: bool
    """
    Whether resolving the value requires a constructed scene.
    """

    @classmethod
    def from_expression(cls, expression: ast.expr) -> SpecificationValue:
        """
        Resolve an expression from a task source into a value.

        :param expression: The expression the task assigned to a placement key.
        :return: The resolved value, marked scene dependent when it cannot be read
            statically.
        """
        if isinstance(expression, ast.Constant):
            return cls(expression.value, False)
        if isinstance(expression, (ast.Tuple, ast.List)):
            return cls.from_elements(expression.elts)
        if isinstance(expression, ast.Attribute):
            return cls.from_attribute(expression)
        if isinstance(expression, ast.UnaryOp):
            return cls.from_unary_operation(expression)
        if isinstance(expression, ast.BinOp):
            return cls.from_binary_operation(expression)
        return cls(None, True)

    @classmethod
    def from_elements(cls, elements: list[ast.expr]) -> SpecificationValue:
        """
        :param elements: The elements of a tuple or list valued specification entry.
        :return: A tuple valued specification value, scene dependent when any element
            is.
        """
        resolved = [cls.from_expression(element) for element in elements]
        if any(element.is_scene_dependent for element in resolved):
            return cls(None, True)
        return cls(tuple(element.literal for element in resolved), False)

    @classmethod
    def from_attribute(cls, expression: ast.Attribute) -> SpecificationValue:
        """
        :param expression: An attribute access such as ``np.pi``.
        :return: The constant it names, or a scene dependent value for anything else.
        """
        if expression.attr in RESOLVABLE_CONSTANTS:
            return cls(RESOLVABLE_CONSTANTS[expression.attr], False)
        return cls(None, True)

    @classmethod
    def from_unary_operation(cls, expression: ast.UnaryOp) -> SpecificationValue:
        """
        :param expression: A unary operation, in practice a negation.
        :return: The negated value, or a scene dependent value for other operators.
        """
        operand = cls.from_expression(expression.operand)
        if operand.is_scene_dependent or not isinstance(expression.op, ast.USub):
            return cls(None, True)
        return cls(-operand.literal, False)

    @classmethod
    def from_binary_operation(cls, expression: ast.BinOp) -> SpecificationValue:
        """
        Resolve arithmetic over constants, so yaw ranges written as multiples of pi
        become literals.

        :param expression: A binary arithmetic expression.
        :return: The computed value, or a scene dependent value when an operand or
            operator is not resolvable.
        """
        left = cls.from_expression(expression.left)
        right = cls.from_expression(expression.right)
        if left.is_scene_dependent or right.is_scene_dependent:
            return cls(None, True)
        operators = {
            ast.Add: lambda a, b: a + b,
            ast.Sub: lambda a, b: a - b,
            ast.Mult: lambda a, b: a * b,
            ast.Div: lambda a, b: a / b,
        }
        operator = operators.get(type(expression.op))
        if operator is None:
            return cls(None, True)
        return cls(operator(left.literal, right.literal), False)

    @property
    def numbers(self) -> list[float]:
        """:return: Every number this value contains, flattened out of any nesting."""
        return list(self._flatten(self.literal))

    @classmethod
    def _flatten(cls, value: Any) -> Iterator[float]:
        """
        :param value: A literal, possibly a nested tuple.
        :return: The numbers it contains, in order.
        """
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            yield value
            return
        if isinstance(value, tuple):
            for element in value:
                yield from cls._flatten(element)

    def contains_any(self, candidates: frozenset[str]) -> bool:
        """
        :param candidates: The strings to look for.
        :return: Whether this value is, or contains, one of the candidates.
        """
        if isinstance(self.literal, str):
            return self.literal in candidates
        if isinstance(self.literal, tuple):
            return any(element in candidates for element in self.literal)
        return False


@dataclass
class PlacementSpecification:
    """
    A single ``placement`` specification of a RoboCasa task, together with the demands it
    places on a product algebra representation.
    """

    task: str
    """
    Name of the task class the specification belongs to.
    """

    module: str
    """
    Name of the task module the specification was read from.
    """

    line: int
    """
    Line the specification starts on in its module.
    """

    arguments: dict[PlacementKey, SpecificationValue] = field(default_factory=dict)
    """
    The placement's keyword arguments, keyed by the placement key they set.
    """

    region_keywords: frozenset[RegionKeyword] = frozenset()
    """
    The region keywords the placement's ``sample_region_kwargs`` sets.
    """

    def argument(self, key: PlacementKey) -> SpecificationValue | None:
        """
        :param key: The placement key to read.
        :return: The value the placement assigns to the key, or ``None`` when unset.
        """
        return self.arguments.get(key)

    def flag_is_enabled(self, key: PlacementKey, default: bool) -> bool:
        """
        :param key: The placement key holding a boolean flag.
        :param default: The value RoboCasa assumes when the placement omits the flag.
        :return: Whether the flag is enabled for this placement.
        """
        value = self.argument(key)
        if value is None or value.is_scene_dependent:
            return default
        return bool(value.literal)

    @property
    def rests_on_fixture(self) -> bool:
        """:return: Whether the region is taken from a fixture's reset region."""
        return PlacementKey.FIXTURE in self.arguments

    @property
    def references_second_frame(self) -> bool:
        """:return: Whether the region is positioned relative to a further fixture or
        object."""
        if RegionKeyword.REFERENCE in self.region_keywords:
            return True
        if any(
            key in self.arguments
            for key in (
                PlacementKey.REFERENCE_OBJECT,
                PlacementKey.OBJECT,
                PlacementKey.REUSE_REGION_FROM,
            )
        ):
            return True
        position = self.argument(PlacementKey.POSITION)
        return position is not None and position.contains_any(frozenset({"ref"}))

    @property
    def orientation_is_free(self) -> bool:
        """
        :return: Whether the object's yaw is sampled over a non degenerate range, which
            RoboCasa does unless the placement pins it.
        """
        rotation = self.argument(PlacementKey.ROTATION)
        if rotation is None:
            return DEFAULT_ORIENTATION_RANGE[0] != DEFAULT_ORIENTATION_RANGE[1]
        if rotation.is_scene_dependent:
            return True
        numbers = rotation.numbers
        if len(numbers) < 2:
            return False
        return any(
            lower != upper for lower, upper in zip(numbers[0::2], numbers[1::2])
        )

    @property
    def couples_orientation_and_position(self) -> bool:
        """:return: Whether the admissible positions depend on the sampled
        orientation."""
        return self.orientation_is_free and self.flag_is_enabled(
            PlacementKey.ENSURE_OBJECT_BOUNDARY_IN_RANGE, True
        )

    @property
    def couples_scene_validity(self) -> bool:
        """:return: Whether the region depends on where the rest of the scene was
        sampled."""
        return self.flag_is_enabled(PlacementKey.ENSURE_VALID_PLACEMENT, True)

    @property
    def has_object_relative_extent(self) -> bool:
        """:return: Whether the extent is derived from the sampled object's bounding
        box."""
        size = self.argument(PlacementKey.SIZE)
        return size is not None and size.contains_any(OBJECT_RELATIVE_EXTENTS)

    @property
    def has_scene_dependent_extent(self) -> bool:
        """:return: Whether extent or position is only computed while building a
        scene."""
        return any(
            value.is_scene_dependent
            for key, value in self.arguments.items()
            if key in (PlacementKey.SIZE, PlacementKey.POSITION, PlacementKey.OFFSET)
        )

    @property
    def demands(self) -> frozenset[RepresentationDemand]:
        """:return: Every demand this specification places on the representation."""
        conditions = {
            RepresentationDemand.FIXTURE_FRAME: self.rests_on_fixture,
            RepresentationDemand.SECOND_FRAME: self.references_second_frame,
            RepresentationDemand.ORIENTATION_COUPLING: self.couples_orientation_and_position,
            RepresentationDemand.SCENE_VALIDITY_COUPLING: self.couples_scene_validity,
            RepresentationDemand.CONTAINMENT: PlacementKey.TRY_TO_PLACE_IN
            in self.arguments,
            RepresentationDemand.QUALITATIVE_REGION: bool(
                self.region_keywords & QUALITATIVE_REGION_KEYWORDS
            ),
            RepresentationDemand.VERTICAL_EXTENT: RegionKeyword.VERTICAL_RANGE
            in self.region_keywords,
            RepresentationDemand.OBJECT_RELATIVE_EXTENT: self.has_object_relative_extent,
            RepresentationDemand.SCENE_DEPENDENT_EXTENT: self.has_scene_dependent_extent,
        }
        return frozenset(demand for demand, holds in conditions.items() if holds)

    @property
    def is_a_single_axis_aligned_box(self) -> bool:
        """
        :return: Whether the region is one axis aligned box in world coordinates, which
            a single :class:`~random_events.product_algebra.SimpleEvent` represents
            exactly.
        """
        return not self.demands


@dataclass
class TaskSuiteParser:
    """
    Reads placement specifications out of the RoboCasa kitchen task sources.
    """

    task_root: pathlib.Path
    """
    Directory holding the kitchen environment packages.
    """

    @classmethod
    def from_installed_package(cls) -> TaskSuiteParser:
        """
        :return: A parser pointed at the installed RoboCasa distribution, located without
            importing it.
        """
        specification = importlib.util.find_spec("robocasa")
        assert (
            specification is not None and specification.origin is not None
        ), "RoboCasa is not installed, so its task suite cannot be read."
        root = pathlib.Path(specification.origin).parent / "environments" / "kitchen"
        assert root.is_dir(), f"RoboCasa kitchen tasks are not at {root}."
        return cls(root)

    def parse(self) -> PlacementCorpus:
        """
        :return: Every placement specification stated by the kitchen task suite.
        """
        specifications = [
            specification
            for module in self.task_modules()
            for specification in self.parse_module(module)
        ]
        return PlacementCorpus(specifications)

    def task_modules(self) -> list[pathlib.Path]:
        """:return: The task modules of every package holding task definitions."""
        return sorted(
            path
            for package in TASK_PACKAGES
            for path in (self.task_root / package).rglob("*.py")
            if path.name != "__init__.py"
        )

    def parse_module(self, module: pathlib.Path) -> Iterator[PlacementSpecification]:
        """
        :param module: The task module to read.
        :return: The placement specifications each task class in the module states.
        """
        tree = ast.parse(module.read_text())
        for task in ast.walk(tree):
            if not isinstance(task, ast.ClassDef):
                continue
            for expression in self.placement_expressions(task):
                yield self.parse_placement(expression, task.name, module.stem)

    @staticmethod
    def placement_expressions(task: ast.ClassDef) -> Iterator[ast.Call]:
        """
        :param task: The task class to search.
        :return: Every ``placement=dict(...)`` expression the class states.
        """
        for node in ast.walk(task):
            if not isinstance(node, ast.keyword) or node.arg != "placement":
                continue
            if isinstance(node.value, ast.Call):
                yield node.value

    def parse_placement(
        self, expression: ast.Call, task: str, module: str
    ) -> PlacementSpecification:
        """
        :param expression: The ``dict(...)`` call stating the placement.
        :param task: Name of the task class the placement belongs to.
        :param module: Name of the module the placement was read from.
        :return: The parsed specification.
        """
        arguments = {
            PlacementKey(keyword.arg): SpecificationValue.from_expression(keyword.value)
            for keyword in expression.keywords
            if PlacementKey.contains(keyword.arg)
        }
        region_keywords = self.parse_region_keywords(expression)
        return PlacementSpecification(
            task=task,
            module=module,
            line=expression.lineno,
            arguments=arguments,
            region_keywords=region_keywords,
        )

    @staticmethod
    def parse_region_keywords(expression: ast.Call) -> frozenset[RegionKeyword]:
        """
        :param expression: The ``dict(...)`` call stating the placement.
        :return: The region keywords its ``sample_region_kwargs`` sets.
        """
        for keyword in expression.keywords:
            if keyword.arg != PlacementKey.SAMPLE_REGION_KWARGS.value:
                continue
            if isinstance(keyword.value, ast.Call):
                names = [entry.arg for entry in keyword.value.keywords]
            elif isinstance(keyword.value, ast.Dict):
                names = [
                    key.value for key in keyword.value.keys if isinstance(key, ast.Constant)
                ]
            else:
                names = []
            return frozenset(
                RegionKeyword(name) for name in names if RegionKeyword.contains(name)
            )
        return frozenset()


@dataclass
class DemandFrequency(ExperimentResult):
    """
    How often one representation demand occurs across the task suite.
    """

    demand: str
    """
    Name of the demand.
    """

    specifications: int
    """
    Number of placement specifications raising the demand.
    """

    share_of_corpus: float
    """
    Fraction of all placement specifications raising the demand.
    """


@dataclass
class PlacementCorpus:
    """
    Every placement specification of the RoboCasa kitchen task suite.
    """

    specifications: list[PlacementSpecification]
    """
    The specifications, in the order they were read.
    """

    @property
    def tasks(self) -> set[str]:
        """:return: Names of the task classes stating at least one placement."""
        return {specification.task for specification in self.specifications}

    def demand_counts(self) -> Counter[RepresentationDemand]:
        """:return: How many specifications raise each demand."""
        counts = Counter()
        for specification in self.specifications:
            counts.update(specification.demands)
        return counts

    def demand_frequencies(self) -> ExperimentsTable:
        """:return: The demand distribution as a table, most frequent demand first."""
        total = len(self.specifications)
        return ExperimentsTable(
            [
                DemandFrequency(
                    demand=demand.name.replace("_", " ").title(),
                    specifications=count,
                    share_of_corpus=round(count / total, 3),
                )
                for demand, count in self.demand_counts().most_common()
            ]
        )

    def demands_per_specification(self) -> Counter[int]:
        """:return: How many specifications raise a given number of demands."""
        return Counter(
            len(specification.demands) for specification in self.specifications
        )

    @property
    def exactly_representable(self) -> list[PlacementSpecification]:
        """:return: Specifications a single simple event represents without
        approximation."""
        return [
            specification
            for specification in self.specifications
            if specification.is_a_single_axis_aligned_box
        ]


def main():
    corpus = TaskSuiteParser.from_installed_package().parse()

    print(
        f"{len(corpus.specifications)} placement specifications "
        f"across {len(corpus.tasks)} task classes"
    )
    print(
        f"{len(corpus.exactly_representable)} are a single axis aligned box "
        f"in world coordinates"
    )
    print()
    print(
        TypstRenderer(corpus.demand_frequencies()).render_figure(
            "Demands the RoboCasa kitchen task suite places on a product algebra "
            "representation of where an object may go. Each row counts the placement "
            "specifications raising that demand; a specification usually raises "
            "several, so the shares do not sum to one."
        )
    )
    print()
    print("Demands per specification:")
    for number_of_demands, count in sorted(corpus.demands_per_specification().items()):
        print(f"  {number_of_demands} demands: {count} specifications")
    print()
    for demand in RepresentationDemand:
        print(f"{demand.name}: {demand.description}")


if __name__ == "__main__":
    main()
