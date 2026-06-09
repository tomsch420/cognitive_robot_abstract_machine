from dataclasses import dataclass, Field
from typing import List

from krrood.class_diagrams.attribute_introspector import (
    DataclassOnlyIntrospector,
    AttributeIntrospector,
    DiscoveredAttribute,
)


@dataclass
class TypstRow:

    @classmethod
    def introspector(cls) -> AttributeIntrospector:
        return DataclassOnlyIntrospector()

    @classmethod
    def recursive_fields(cls) -> List[DiscoveredAttribute]:
        result = []
        for field_ in cls.introspector().discover(cls):
            if issubclass(field_.field.type, TypstRow):
                result.extend(field_.field.type.recursive_fields())
            else:
                result.append(field_)
        return result

    @classmethod
    def get_column_names(cls) -> list[str]:
        return [field_.field.name for field_ in cls.recursive_fields()]

    def get_column_values(self) -> list[str]:
        return [getattr(self, field_.field.name) for field_ in self.recursive_fields()]

    def render_row(self) -> str:
        return ", ".join([f"[{v}]" for v in self.get_column_values()])


@dataclass
class TypstTable:
    rows: list[TypstRow]

    def __post_init__(self):
        row_types = {type(row) for row in self.rows}
        assert (
            len(row_types) == 1
        ), "Tables can only be constructed over rows that have the same type everywhere."
