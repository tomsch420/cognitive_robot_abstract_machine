from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional
from krrood.symbol_graph.symbol_graph import Symbol
from krrood.parametrization.feature_extraction.aggregations import (
    AggregationStatistic,
    aggregation_statistic,
)
from krrood.entity_query_language.factories import variable, count, entity, count_range


@dataclass(unsafe_hash=True)
class Position(Symbol):
    x: float
    y: float
    z: float


@dataclass
class Address(Symbol):
    street: str
    number: int


@dataclass
class Dimensions(Symbol):
    width: float
    length: float
    height: float


class FurnitureType(Enum):
    CHAIR = "chair"
    TABLE = "table"
    BED = "bed"
    DESK = "desk"


@dataclass
class CampusDrawer(Symbol):
    items_count: int
    label: str


@dataclass
class Furniture(Symbol):
    type: FurnitureType
    position: Position
    drawers: List[CampusDrawer] = field(default_factory=list)


class RoomType(Enum):
    OFFICE = "office"
    BEDROOM = "bedroom"
    KITCHEN = "kitchen"


@dataclass
class Room(Symbol):
    furniture: List[Furniture]
    purpose: RoomType
    dimensions: Dimensions


@dataclass
class Building(Symbol):
    rooms: List[Room]
    floors: int
    address: Address


@dataclass
class Campus(Symbol):
    buildings: List[Building]
    location: Position


# Aggregations
@dataclass
class FurnitureAggregations(AggregationStatistic[Furniture]):
    @aggregation_statistic("drawers")
    def total_items(self) -> int:
        res = (
            entity(variable(CampusDrawer, self.instance.drawers).items_count)
            .sum()
            .tolist()
        )
        return res[0] if res else 0


@dataclass
class RoomAggregations(AggregationStatistic[Room]):
    @aggregation_statistic("furniture")
    def chair_count(self) -> int:
        type_var = variable(Furniture, self.instance.furniture).type
        res = (
            entity(count_range(type_var))
            .where(type_var == FurnitureType.CHAIR)
            .tolist()
        )
        return res[0] if res else 0

    @aggregation_statistic("furniture")
    def table_count(self) -> int:
        type_var = variable(Furniture, self.instance.furniture).type
        res = (
            entity(count_range(type_var))
            .where(type_var == FurnitureType.TABLE)
            .tolist()
        )
        return res[0] if res else 0


@dataclass
class BuildingAggregations(AggregationStatistic[Building]):
    @aggregation_statistic("rooms")
    def office_count(self) -> int:
        type_var = variable(Room, self.instance.rooms).purpose
        res = (
            entity(count_range(type_var)).where(type_var == RoomType.OFFICE).tolist()
        )
        return res[0] if res else 0

    @aggregation_statistic("rooms")
    def bedroom_count(self) -> int:
        type_var = variable(Room, self.instance.rooms).purpose
        res = (
            entity(count_range(type_var)).where(type_var == RoomType.BEDROOM).tolist()
        )
        return res[0] if res else 0


@dataclass
class CampusAggregations(AggregationStatistic[Campus]):
    @aggregation_statistic("buildings")
    def building_count(self) -> int:
        res = count(variable(Building, self.instance.buildings)).tolist()
        return res[0] if res else 0
