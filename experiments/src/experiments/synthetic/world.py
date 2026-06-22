"""
Synthetic multi-level relational world for RSPN poster experiments.

Hierarchy:
    Building  ──owns──▶  Floor  ──owns──▶  Room  ──owns──▶  Furniture
              ◀──aggs──           ◀──aggs──        ◀──aggs──

Each level carries scalar attributes and a typed enum so the poster
renders enum boxes at every depth.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List

from krrood.entity_query_language.factories import count, count_range, entity, variable
from krrood.parametrization.feature_extraction.aggregations import (
    AggregationStatistic,
    aggregation_statistic,
)
from krrood.symbol_graph.symbol_graph import Symbol


# ── enumerations ──────────────────────────────────────────────────────────────


class BuildingUse(Enum):
    RESIDENTIAL = "residential"
    COMMERCIAL = "commercial"
    INDUSTRIAL = "industrial"


class FloorUse(Enum):
    OFFICE = "office"
    RESIDENTIAL = "residential"
    RETAIL = "retail"


class RoomFunction(Enum):
    OFFICE = "office"
    BEDROOM = "bedroom"
    KITCHEN = "kitchen"
    LIVING_ROOM = "living_room"


class FurnitureCategory(Enum):
    SEATING = "seating"
    STORAGE = "storage"
    SURFACE = "surface"
    LIGHTING = "lighting"


# ── value objects ─────────────────────────────────────────────────────────────


@dataclass
class GeoCoordinate(Symbol):
    """WGS-84 latitude / longitude pair."""

    latitude: float
    longitude: float


@dataclass
class Dimensions(Symbol):
    """Width × depth × height in metres."""

    width: float
    depth: float
    height: float


# ── leaf entity ───────────────────────────────────────────────────────────────


@dataclass
class Furniture(Symbol):
    """A single piece of furniture inside a room."""

    category: FurnitureCategory
    weight: float
    """Mass in kilograms."""

    dimensions: Dimensions


# ── room ──────────────────────────────────────────────────────────────────────


@dataclass
class Room(Symbol):
    """A room on a floor, containing furniture."""

    function: RoomFunction
    area: float
    """Floor area in square metres."""

    furniture: List[Furniture]


@dataclass
class RoomAggregations(AggregationStatistic[Room]):
    """Aggregation statistics for :class:`Room` over its ``furniture`` field."""

    @aggregation_statistic(variable(Room, None).furniture)
    def furniture_count(self) -> int:
        """Total number of furniture items."""
        [c] = count(variable(Furniture, self.instance.furniture)).tolist()
        return c

    @aggregation_statistic(variable(Room, None).furniture)
    def seating_count(self) -> int:
        """Number of seating items (chairs, sofas …)."""
        cat = variable(Furniture, self.instance.furniture).category
        [c] = (
            entity(count_range(cat))
            .where(cat == FurnitureCategory.SEATING)
            .tolist()
        )
        return c

    @aggregation_statistic(variable(Room, None).furniture)
    def storage_count(self) -> int:
        """Number of storage items (shelves, wardrobes …)."""
        cat = variable(Furniture, self.instance.furniture).category
        [c] = (
            entity(count_range(cat))
            .where(cat == FurnitureCategory.STORAGE)
            .tolist()
        )
        return c


# ── floor ─────────────────────────────────────────────────────────────────────


@dataclass
class Floor(Symbol):
    """A floor in a building, containing rooms."""

    use: FloorUse
    elevation: float
    """Height above ground level in metres."""

    rooms: List[Room]


@dataclass
class FloorAggregations(AggregationStatistic[Floor]):
    """Aggregation statistics for :class:`Floor` over its ``rooms`` field."""

    @aggregation_statistic(variable(Floor, None).rooms)
    def room_count(self) -> int:
        """Total number of rooms."""
        [c] = count(variable(Room, self.instance.rooms)).tolist()
        return c

    @aggregation_statistic(variable(Floor, None).rooms)
    def office_room_count(self) -> int:
        """Number of office rooms."""
        fn = variable(Room, self.instance.rooms).function
        [c] = (
            entity(count_range(fn))
            .where(fn == RoomFunction.OFFICE)
            .tolist()
        )
        return c

    @aggregation_statistic(variable(Floor, None).rooms)
    def bedroom_count(self) -> int:
        """Number of bedrooms."""
        fn = variable(Room, self.instance.rooms).function
        [c] = (
            entity(count_range(fn))
            .where(fn == RoomFunction.BEDROOM)
            .tolist()
        )
        return c


# ── building ──────────────────────────────────────────────────────────────────


@dataclass
class Building(Symbol):
    """A building, containing floors."""

    use: BuildingUse
    coordinate: GeoCoordinate
    footprint_area: float
    """Ground footprint in square metres."""

    floors: List[Floor]


@dataclass
class BuildingAggregations(AggregationStatistic[Building]):
    """Aggregation statistics for :class:`Building` over its ``floors`` field."""

    @aggregation_statistic(variable(Building, None).floors)
    def floor_count(self) -> int:
        """Total number of floors."""
        [c] = count(variable(Floor, self.instance.floors)).tolist()
        return c

    @aggregation_statistic(variable(Building, None).floors)
    def residential_floor_count(self) -> int:
        """Number of residential floors."""
        use = variable(Floor, self.instance.floors).use
        [c] = (
            entity(count_range(use))
            .where(use == FloorUse.RESIDENTIAL)
            .tolist()
        )
        return c

    @aggregation_statistic(variable(Building, None).floors)
    def office_floor_count(self) -> int:
        """Number of office floors."""
        use = variable(Floor, self.instance.floors).use
        [c] = (
            entity(count_range(use))
            .where(use == FloorUse.OFFICE)
            .tolist()
        )
        return c
