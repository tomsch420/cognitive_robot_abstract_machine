"""
Generate a poster plot for an RSPN fitted on a synthetic Building world.

Run with:
    <venv>/bin/python experiments/scripts/synthetic_poster.py
"""

from __future__ import annotations

import random

# Import the ORM interface FIRST so DAOs are registered before to_dao() is called
import experiments.synthetic.ormatic_interface  # noqa: F401  # type: ignore

from experiments.synthetic.world import (
    Building,
    BuildingUse,
    Dimensions,
    Floor,
    FloorUse,
    Furniture,
    FurnitureCategory,
    GeoCoordinate,
    Room,
    RoomFunction,
)
from krrood.ormatic.data_access_objects.helper import to_dao
from probabilistic_model.probabilistic_circuit.relational.poster import RSPNPosterPlotter
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)

_RNG = random.Random(42)


def _random_furniture() -> Furniture:
    return Furniture(
        category=_RNG.choice(list(FurnitureCategory)),
        weight=_RNG.uniform(2.0, 120.0),
        dimensions=Dimensions(
            width=_RNG.uniform(0.3, 3.0),
            depth=_RNG.uniform(0.3, 3.0),
            height=_RNG.uniform(0.4, 2.5),
        ),
    )


def _random_room() -> Room:
    n_furniture = _RNG.randint(1, 6)
    return Room(
        function=_RNG.choice(list(RoomFunction)),
        area=_RNG.uniform(8.0, 60.0),
        furniture=[_random_furniture() for _ in range(n_furniture)],
    )


def _random_floor(use: FloorUse, elevation: float) -> Floor:
    n_rooms = _RNG.randint(2, 8)
    return Floor(
        use=use,
        elevation=elevation,
        rooms=[_random_room() for _ in range(n_rooms)],
    )


def _random_building() -> Building:
    building_use = _RNG.choice(list(BuildingUse))
    n_floors = _RNG.randint(2, 10)
    floor_uses = [_RNG.choice(list(FloorUse)) for _ in range(n_floors)]
    floors = [
        _random_floor(fu, i * _RNG.uniform(2.8, 4.5))
        for i, fu in enumerate(floor_uses)
    ]
    return Building(
        use=building_use,
        coordinate=GeoCoordinate(
            latitude=_RNG.uniform(48.0, 54.0),
            longitude=_RNG.uniform(8.0, 15.0),
        ),
        footprint_area=_RNG.uniform(100.0, 5000.0),
        floors=floors,
    )


def main() -> None:
    buildings = [_random_building() for _ in range(40)]
    building_daos = [to_dao(b) for b in buildings]

    rspn = RelationalProbabilisticCircuit(Building)
    rspn.fit(building_daos)

    plotter = RSPNPosterPlotter(rspn)
    output_path = "synthetic_building_poster.png"
    plotter.save(output_path)
    print(f"Poster saved to {output_path}")


if __name__ == "__main__":
    main()
