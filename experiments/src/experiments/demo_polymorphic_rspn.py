"""Demo: Polymorphic RSPNs

Shows how a RelationalProbabilisticCircuit handles a class hierarchy where
a single abstract type (PolymorphicSceneItem) has two concrete subtypes
(BookSceneItem, LampSceneItem). The circuit learns type frequencies and
per-type feature distributions, then grounds a query over the abstract type
into a SumUnit mixture with an explicit type variable.

Also demonstrates polymorphic exchangeable parts: a LibraryRoom that contains
a heterogeneous list of items is fitted and grounded.

Run with:
    .venv/bin/python demo_polymorphic_rspn.py
"""
from __future__ import annotations

import math

from krrood.entity_query_language.factories import underspecified
from krrood.ormatic.data_access_objects.helper import to_dao
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import SumUnit

import test.krrood_test.dataset.ormatic_interface  # noqa: F401 — registers DAOs

from test.krrood_test.dataset.example_classes import (
    BookSceneItem,
    KRROODPosition,
    LampSceneItem,
    LibraryRoom,
    PolymorphicSceneItem,
)


def section(title: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")


# ── 1. Fit a polymorphic RPC on mixed item instances ─────────────────────────

section("1. Fit RelationalProbabilisticCircuit on PolymorphicSceneItem")

items = [
    BookSceneItem(size=0.5, pages=200),
    BookSceneItem(size=0.8, pages=350),
    LampSceneItem(size=1.0, brightness=100.0),
    LampSceneItem(size=1.2, brightness=150.0),
]
item_daos = [to_dao(item) for item in items]

model = RelationalProbabilisticCircuit(PolymorphicSceneItem)
model.fit(item_daos)

print(f"  Concrete subtypes learned : {[t.__name__ for t in model.sub_type_circuits]}")
print(f"  Type variable             : {model.type_variable.name}")
print(f"  Type weights (normalised) :", end="")
for cls, log_w in model.log_type_weights.items():
    print(f"  {cls.__name__}={math.exp(log_w):.2f}", end="")
print()
print(f"  Weights sum to 1          : {abs(sum(math.exp(w) for w in model.log_type_weights.values()) - 1.0) < 1e-9}")

# ── 2. Ground a query over the abstract type ─────────────────────────────────

section("2. Ground a query: underspecified(PolymorphicSceneItem)(size=...)")

query = underspecified(PolymorphicSceneItem)(size=...)
query.resolve()

grounded = model.ground(query)

print(f"  Circuit is valid          : {bool(grounded.is_valid())}")
print(f"  Variables in circuit      : {sorted(v.name for v in grounded.variables)}")
print(f"  Contains SumUnit          : {any(isinstance(n, SumUnit) for n in grounded.nodes())}")

sum_units = [n for n in grounded.nodes() if isinstance(n, SumUnit)]
top = next(su for su in sum_units if len(su.subcircuits) == len(model.sub_type_circuits))
print(f"  Mixture components        : {len(top.subcircuits)} (one per concrete subtype)")

# ── 3. LibraryRoom — polymorphic exchangeable part ───────────────────────────

section("3. LibraryRoom with a polymorphic items list")

rooms = [
    LibraryRoom(
        position=KRROODPosition(x=1.0, y=2.0, z=0.0),
        items=[
            BookSceneItem(size=0.5, pages=200),
            LampSceneItem(size=1.0, brightness=100.0),
        ],
    ),
    LibraryRoom(
        position=KRROODPosition(x=3.0, y=4.0, z=0.0),
        items=[
            BookSceneItem(size=0.8, pages=350),
            BookSceneItem(size=0.6, pages=150),
            LampSceneItem(size=1.2, brightness=150.0),
        ],
    ),
]

room_model = RelationalProbabilisticCircuit(LibraryRoom)
room_model.fit([to_dao(r) for r in rooms])

template = room_model.exchangeable_distribution_templates["items"]
print(f"  Polymorphic subtypes in template : {[t.__name__ for t in template.template_distribution.sub_type_circuits]}")

library_query = underspecified(LibraryRoom)(
    position=underspecified(KRROODPosition)(x=..., y=..., z=...),
    items=[
        underspecified(PolymorphicSceneItem)(size=...),
        underspecified(PolymorphicSceneItem)(size=...),
    ],
)
library_query.resolve()

grounded_room = room_model.ground(library_query)

print(f"  Grounded circuit is valid        : {bool(grounded_room.is_valid())}")
item_names = [v.name for v in grounded_room.variables if "items" in v.name]
print(f"  Item-related variables           : {sorted(item_names)}")
print(f"  All variables                    : {sorted(v.name for v in grounded_room.variables)}")

section("Done")
