from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import List, Optional

from krrood.symbol_graph.symbol_graph import Symbol


@dataclass(eq=False)
class Handle(Symbol):
    """
    A handle mounted on a container.

    Declared ``eq=False``, matching :class:`~krrood.symbol_graph.symbol_graph.Symbol`'s
    own default: :attr:`container` and :class:`Container`'s :attr:`Container.handles`
    reference each other, so field-based equality would recurse across that cycle.
    Identity equality (inherited from ``object``) is what every test in this fixture
    already relies on.
    """

    name: str
    """The handle's identifying name."""

    container: Optional[Container] = field(default=None, repr=False)
    """
    The container this handle is mounted on, if any.
    """


@dataclass(eq=False)
class Container(Symbol):
    """
    A container that may hold handles.

    Declared ``eq=False`` for the same reason as :class:`Handle`.
    """

    name: str
    """
    The container's identifying name.
    """

    handles: List[Handle] = field(default_factory=list)
    """
    The handles mounted on this container.
    """
