from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import List, Optional

from krrood.symbol_graph.symbol_graph import Symbol


@dataclass
class Handle(Symbol):
    """
    A handle mounted on a container.
    """

    name: str
    """The handle's identifying name."""

    container: Optional[Container] = field(default=None, repr=False)
    """
    The container this handle is mounted on, if any.
    """

    def __hash__(self) -> int:
        return hash((self.__class__.__name__, self.name))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Handle):
            return False
        return self.name == other.name


@dataclass
class Container(Symbol):
    """
    A container that may hold handles.
    """

    name: str
    """
    The container's identifying name.
    """

    handles: List[Handle] = field(default_factory=list)
    """
    The handles mounted on this container.
    """

    def __hash__(self) -> int:
        return hash((self.__class__.__name__, self.name))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Container):
            return False
        return self.name == other.name
