"""
Classes used to verify that ORMatic can store an object in a field typed with a mixin
that the object's class inherits next to the base it is stored as.

Kept in their own module so the generated interface can import them by name.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass

# %% a mixin every subclass inherits next to its primary base


@dataclass(kw_only=True)
class HasPairOfJaws(ABC):
    """
    Something with two jaws that close on what lies between them, abstract only by
    listing :class:`ABC` among its bases.
    """

    jaw_gap: float
    """
    How far apart the open jaws stand, in metres.
    """


@dataclass(kw_only=True)
class HandTool:
    """
    A tool held in one hand, the primary base of every tool with jaws.
    """

    tool_name: str
    """
    What the tool is called.
    """


@dataclass(kw_only=True)
class Pliers(HandTool, HasPairOfJaws):
    """
    A hand tool with short jaws.
    """


@dataclass(kw_only=True)
class Tongs(HandTool, HasPairOfJaws):
    """
    A hand tool with long jaws.
    """


@dataclass(kw_only=True)
class JawsInUse:
    """
    Refers to tools only by the mixin they share.
    """

    closing: HasPairOfJaws
    """
    The tool whose jaws close now.
    """

    waiting: list[HasPairOfJaws]
    """
    The tools that wait their turn.
    """


# %% a mixin that can be made itself


@dataclass(kw_only=True)
class HasHandle:
    """
    Something with a handle, which can be made without being anything else.
    """

    handle_length: float
    """
    How long the handle is, in metres.
    """


@dataclass(kw_only=True)
class Hammer(HandTool, HasHandle):
    """
    A hand tool with a handle.
    """


@dataclass(kw_only=True)
class HandleInUse:
    """
    Refers to something with a handle only by that mixin.
    """

    held: HasHandle
    """
    What is held by its handle.
    """
