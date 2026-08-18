"""
Exceptions raised while incrementally constructing a candidate rule body.
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Type

from krrood.exceptions import DataclassException


@dataclass
class UnknownAttributeError(DataclassException):
    """
    Raised when a dangling-atom traversal names an attribute that is not declared on the
    source variable's static type.
    """

    owner_type: Type
    """
    The static type the attribute was looked up on.
    """

    attribute_name: str
    """
    The attribute name that is not declared on :attr:`owner_type`.
    """

    def error_message(self) -> str:
        return (
            f"'{self.owner_type.__name__}' has no declared attribute "
            f"'{self.attribute_name}'."
        )

    def suggest_correction(self) -> str:
        return "Check the attribute name against the type's declared dataclass fields."


@dataclass
class IncompatibleVariableTypesError(DataclassException):
    """
    Raised when a closing atom would equate two variables whose static types share no
    common subtype, so the equality could never hold.
    """

    type_a: Type
    """
    The static type of the first variable.
    """

    type_b: Type
    """
    The static type of the second variable.
    """

    def error_message(self) -> str:
        return (
            f"'{self.type_a.__name__}' and '{self.type_b.__name__}' share no common "
            f"subtype; an equality between them can never hold."
        )

    def suggest_correction(self) -> str:
        return "Equate variables whose static types are related by subclassing."
