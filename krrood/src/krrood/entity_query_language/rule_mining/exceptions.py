"""
Exceptions raised while incrementally constructing or scoring a candidate rule body.
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


@dataclass
class EmptyRuleBodyError(DataclassException):
    """
    Raised when scoring a candidate rule body that has no conditions yet, so there is no
    most-recently-added atom to compute confidence against.
    """

    def error_message(self) -> str:
        return "Cannot score a candidate rule body with no conditions."

    def suggest_correction(self) -> str:
        return "Apply at least one refinement operator before scoring the rule body."
