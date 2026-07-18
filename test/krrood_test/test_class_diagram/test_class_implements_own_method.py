from __future__ import annotations

from krrood.class_diagrams.utils import class_implements_own_method


class MethodOwner:
    """
    Base defining a plain method, a classmethod, and a staticmethod to override.
    """

    def plain_method(self) -> str:
        return "base"

    @classmethod
    def class_method(cls) -> str:
        return "base"

    @staticmethod
    def static_method() -> str:
        return "base"


class OverridesEveryKind(MethodOwner):
    """
    Overrides every kind of method, matching the base's descriptor kind.
    """

    def plain_method(self) -> str:
        return "override"

    @classmethod
    def class_method(cls) -> str:
        return "override"

    @staticmethod
    def static_method() -> str:
        return "override"


class InheritsEveryKind(MethodOwner):
    """
    Inherits every method unchanged.
    """


def test_reports_overridden_plain_method():
    assert class_implements_own_method(OverridesEveryKind, MethodOwner, "plain_method")


def test_reports_overridden_classmethod():
    assert class_implements_own_method(OverridesEveryKind, MethodOwner, "class_method")


def test_reports_overridden_staticmethod():
    assert class_implements_own_method(OverridesEveryKind, MethodOwner, "static_method")


def test_reports_inherited_plain_method_as_not_overridden():
    assert not class_implements_own_method(
        InheritsEveryKind, MethodOwner, "plain_method"
    )


def test_reports_inherited_classmethod_as_not_overridden():
    assert not class_implements_own_method(
        InheritsEveryKind, MethodOwner, "class_method"
    )


def test_reports_inherited_staticmethod_as_not_overridden():
    assert not class_implements_own_method(
        InheritsEveryKind, MethodOwner, "static_method"
    )


def test_base_class_does_not_override_itself():
    assert not class_implements_own_method(MethodOwner, MethodOwner, "plain_method")
