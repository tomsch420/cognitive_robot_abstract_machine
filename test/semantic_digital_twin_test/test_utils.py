from krrood.adapters.json_serializer import get_full_class_name
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.utils import (
    PackageNotFoundError,
    hsrb_installed,
    tracy_installed,
    type_string_to_type,
)


def test_type_string_to_string():
    original_class = Handle
    original_class_name = get_full_class_name(original_class)

    converted_class = type_string_to_type(original_class_name)

    assert converted_class == original_class


def test_package_not_found_error_fallback_is_a_valid_exception_type():
    assert isinstance(PackageNotFoundError, type)
    assert issubclass(PackageNotFoundError, BaseException)


def test_hsrb_installed_returns_a_boolean_instead_of_raising():
    assert isinstance(hsrb_installed(), bool)


def test_tracy_installed_returns_a_boolean_instead_of_raising():
    assert isinstance(tracy_installed(), bool)
