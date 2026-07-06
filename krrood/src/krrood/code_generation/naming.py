"""Naming-convention helpers for generated Python identifiers."""

from __future__ import annotations

import re

# %%
# Naming utilities


def str_to_snake_case(snake_str: str) -> str:
    """Convert any string to snake_case.

    :param snake_str: The string to convert.
    :return: The snake_case string.
    """
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", snake_str)
    s1 = re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()
    s1 = re.sub(r"_{2,}", "_", s1)
    s1 = re.sub(r"^_|_$", "", s1)
    return s1


def to_camel_case(name: str) -> str:
    """Convert snake_case to CamelCase. E.g. ``'my_func'`` → ``'MyFunc'``."""
    return "".join(part.capitalize() for part in name.split("_"))


def class_name_to_instance_name(class_name: str) -> str:
    """Convert a CamelCase class name to a lowerCamelCase instance name.

    E.g. ``"Distance"`` → ``"distance"``, ``"MyDistance"`` → ``"myDistance"``.
    """
    return class_name[0].lower() + class_name[1:] if class_name else class_name
