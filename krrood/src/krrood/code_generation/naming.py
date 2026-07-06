"""Naming-convention helpers for generated Python identifiers."""

from __future__ import annotations

import re

# %%
# Naming utilities


def to_snake_case(name: str) -> str:
    """Convert any string to snake_case.

    :param name: The string to convert.
    :return: The snake_case string.
    """
    converted = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    converted = re.sub("([a-z0-9])([A-Z])", r"\1_\2", converted).lower()
    converted = re.sub(r"_{2,}", "_", converted)
    converted = re.sub(r"^_|_$", "", converted)
    return converted


def to_camel_case(name: str) -> str:
    """Convert snake_case to CamelCase. E.g. ``'my_func'`` → ``'MyFunc'``."""
    return "".join(part.capitalize() for part in name.split("_"))


def camel_case_to_lower_camel_case(name: str) -> str:
    """Convert a CamelCase name to a lowerCamelCase name.

    E.g. ``"Distance"`` → ``"distance"``, ``"MyDistance"`` → ``"myDistance"``.
    """
    return name[0].lower() + name[1:] if name else name
