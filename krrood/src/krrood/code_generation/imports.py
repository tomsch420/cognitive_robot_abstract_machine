"""Import-line generation and inspection of callables for code generation."""

from __future__ import annotations

import inspect
import sys
from collections import defaultdict
from dataclasses import dataclass
from importlib.util import resolve_name
from typing import Callable, Dict, Iterable, List, Optional, Set, Type

from krrood.utils import (
    get_function_import_data,
    get_relative_import,
    is_typing_type,
)

# %%
# Exceptions


class FunctionMissingAnnotationsError(TypeError):
    """Raised at decoration time when a function lacks required type annotations."""


# %%
# Callable inspection


@dataclass
class CallableImport:
    """Import data required to reference a callable from generated source."""

    import_line: str
    """The ``from … import …`` statement that makes the callable available."""

    access_expression: str
    """The name expression used to reference the callable after the import."""


def generate_callable_import(func: Callable) -> CallableImport:
    """Return the :class:`CallableImport` for *func*.

    :param func: The callable to generate an import for.
    :returns: A :class:`CallableImport` bundling the ``from … import …`` line and
        the name expression used to reference the callable after that import.

    Module-level function ``distance`` in ``my.module``::

        CallableImport("from my.module import distance", "distance")

    Method ``MyClass.distance`` in ``my.module``::

        CallableImport("from my.module import MyClass", "MyClass.distance")
    """
    module_name = func.__module__
    qualname = func.__qualname__
    qualname_parts = qualname.split(".")

    parent_segment = qualname_parts[-2] if len(qualname_parts) >= 2 else None
    is_method = (
        parent_segment is not None
        and parent_segment.isidentifier()
        and "<" not in parent_segment
    )

    if is_method:
        class_name = parent_segment
        import_line = f"from {module_name} import {class_name}"
        access_expression = f"{class_name}.{func.__name__}"
    else:
        import_line = f"from {module_name} import {func.__name__}"
        access_expression = func.__name__

    return CallableImport(import_line=import_line, access_expression=access_expression)


def validate_annotations(func: Callable) -> None:
    """Raise :exc:`FunctionMissingAnnotationsError` if any required annotation is absent.

    Unannotated ``self`` and ``cls`` parameters are silently excluded.
    """
    sig = inspect.signature(func)
    for param_name, param in sig.parameters.items():
        if param_name in ("self", "cls"):
            continue
        if param.annotation is inspect.Parameter.empty:
            raise FunctionMissingAnnotationsError(
                f"Parameter '{param_name}' of '{func.__qualname__}' "
                f"lacks a type annotation."
            )
    if sig.return_annotation is inspect.Parameter.empty:
        raise FunctionMissingAnnotationsError(
            f"Function '{func.__qualname__}' lacks a return type annotation."
        )


# %%
# Import extraction / generation


def generate_relative_import(
    from_module: str, target_module: str, symbol: Optional[str] = None
) -> str:
    """Generate a relative import statement using Python's own resolver.

    :param from_module: The module where the import is being made.
    :param target_module: The module to import.
    :param symbol: The symbol (e.g., a class, a method, etc.) to import (optional).
    :returns: A relative import statement string.
    """
    absolute = resolve_name(target_module, from_module)

    from_pkg = from_module.rsplit(".", 1)[0]
    from_parts = from_pkg.split(".")
    target_parts = absolute.split(".")

    i = 0
    while (
        i < min(len(from_parts), len(target_parts)) and from_parts[i] == target_parts[i]
    ):
        i += 1

    up = len(from_parts) - i
    prefix = "." * (up + 1)

    remainder = ".".join(target_parts[i:])

    if symbol:
        if remainder:
            return f"from {prefix}{remainder} import {symbol}"
        return f"from {prefix} import {symbol}"
    else:
        return f"from {prefix} import {remainder}"


def get_type_names_per_module_from_types(
    type_objects: Iterable[Type],
    excluded_names: Optional[List[str]] = None,
    excluded_modules: Optional[List[str]] = None,
) -> Dict[str, List[str]]:
    """Get a dictionary of type names grouped by module.

    :param type_objects: A list of type objects to format.
    :param excluded_names: A list of names to exclude from the imports.
    :param excluded_modules: A list of modules to exclude from the imports.
    :return: A dictionary of type names grouped by module.
    """
    excluded_modules = [] if excluded_modules is None else excluded_modules
    excluded_names = [] if excluded_names is None else excluded_names
    module_to_types: Dict[str, List[str]] = defaultdict(list)
    for type_object in type_objects:
        try:
            if isinstance(type_object, type) or is_typing_type(type_object):
                mod = type_object.__module__
                name = type_object.__qualname__
            elif callable(type_object):
                mod, name = get_function_import_data(type_object)
            elif hasattr(type(type_object), "__module__"):
                mod = type(type_object).__module__
                name = type(type_object).__qualname__
            else:
                continue
            if name == "NoneType":
                mod = "types"
            if (
                mod is None
                or mod == "builtins"
                or mod.startswith("_")
                or mod in sys.builtin_module_names
                or mod in excluded_modules
                or "<" in mod
                or name in excluded_names
                or "site-packages" in mod.split(".")
            ):
                continue
            if mod == "typing":
                mod = "typing_extensions"
            module_to_types[mod].append(name)
        except AttributeError:
            continue
    return module_to_types


def get_imports_from_types(
    type_objects: Iterable[Type],
    target_file_path: Optional[str] = None,
    package_name: Optional[str] = None,
    excluded_names: Optional[List[str]] = None,
    excluded_modules: Optional[List[str]] = None,
) -> List[str]:
    """Format import lines from type objects.

    :param type_objects: A list of type objects to format.
    :param target_file_path: The file path to which the imports should be relative.
    :param package_name: The name of the package to use for relative imports.
    :param excluded_names: A list of names to exclude from the imports.
    :param excluded_modules: A list of modules to exclude from the imports.
    :return: A list of formatted import lines.
    """
    module_to_types = get_type_names_per_module_from_types(
        type_objects, excluded_names, excluded_modules
    )

    lines: List[str] = []
    stem_imports: List[str] = []
    for module, names in module_to_types.items():
        filtered_names: Set[str] = set()
        for name in set(names):
            if "." in name:
                stem = ".".join(name.split(".")[1:])
                name_to_import = name.split(".")[0]
                filtered_names.add(name_to_import)
                stem_imports.append(f"{stem} = {name_to_import}.{stem}")
            else:
                filtered_names.add(name)
        joined = ", ".join(sorted(set(filtered_names)))
        import_path = module
        if (
            (target_file_path is not None)
            and (package_name is not None)
            and (package_name in module)
        ):
            import_path = get_relative_import(
                target_file_path, module_name=module, package_name=package_name
            )
        lines.append(f"from {import_path} import {joined}")
    lines.extend(stem_imports)
    return lines
