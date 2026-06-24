from __future__ import annotations

from dataclasses import dataclass, Field

from typing_extensions import Optional

KRROOD_FIELD_METADATA_KEY = "krrood"
"""The key under which :class:`KrroodFieldMetadata` is stored in a field's ``metadata`` dict."""


@dataclass
class KrroodFieldMetadata:
    """Configuration object for krrood-specific field behaviour.

    Place an instance of this class under the :data:`KRROOD_FIELD_METADATA_KEY` key in the
    ``metadata`` dict of a :func:`dataclasses.field` call to override krrood's built-in
    assumptions about that field.

    Each attribute accepts three values:

    - ``None`` — apply the default heuristic (described per attribute).
    - ``True`` — force the described behaviour regardless of heuristics.
    - ``False`` — suppress the described behaviour even when the heuristic would activate it.

    Example::

        @dataclass
        class Robot:
            name: str
            _internal_cache: dict = field(
                default_factory=dict,
                metadata={
                    "krrood": KrroodFieldMetadata(
                        exclude_from_orm=True,
                        exclude_from_serialization=True,
                    )
                },
            )
    """

    exclude_from_class_diagram: Optional[bool] = None
    """
    Control class-diagram visibility.

    Default heuristic: exclude fields whose name starts with ``_`` or whose
    ``init`` flag is ``False``.
    """

    exclude_from_orm: Optional[bool] = None
    """
    Control ORM column/relationship generation.

    Default heuristic: exclude ``_``-prefixed fields, unresolvable generics,
    and ``dict`` fields.
    """

    exclude_from_serialization: Optional[bool] = None
    """
    Control JSON serialisation inclusion.

    Default heuristic: exclude ``_``-prefixed fields.
    """

    exclude_from_relation_tracking: Optional[bool] = None
    """
    Control whether this field participates in the tracked-object relation graph.

    Default heuristic: exclude ``_``-prefixed fields.
    """

    role_taker: Optional[bool] = None
    """
    Override the inferred role-taker status.

    Default heuristic: a field is a role taker when it is a many-to-one
    relationship (non-container, non-builtin type) that is non-optional and
    has no default value.
    """


def get_krrood_field_metadata(field: Field) -> KrroodFieldMetadata:
    """Return the :class:`KrroodFieldMetadata` attached to *field*, or a default instance.

    :param field: A :class:`dataclasses.Field` whose ``metadata`` dict may contain
        a :class:`KrroodFieldMetadata` under :data:`KRROOD_FIELD_METADATA_KEY`.
    :return: The attached :class:`KrroodFieldMetadata`, or a fresh default instance
        when none is present.
    """
    return field.metadata.get(KRROOD_FIELD_METADATA_KEY, KrroodFieldMetadata())
