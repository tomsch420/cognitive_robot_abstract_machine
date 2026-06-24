from __future__ import annotations

from dataclasses import dataclass, field, fields

import pytest

from krrood.field_metadata import (
    KRROOD_FIELD_METADATA_KEY,
    KrroodFieldMetadata,
    get_krrood_field_metadata,
)


def get_field(cls, name: str):
    for f in fields(cls):
        if f.name == name:
            return f
    raise KeyError(name)


@dataclass
class _ClassWithoutMetadata:
    value: int


@dataclass
class _ClassWithFullMetadata:
    value: int = field(
        default=0,
        metadata={
            KRROOD_FIELD_METADATA_KEY: KrroodFieldMetadata(
                exclude_from_class_diagram=True,
                exclude_from_orm=False,
                exclude_from_serialization=True,
                exclude_from_relation_tracking=False,
                role_taker=True,
            )
        },
    )


@dataclass
class _ClassWithPartialMetadata:
    value: int = field(
        default=0,
        metadata={KRROOD_FIELD_METADATA_KEY: KrroodFieldMetadata(exclude_from_orm=True)},
    )


class TestKrroodFieldMetadataDefaults:
    def test_all_none_by_default(self):
        metadata = KrroodFieldMetadata()
        assert metadata.exclude_from_class_diagram is None
        assert metadata.exclude_from_orm is None
        assert metadata.exclude_from_serialization is None
        assert metadata.exclude_from_relation_tracking is None
        assert metadata.role_taker is None


class TestGetKrroodFieldMetadata:
    def test_returns_default_when_no_metadata_present(self):
        f = get_field(_ClassWithoutMetadata, "value")
        result = get_krrood_field_metadata(f)
        assert isinstance(result, KrroodFieldMetadata)
        assert result == KrroodFieldMetadata()

    def test_returns_attached_metadata(self):
        f = get_field(_ClassWithFullMetadata, "value")
        result = get_krrood_field_metadata(f)
        assert result.exclude_from_class_diagram is True
        assert result.exclude_from_orm is False
        assert result.exclude_from_serialization is True
        assert result.exclude_from_relation_tracking is False
        assert result.role_taker is True

    def test_partial_metadata_preserves_none_for_unset(self):
        f = get_field(_ClassWithPartialMetadata, "value")
        result = get_krrood_field_metadata(f)
        assert result.exclude_from_orm is True
        assert result.exclude_from_class_diagram is None
        assert result.exclude_from_serialization is None
        assert result.exclude_from_relation_tracking is None
        assert result.role_taker is None
