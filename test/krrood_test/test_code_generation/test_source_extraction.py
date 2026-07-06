"""Tests for source extraction in ``krrood.code_generation.source_extraction_utils``."""

from __future__ import annotations

import pytest

from krrood.code_generation.source_extraction_utils import (
    LineSpan,
    extract_class_source,
    extract_function_source,
)
from krrood.exceptions import SourceDataNotProvided

SOURCE = """
def alpha(x: int) -> int:
    return x


def beta(y: int) -> int:
    return y


class Gamma:
    value: int
"""


class TestExtractFunctionSource:
    """Guarantees for :func:`extract_function_source`."""

    def test_extracts_named_function(self):
        result = extract_function_source(["alpha"], source=SOURCE)
        assert [definition.name for definition in result.definitions] == ["alpha"]
        assert "def alpha(x: int) -> int:" in result.source_of("alpha")

    def test_extracts_all_when_names_empty(self):
        result = extract_function_source([], source=SOURCE)
        assert [definition.name for definition in result.definitions] == [
            "alpha",
            "beta",
        ]

    def test_does_not_extract_classes(self):
        result = extract_function_source(["Gamma"], source=SOURCE)
        assert result.definitions == []

    def test_line_span_is_reported(self):
        result = extract_function_source(["beta"], source=SOURCE)
        line_span = result.definitions[0].line_span
        assert isinstance(line_span, LineSpan)
        assert line_span.start_line <= line_span.end_line

    def test_exclude_signature(self):
        result = extract_function_source(
            ["alpha"], source=SOURCE, include_signature=False
        )
        assert "def alpha" not in result.source_of("alpha")
        assert "return x" in result.source_of("alpha")

    def test_join_lines_false_returns_list(self):
        result = extract_function_source(["alpha"], source=SOURCE, join_lines=False)
        assert isinstance(result.source_of("alpha"), list)

    def test_reads_from_file(self, tmp_path):
        file_path = tmp_path / "module.py"
        file_path.write_text(SOURCE)
        result = extract_function_source(["alpha"], file_path=str(file_path))
        assert "def alpha" in result.source_of("alpha")

    def test_missing_source_raises(self):
        with pytest.raises(SourceDataNotProvided):
            extract_function_source(["alpha"])


class TestExtractClassSource:
    """Guarantees for :func:`extract_class_source`."""

    def test_extracts_named_class(self):
        result = extract_class_source(["Gamma"], source=SOURCE)
        assert [definition.name for definition in result.definitions] == ["Gamma"]
        assert "class Gamma:" in result.source_of("Gamma")

    def test_does_not_extract_functions(self):
        result = extract_class_source(["alpha"], source=SOURCE)
        assert result.definitions == []
