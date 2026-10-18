"""
A part locator that answers with the parts a part finding server found for the same
query before, so that the scenes of the tests split their objects as the server would
without asking it.

Run this module, with ``SEMANTIC_DIGITAL_TWIN_PART_FINDING_SERVER_URL`` set, to record
the answers for every object a scene can hold again, for instance after the shape of an
object changed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta
from pathlib import Path

import pytest

from experiments.physical_pick_up.objects import (
    PickUpObject,
    RoboCasaObjectDescription,
)
from experiments.physical_pick_up.scene import PickUpScene
from semantic_digital_twin.adapters.part_finding.client import RemotePartLocator
from semantic_digital_twin.adapters.part_finding.server import PartFindingCache
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
)
from semantic_digital_twin.part_finding.annotation_part_finder import (
    AnnotationPartFinder,
)
from semantic_digital_twin.part_finding.locating import (
    LocatedPart,
    PartLocator,
    PartQuery,
)

# %% answering from recorded answers


@dataclass
class RecordedPartLocator(PartLocator):
    """
    Answers every query with the parts recorded for it, failing the test for a query
    that was never recorded.
    """

    cache: PartFindingCache = field(
        default_factory=lambda: PartFindingCache(
            directory=Path(__file__).resolve().parent / "located_parts"
        )
    )
    """
    Keeps the recorded answers, one file per query.
    """

    recorder: PartLocator | None = None
    """
    Answers the queries that were never recorded, and has its answers recorded; ``None``
    fails the test for such a query instead.
    """

    def locate(self, query: PartQuery) -> list[LocatedPart]:
        key = self.cache.key(query, locator="")
        parts = self.cache.read(key)
        if parts is not None:
            return parts
        if self.recorder is None:
            pytest.fail(
                f"No parts are recorded for the {query.whole_name} queried; record them "
                f"by running {__name__}."
            )
        parts = self.recorder.locate(query)
        self.cache.directory.mkdir(exist_ok=True)
        self.cache.write(key, parts)
        return parts


# %% recording the answers


def record_every_object(timeout: timedelta = timedelta(minutes=10)) -> None:
    """
    Record what the part finding server finds for every object a scene can hold, leaving
    out the RoboCasa objects when their assets are not downloaded.

    :param timeout: How long to wait for the server to answer one query.
    """
    robocasa_assets_downloaded = (
        RoboCasaDatasetLoader().directory / "objects"
    ).is_dir()
    part_finder = AnnotationPartFinder(
        locator=RecordedPartLocator(recorder=RemotePartLocator(timeout=timeout))
    )
    for pick_up_object in PickUpObject:
        if (
            isinstance(pick_up_object.value, RoboCasaObjectDescription)
            and not robocasa_assets_downloaded
        ):
            continue
        PickUpScene(object_description=pick_up_object.value, part_finder=part_finder)


if __name__ == "__main__":
    record_every_object()
