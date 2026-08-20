"""
Mine relational rules from real PartNet-Mobility models and report what they say about
the dataset's own annotations.

PartNet labels a link twice and the two labels disagree: ``semantics.txt`` calls a link
``drawer`` or ``rotation_door``, while ``mobility_v2.json``'s part hierarchy calls it
``drawer``, ``drawer_box``, ``cabinet_door`` or even ``handle``. This run mines patterns
over links and reports, for each, how the ``semantics.txt`` labels of the links it
matches are distributed — which is what makes a pattern readable as evidence about the
alignment rather than an opaque score.

Run it through the neem-4 container, which mounts the corpus read-only::

    RemoteMiningJob(configuration=..., module=__name__, arguments=["--model-count", "40"])
"""

from __future__ import annotations

import argparse
import collections
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

from typing_extensions import Dict, List, Sequence

from krrood.entity_query_language.rule_mining.candidate_rule import CandidateRuleBody
from krrood.entity_query_language.rule_mining.miner import RuleMiner, SeedDomain
from krrood.entity_query_language.rule_mining.scoring import ScoreThresholds
from semantic_digital_twin.adapters.partnet_mobility_dataset.domain_model import (
    HANDLE_PART_NAME,
    PartNetLink,
    PartNetModel,
    PartNetMotionKind,
    PartNetPart,
    StorageFurnitureLabel,
    UrdfJointType,
)

DATASET_DIRECTORY_VARIABLE_NAME = "PARTNET_MOBILITY_DATASET_DIRECTORY"
"""
Environment variable naming the corpus location.
"""

CATEGORY = "StorageFurniture"
"""
The category this run is scoped to.
"""

METADATA_DIRECTORY_NAME = "partnet_meta"
"""
The sibling directory holding the dataset's own id and category index.
"""

CATEGORY_INDEX_FILE_NAME = "all_ids.txt"
"""
The ``id,category`` index inside the metadata directory.
"""

# %% loading a slice of the corpus


def storage_furniture_model_ids(dataset_directory: Path, limit: int) -> List[int]:
    """
    :param dataset_directory: The corpus location.
    :param limit: The greatest number of ids to return.
    :return: Ids of models in :data:`CATEGORY` that are actually present on disk.

    .. note::
        The index lists more ids than the corpus holds — it covers models that are not
        well formed and were never unpacked — so presence on disk is checked here.
    """
    index_file = (
        dataset_directory.parent / METADATA_DIRECTORY_NAME / CATEGORY_INDEX_FILE_NAME
    )
    model_ids = []
    for line in index_file.read_text().splitlines():
        identifier, _, category = line.partition(",")
        if category != CATEGORY:
            continue
        if not (dataset_directory / identifier).is_dir():
            continue
        model_ids.append(int(identifier))
        if len(model_ids) >= limit:
            break
    return model_ids


def load_models(
    dataset_directory: Path, model_ids: Sequence[int]
) -> List[PartNetModel]:
    """
    :param dataset_directory: The corpus location.
    :param model_ids: The models to load.
    :return: One :class:`PartNetModel` per id.

    .. note::
        Read straight from the dataset files rather than from a loaded world: building
        a world runs convex decomposition over every mesh, which dominates the runtime
        and contributes nothing a mined rule can be about.
    """
    return [
        PartNetModel.from_dataset(
            model_directory=dataset_directory / str(model_id), model_id=model_id
        )
        for model_id in model_ids
    ]


# %% reporting what a mined pattern says about the labels


@dataclass
class LabelDistribution:
    """
    How the links a pattern matches are labelled by ``semantics.txt``.

    The miner scores rule bodies, not implications, so this is what turns a scored
    pattern into a statement about the dataset's own annotations.
    """

    counts: Dict[str, int] = field(default_factory=dict)
    """
    Number of matched links per label.
    """

    @classmethod
    def of(cls, links: Sequence[PartNetLink]) -> LabelDistribution:
        """
        :param links: The links a pattern matched.
        :return: Their label distribution.
        """
        return cls(
            counts=dict(collections.Counter(link.semantic_label for link in links))
        )

    @property
    def total(self) -> int:
        """
        :return: How many links were matched.
        """
        return sum(self.counts.values())

    def share_of(self, label: str) -> float:
        """
        :param label: The label to measure.
        :return: The fraction of matched links carrying it, zero when nothing matched.
        """
        if not self.total:
            return 0.0
        return self.counts.get(label, 0) / self.total

    def describe(self) -> str:
        """
        :return: The distribution, most common label first.
        """
        ordered = sorted(self.counts.items(), key=lambda item: -item[1])
        return ", ".join(
            f"{label} {100 * count / self.total:.0f}%" for label, count in ordered
        )


def describe_rule(body: CandidateRuleBody) -> str:
    """
    :param body: A mined rule body.
    :return: Its conditions, as EQL renders them.
    """
    return " AND ".join(str(condition) for condition in body.conditions)


# %% the run


def mine_over(models: Sequence[PartNetModel]) -> List[CandidateRuleBody]:
    """
    :param models: The models to mine over.
    :return: Rule bodies over :class:`PartNetLink` meeting the thresholds.
    """
    links = [link for model in models for link in model.links]
    parts = [part for link in links for part in link.parts]
    return RuleMiner(
        thresholds=ScoreThresholds(minimum_support=5, minimum_confidence=0.05),
        maximum_atoms=3,
    ).mine(
        PartNetLink,
        links,
        auxiliary_domains=[SeedDomain(entity_type=PartNetPart, instances=parts)],
        candidate_values={
            "motion_kind": list(PartNetMotionKind),
            "joint_type": list(UrdfJointType),
            "name": [HANDLE_PART_NAME],
            "part_name": [
                StorageFurnitureLabel.DRAWER.value,
                "drawer_box",
                "drawer_front",
                "cabinet_door",
            ],
        },
    )


def report(bodies: Sequence[CandidateRuleBody], link_count: int) -> None:
    """
    Print each mined pattern with its score and label distribution, most drawer-like
    first.

    :param bodies: The mined rule bodies.
    :param link_count: How many links were mined over.
    """
    rows = []
    for body in bodies:
        matched = list(body.to_query().evaluate())
        distribution = LabelDistribution.of(matched)
        rows.append(
            (distribution.share_of(StorageFurnitureLabel.DRAWER), body, distribution)
        )
    rows.sort(key=lambda row: -row[0])

    print(f"\nmined {len(bodies)} rule(s) over {link_count} links\n")
    for drawer_share, body, distribution in rows:
        score = body.score()
        print(f"  support={score.support:<5d} confidence={score.confidence:.2f}")
        print(f"    rule:   {describe_rule(body)}")
        print(f"    labels: {distribution.describe()}")
        print(
            f"    -> {100 * drawer_share:.0f}% of matched links are PartNet drawers\n"
        )


def main(argument_values: Sequence[str]) -> int:
    """
    :param argument_values: Command line arguments.
    :return: The process exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model-count",
        type=int,
        default=40,
        help="how many StorageFurniture models to mine over",
    )
    arguments = parser.parse_args(argument_values)

    dataset_directory = os.environ.get(DATASET_DIRECTORY_VARIABLE_NAME)
    if dataset_directory is None:
        print(f"{DATASET_DIRECTORY_VARIABLE_NAME} is not set", file=sys.stderr)
        return 2

    directory = Path(dataset_directory)
    model_ids = storage_furniture_model_ids(directory, arguments.model_count)
    print(f"loading {len(model_ids)} {CATEGORY} models ...", flush=True)
    models = load_models(directory, model_ids)
    links = [link for model in models for link in model.links]

    report(mine_over(models), len(links))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
