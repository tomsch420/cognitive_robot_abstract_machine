"""
The datasets of the circuit learning benchmark: the twenty binary density estimation
datasets and the UCI datasets of the joint probability tree paper, downloaded on first
use and kept in a local cache.
"""

from __future__ import annotations

# %% imports
import io
import urllib.request
import zipfile
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum, StrEnum
from pathlib import Path

import numpy as np
import pandas as pd
from random_events.set import Set
from random_events.variable import Continuous, Symbolic, Variable
from scipy.io import arff
from sklearn.datasets import fetch_openml
from sortedcontainers import SortedSet


# %% splits


@dataclass(kw_only=True)
class BenchmarkDataset:
    """
    A dataset split into training, validation and test rows, with symbolic columns as
    strings and continuous columns standardized by the training rows.
    """

    name: str
    """
    The name of the dataset.
    """

    train: pd.DataFrame
    """
    The training rows.
    """

    validation: pd.DataFrame
    """
    The validation rows.
    """

    test: pd.DataFrame
    """
    The test rows.
    """

    variables: SortedSet = field(init=False)
    """
    One variable per column: symbolic over the values of all splits, or continuous.
    """

    def __post_init__(self) -> None:
        """
        Infer :attr:`variables` from the values of all splits.
        """
        frames = pd.concat([self.train, self.validation, self.test])
        variables: list[Variable] = []
        for column in self.train.columns:
            if pd.api.types.is_float_dtype(self.train[column]):
                variables.append(Continuous(column))
            else:
                values = sorted(frames[column].unique())
                variables.append(
                    Symbolic(name=column, domain=Set.from_iterable(values))
                )
        self.variables = SortedSet(variables)

    @property
    def is_binary(self) -> bool:
        """
        :return: Whether every variable is symbolic with at most two values.
        """
        return all(
            isinstance(variable, Symbolic) and len(variable.domain.simple_sets) <= 2
            for variable in self.variables
        )

    def encoded(self, *, frame: pd.DataFrame) -> np.ndarray:
        """
        :param frame: Rows of one of the splits.
        :return: The rows with every symbolic value replaced by the hash of its domain
            element, the columns in the order of :attr:`variables`.
        """
        columns = []
        for variable in self.variables:
            values = frame[variable.name]
            if isinstance(variable, Symbolic):
                columns.append(values.map(hash).to_numpy(dtype=float))
            else:
                columns.append(values.to_numpy(dtype=float))
        return np.stack(columns, axis=1)


# %% sources


@dataclass(kw_only=True)
class DatasetSource(ABC):
    """
    Where the rows of a dataset come from.
    """

    cache_directory: Path
    """
    The directory downloaded files are kept in.
    """

    @abstractmethod
    def load(self) -> BenchmarkDataset:
        """
        :return: The dataset, downloaded if it is not in the cache yet.
        """
        raise NotImplementedError

    def cached_bytes(self, *, url: str, filename: str) -> bytes:
        """
        :param url: The address of the file.
        :param filename: The name of the file in the cache.
        :return: The content of the file, downloaded on first use.
        """
        path = self.cache_directory / filename
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            with urllib.request.urlopen(url) as response:
                path.write_bytes(response.read())
        return path.read_bytes()


class BinaryDatasetName(StrEnum):
    """
    The twenty binary density estimation datasets, as the repository of
    :cite:`liu2021tractable` names them.
    """

    ACCIDENTS = "accidents"
    AD = "ad"
    AUDIO = "baudio"
    BBC = "bbc"
    NETFLIX = "bnetflix"
    BOOK = "book"
    NEWSGROUPS = "c20ng"
    REUTERS = "cr52"
    WEBKB = "cwebkb"
    DNA = "dna"
    JESTER = "jester"
    KDD = "kdd"
    KOSAREK = "kosarek"
    MSNBC = "msnbc"
    MSWEB = "msweb"
    NLTCS = "nltcs"
    PLANTS = "plants"
    PUMSB_STAR = "pumsb_star"
    EACH_MOVIE = "tmovie"
    RETAIL = "tretail"


class BinarySplit(StrEnum):
    """
    The splits of a binary dataset, as their files are named.
    """

    TRAIN = "train"
    VALIDATION = "valid"
    TEST = "test"


@dataclass(kw_only=True)
class BinaryDatasetSource(DatasetSource):
    """
    A binary dataset of the Density-Estimation-Datasets repository, with its own splits.
    """

    dataset: BinaryDatasetName
    """
    The dataset.
    """

    repository_url: str = (
        "https://raw.githubusercontent.com/UCLA-StarAI/Density-Estimation-Datasets/"
        "master/datasets"
    )
    """
    The address of the directory of the datasets.
    """

    def load(self) -> BenchmarkDataset:
        frames = {}
        for split in BinarySplit:
            filename = f"{self.dataset}.{split}.data"
            content = self.cached_bytes(
                url=f"{self.repository_url}/{self.dataset}/{filename}",
                filename=f"binary/{filename}",
            )
            values = np.loadtxt(io.BytesIO(content), delimiter=",", dtype=int)
            frames[split] = pd.DataFrame(
                values.astype(str),
                columns=[f"x{index:04d}" for index in range(values.shape[1])],
            )
        return BenchmarkDataset(
            name=str(self.dataset),
            train=frames[BinarySplit.TRAIN],
            validation=frames[BinarySplit.VALIDATION],
            test=frames[BinarySplit.TEST],
        )


class MixedDatasetName(StrEnum):
    """
    The UCI datasets the joint probability tree paper :cite:`nyga2023joint` evaluates
    on, table 3.
    """

    IRIS = "iris"
    ADULT = "adult"
    DRY_BEAN = "dry_bean"
    WINE = "wine"
    WINE_QUALITY = "wine_quality"
    BANK_MARKETING = "bank_marketing"
    CAR_EVALUATION = "car_evaluation"
    ABALONE = "abalone"


class OpenMLIdentifier(IntEnum):
    """
    The OpenML data identifiers of the mixed datasets that OpenML hosts.
    """

    IRIS = 61
    ADULT = 1590
    WINE = 187
    WINE_QUALITY = 40691
    BANK_MARKETING = 1461
    CAR_EVALUATION = 40975
    ABALONE = 183


@dataclass(kw_only=True)
class MixedDatasetSource(DatasetSource):
    """
    A UCI dataset with symbolic and continuous columns, split at random.
    """

    dataset: MixedDatasetName
    """
    The dataset.
    """

    validation_fraction: float = 0.1
    """
    The fraction of the rows that become validation rows.
    """

    test_fraction: float = 0.2
    """
    The fraction of the rows that become test rows.
    """

    seed: int = 0
    """
    The seed of the split.
    """

    dry_bean_url: str = (
        "https://archive.ics.uci.edu/static/public/602/dry+bean+dataset.zip"
    )
    """
    The address of the archive of the dry bean dataset, which OpenML does not host.
    """

    def load(self) -> BenchmarkDataset:
        frame = self.raw_frame().dropna().reset_index(drop=True)
        for column in frame.columns:
            if pd.api.types.is_numeric_dtype(frame[column]):
                frame[column] = frame[column].astype(float)
            else:
                frame[column] = frame[column].astype(str)
        order = np.random.default_rng(self.seed).permutation(len(frame))
        number_of_test_rows = int(self.test_fraction * len(frame))
        number_of_validation_rows = int(self.validation_fraction * len(frame))
        test = frame.iloc[order[:number_of_test_rows]]
        validation = frame.iloc[
            order[number_of_test_rows : number_of_test_rows + number_of_validation_rows]
        ]
        train = frame.iloc[order[number_of_test_rows + number_of_validation_rows :]]
        continuous = [
            column
            for column in frame.columns
            if pd.api.types.is_float_dtype(frame[column])
        ]
        mean = train[continuous].mean()
        scale = train[continuous].std().replace(0.0, 1.0)
        splits = []
        for split in (train, validation, test):
            split = split.copy()
            split[continuous] = (split[continuous] - mean) / scale
            splits.append(split.reset_index(drop=True))
        return BenchmarkDataset(
            name=str(self.dataset),
            train=splits[0],
            validation=splits[1],
            test=splits[2],
        )

    def raw_frame(self) -> pd.DataFrame:
        """
        :return: All rows of the dataset, as published.
        """
        if self.dataset is MixedDatasetName.DRY_BEAN:
            archive = zipfile.ZipFile(
                io.BytesIO(
                    self.cached_bytes(url=self.dry_bean_url, filename="dry_bean.zip")
                )
            )
            content = archive.read("DryBeanDataset/Dry_Bean_Dataset.arff").decode()
            records, _ = arff.loadarff(io.StringIO(content))
            frame = pd.DataFrame(records)
            for column in frame.columns:
                if frame[column].dtype == object:
                    frame[column] = frame[column].str.decode("utf-8")
            return frame
        bunch = fetch_openml(
            data_id=int(OpenMLIdentifier[self.dataset.name]),
            as_frame=True,
            data_home=str(self.cache_directory / "openml"),
            parser="liac-arff",
        )
        return bunch.frame
