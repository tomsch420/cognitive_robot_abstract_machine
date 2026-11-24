"""
Changes of the sufficient statistics that tensorized expectation maximization counts,
made on the host before they are maximized: the momentum and the learning rate per node
of anemone :cite:`liu2026rethinking`.
"""

from __future__ import annotations

# %% imports
import dataclasses
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from typing_extensions import Self

from probabilistic_model.learning.layered.flows import TopDownProbabilities
from probabilistic_model.learning.parameters.expectation_maximization.exceptions import (
    UnsupportedLayerError,
)
from probabilistic_model.learning.parameters.expectation_maximization.layer_maximization import (
    EdgeCounts,
    GaussianStatistics,
    StateCounts,
    SufficientStatistics,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import Layer

# %% statistics of the learned layers


@dataclass(kw_only=True)
class LearnedLayerStatistics:
    """
    The sufficient statistics of every learned layer of a circuit since the last update,
    with host parameters and arrays.
    """

    root: Layer
    """
    The root layer of the circuit at the parameters the statistics were counted with.
    """

    layers: list[Layer]
    """
    The layers with parameters, in the order of :meth:`Layer.all_layers`.
    """

    statistics: list[SufficientStatistics]
    """
    The statistics of every layer of :attr:`layers`.
    """

    number_of_rows: int
    """
    The number of rows the statistics were counted on.
    """

    def with_statistics(self, *, statistics: list[SufficientStatistics]) -> Self:
        """
        :param statistics: Other statistics of every layer of :attr:`layers`.
        :return: These statistics of the same layers and rows.
        """
        return dataclasses.replace(self, statistics=statistics)


# %% adjustments


@dataclass(kw_only=True)
class StatisticsAdjustment(ABC):
    """
    A change of the sufficient statistics before the maximization step turns them into
    parameters.
    """

    @abstractmethod
    def adjusted(self, *, statistics: LearnedLayerStatistics) -> LearnedLayerStatistics:
        """
        :param statistics: The statistics since the last update.
        :return: The statistics the maximization step uses instead.
        """
        raise NotImplementedError


@dataclass(kw_only=True)
class Momentum(StatisticsAdjustment):
    """
    Replaces the statistics per row by their bias-corrected exponential moving average
    over the updates, the momentum of :cite:`liu2026rethinking`, section 5.1.

    .. note::
        It keeps the average of the updates it saw, so every training needs its own.
    """

    decay: float
    """
    The weight of the previous average, called beta in the paper.
    """

    averages: list[list[np.ndarray]] | None = field(default=None, init=False)
    """
    The average of the uncentered statistics per row of every learned layer, ``None``
    before the first update.
    """

    number_of_updates: int = field(default=0, init=False)
    """
    The number of updates the averages include.
    """

    def adjusted(self, *, statistics: LearnedLayerStatistics) -> LearnedLayerStatistics:
        per_row = [
            [array / statistics.number_of_rows for array in arrays]
            for arrays in self.uncentered_arrays(statistics=statistics)
        ]
        previous = (
            [[np.zeros_like(array) for array in arrays] for arrays in per_row]
            if self.averages is None
            else self.averages
        )
        self.averages = [
            [
                self.decay * average + (1 - self.decay) * array
                for average, array in zip(averages, arrays)
            ]
            for averages, arrays in zip(previous, per_row)
        ]
        self.number_of_updates += 1
        rows_after_correction = statistics.number_of_rows / (
            1 - self.decay**self.number_of_updates
        )
        return statistics.with_statistics(
            statistics=[
                self.centered_statistics(
                    layer=layer,
                    statistics_type=type(layer_statistics),
                    arrays=[average * rows_after_correction for average in averages],
                )
                for layer, layer_statistics, averages in zip(
                    statistics.layers, statistics.statistics, self.averages
                )
            ]
        )

    @staticmethod
    def uncentered_arrays(
        *, statistics: LearnedLayerStatistics
    ) -> list[list[np.ndarray]]:
        """
        :param statistics: The statistics since the last update.
        :return: The arrays of every layer's statistics, the sums of a Gaussian layer
            about zero rather than about its location, which changes with every update.
        """
        result = []
        for layer, layer_statistics in zip(statistics.layers, statistics.statistics):
            if not isinstance(layer_statistics, GaussianStatistics):
                result.append(layer_statistics.arrays)
                continue
            location = layer.location
            sums = layer_statistics.centered_sums + layer_statistics.counts * location
            squared_sums = (
                layer_statistics.centered_squared_sums
                + 2 * location * layer_statistics.centered_sums
                + layer_statistics.counts * location**2
            )
            result.append([layer_statistics.counts, sums, squared_sums])
        return result

    @staticmethod
    def centered_statistics(
        *,
        layer: Layer,
        statistics_type: type[SufficientStatistics],
        arrays: list[np.ndarray],
    ) -> SufficientStatistics:
        """
        :param layer: A learned layer at its current parameters.
        :param statistics_type: The type of the layer's statistics.
        :param arrays: Arrays in the order :meth:`uncentered_arrays` gives them.
        :return: The statistics, those of a Gaussian layer centered at its location.
        """
        if statistics_type is not GaussianStatistics:
            return statistics_type.from_arrays(arrays=arrays)
        counts, sums, squared_sums = arrays
        location = layer.location
        return GaussianStatistics(
            counts=counts,
            centered_sums=sums - counts * location,
            centered_squared_sums=squared_sums
            - 2 * location * sums
            + counts * location**2,
        )


@dataclass(kw_only=True)
class StateCountSmoothing(StatisticsAdjustment):
    """
    Adds :attr:`pseudocount` to the count of every possible state of every discrete
    node, so that a state the rows never show stays possible; a state of probability
    zero, such as the padding of a layer over variables of different domain sizes, stays
    impossible.
    """

    pseudocount: float = 0.1
    """
    The count added to every possible state of every discrete node.
    """

    def adjusted(self, *, statistics: LearnedLayerStatistics) -> LearnedLayerStatistics:
        return statistics.with_statistics(
            statistics=[
                StateCounts(
                    counts=layer_statistics.counts
                    + self.pseudocount * np.isfinite(layer.table.log_probabilities)
                )
                if isinstance(layer_statistics, StateCounts)
                else layer_statistics
                for layer, layer_statistics in zip(
                    statistics.layers, statistics.statistics
                )
            ]
        )


@dataclass(kw_only=True)
class AnemoneMixing(StatisticsAdjustment):
    """
    Mixes the current parameters of every node, weighted by its top-down probability,
    with the statistics of the batch, weighted by :attr:`learning_rate` per row, the
    update of anemone :cite:`liu2026rethinking`, equation 7, for a maximization step of
    step size one.
    """

    learning_rate: float = 0.1
    """
    The learning rate, called eta in the paper.
    """

    def adjusted(self, *, statistics: LearnedLayerStatistics) -> LearnedLayerStatistics:
        top_down_probabilities = (
            TopDownProbabilities(root=statistics.root).compute().node_values
        )
        batch_weight = self.learning_rate / statistics.number_of_rows
        return statistics.with_statistics(
            statistics=[
                self.mixed_statistics(
                    layer=layer,
                    statistics=layer_statistics,
                    top_down_probabilities=top_down_probabilities[layer],
                    batch_weight=batch_weight,
                )
                for layer, layer_statistics in zip(
                    statistics.layers, statistics.statistics
                )
            ]
        )

    @staticmethod
    def mixed_statistics(
        *,
        layer: Layer,
        statistics: SufficientStatistics,
        top_down_probabilities: np.ndarray,
        batch_weight: float,
    ) -> SufficientStatistics:
        """
        :param layer: A learned layer at its current parameters.
        :param statistics: The statistics of the layer on the batch.
        :param top_down_probabilities: The top-down probability of every node.
        :param batch_weight: The weight of the statistics of the batch.
        :return: The statistics whose maximization is the mixture of the current and
            the batch parameters.
        :raises UnsupportedLayerError: If the statistics are of no known kind.
        """
        if isinstance(statistics, EdgeCounts):
            return EdgeCounts(
                counts=layer.edges.values_of_rows(values=top_down_probabilities)
                * layer.normalized_weights
                + batch_weight * statistics.counts
            )
        if isinstance(statistics, StateCounts):
            return StateCounts(
                counts=top_down_probabilities[:, np.newaxis]
                * layer.table.normalized_probabilities
                + batch_weight * statistics.counts
            )
        if isinstance(statistics, GaussianStatistics):
            return GaussianStatistics(
                counts=top_down_probabilities + batch_weight * statistics.counts,
                centered_sums=batch_weight * statistics.centered_sums,
                centered_squared_sums=top_down_probabilities * layer.scale**2
                + batch_weight * statistics.centered_squared_sums,
            )
        raise UnsupportedLayerError(layer_type=type(layer))


@dataclass(kw_only=True)
class StatisticsRecorder(StatisticsAdjustment):
    """
    Keeps the statistics of the last update and leaves them unchanged.
    """

    recorded: LearnedLayerStatistics | None = field(default=None, init=False)
    """
    The statistics of the last update, ``None`` before the first.
    """

    def adjusted(self, *, statistics: LearnedLayerStatistics) -> LearnedLayerStatistics:
        self.recorded = statistics
        return statistics
