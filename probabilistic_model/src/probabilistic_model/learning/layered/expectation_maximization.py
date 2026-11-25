"""
Expectation maximization of the parameters of layered circuits by
:class:`TensorizedExpectationMaximization`, with data softening and adjustments of the
sufficient statistics such as anemone :cite:`liu2026rethinking`.
"""

from __future__ import annotations

# %% imports
import copy
from abc import ABC
from dataclasses import dataclass, field

import array_api_compat
import jax
import jax.numpy as jnp
import numpy as np
import torch
from array_api_compat import array_namespace, is_jax_array, is_torch_array
from typing_extensions import Self

from probabilistic_model.learning.layered.exceptions import (
    UndifferentiableBackendError,
)
from probabilistic_model.learning.layered.regularization import DataSoftening
from probabilistic_model.learning.layered.statistics_adjustment import (
    AnemoneMixing,
    LearnedLayerStatistics,
    Momentum,
    StateCountSmoothing,
    StatisticsAdjustment,
)
from probabilistic_model.learning.parameters.circuit_parameter_learning import (
    LayerType,
    TrainingRun,
)
from probabilistic_model.learning.parameters.expectation_maximization.expectation_maximization import (
    MaximizationRegularization,
)
from probabilistic_model.learning.parameters.expectation_maximization.jax_expectation_maximization import (
    ExpectationMaximizationState,
    JaxExpectationMaximization,
    JaxExpectationMaximizationTraining,
)
from probabilistic_model.learning.parameters.expectation_maximization.layer_maximization import (
    SufficientStatistics,
)
from probabilistic_model.learning.parameters.expectation_maximization.tensorized_expectation_maximization import (
    TensorizedExpectationMaximization,
)
from probabilistic_model.learning.parameters.expectation_maximization.torch_expectation_maximization import (
    TorchExpectationMaximization,
    TorchExpectationMaximizationTraining,
)
from probabilistic_model.learning.parameters.training_schedule import (
    Convergence,
    ConvergenceHistory,
    MiniBatches,
)
from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    BackendArray,
    SampleArray,
    SampleRows,
)
from probabilistic_model.probabilistic_circuit.tensorized.has_parameter_arrays import (
    HasParameterArrays,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import Layer
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)

# %% adjusted expectation maximization


@dataclass(kw_only=True)
class AdjustedExpectationMaximization(
    TensorizedExpectationMaximization[LayerType], ABC
):
    """
    :class:`TensorizedExpectationMaximization` that counts the statistics on softened
    data and adjusts them on the host before every update.
    """

    statistics_adjustments: list[StatisticsAdjustment] = field(default_factory=list)
    """
    The adjustments applied to the statistics before every update, in order.
    """

    data_softening: DataSoftening | None = None
    """
    The softening of the data the statistics are counted on, none if ``None``.
    """

    def total_log_likelihood(
        self,
        *,
        parameters: list[BackendArray],
        node_offsets: list[BackendArray],
        data: SampleArray,
    ) -> BackendArray:
        """
        :meth:`TensorizedExpectationMaximization.total_log_likelihood` of the softened
        data if :attr:`data_softening` is set.
        """
        evaluated_parameters = (
            parameters
            if self.data_softening is None
            else self.data_softening.transformed(root=self.layer, parameters=parameters)
        )
        return super().total_log_likelihood(
            parameters=evaluated_parameters, node_offsets=node_offsets, data=data
        )

    def adjusted_maximized_parameters(
        self,
        *,
        parameters: list[BackendArray],
        statistics: list[SufficientStatistics],
        number_of_rows: int,
    ) -> list[BackendArray]:
        """
        :param parameters: Arrays in the order of the circuit's parameters, those the
            statistics were counted with.
        :param statistics: The sufficient statistics of every learned layer since the
            last update.
        :param number_of_rows: The number of rows the statistics were counted on.
        :return: The parameters that maximize the adjusted statistics, on the backend
            and the device of ``parameters``.
        """
        host_parameters = [HasParameterArrays.host_copy(array) for array in parameters]
        host_root = self.layer.copy_with_circuit_parameters(parameters=host_parameters)
        adjusted = LearnedLayerStatistics(
            root=host_root,
            layers=self.learned_layers(root=host_root),
            statistics=[
                type(layer_statistics).from_arrays(
                    arrays=[
                        HasParameterArrays.host_copy(array)
                        for array in layer_statistics.arrays
                    ]
                )
                for layer_statistics in statistics
            ],
            number_of_rows=number_of_rows,
        )
        for adjustment in self.statistics_adjustments:
            adjusted = adjustment.adjusted(statistics=adjusted)
        namespace = array_namespace(*parameters)
        return [
            namespace.asarray(
                maximized, dtype=array.dtype, device=array_api_compat.device(array)
            )
            for maximized, array in zip(
                self.maximized_circuit_parameters(
                    parameters=host_parameters, statistics=adjusted.statistics
                ),
                parameters,
            )
        ]


# %% jax


@dataclass(kw_only=True)
class JaxAdjustedExpectationMaximization(
    AdjustedExpectationMaximization[LayerType], JaxExpectationMaximization[LayerType]
):
    """
    :class:`AdjustedExpectationMaximization` with jax: compiled steps and an update on
    the host.
    """

    def start_training(
        self,
        *,
        arguments: tuple[jax.Array, ...],
        mini_batches: MiniBatches,
    ) -> JaxAdjustedExpectationMaximizationTraining[LayerType]:
        """
        :meth:`JaxExpectationMaximization.start_training` with updates on the host.
        """
        return JaxAdjustedExpectationMaximizationTraining(
            arguments=arguments,
            mini_batches=mini_batches,
            batches_per_update=self.batches_per_update,
            learning=self,
        )


@dataclass(kw_only=True)
class JaxAdjustedExpectationMaximizationTraining(
    JaxExpectationMaximizationTraining[LayerType]
):
    """
    Mini-batch training by :class:`JaxAdjustedExpectationMaximization`, whose steps are
    compiled and whose updates run on the host.
    """

    learning: JaxAdjustedExpectationMaximization[LayerType]
    """
    The expectation maximization whose steps are taken.
    """

    rows_since_update: int = field(default=0, init=False)
    """
    The number of rows whose statistics were accumulated since the last update.
    """

    def accumulate_statistics(self, *, batch: tuple[jax.Array, ...]) -> jax.Array:
        """
        :meth:`JaxExpectationMaximizationTraining.accumulate_statistics`, counting the
        rows of the batch.
        """
        self.rows_since_update += batch[0].shape[0]
        return super().accumulate_statistics(batch=batch)

    def accumulate_statistics_and_update(
        self, *, batch: tuple[jax.Array, ...]
    ) -> jax.Array:
        """
        :meth:`JaxExpectationMaximizationTraining.accumulate_statistics_and_update`,
        updating on the host.
        """
        loss = self.accumulate_statistics(batch=batch)
        self.update_parameters()
        return loss

    def step_on_rows(self, *, rows: SampleRows) -> jax.Array:
        """
        :meth:`JaxExpectationMaximizationTraining.step_on_rows`, updating on the host.
        """
        self.rows_since_update += len(rows)
        loss = self.take_compiled_step_on_rows(step=self.compiled_step, rows=rows)
        if self.count_batch_toward_update():
            self.update_parameters()
        return loss

    def update_parameters(self) -> None:
        """
        :meth:`JaxExpectationMaximizationTraining.update_parameters` from the adjusted
        statistics, on the host.
        """
        state = ExpectationMaximizationState(
            parameters=self.learning.adjusted_maximized_parameters(
                parameters=self.state.parameters,
                statistics=self.state.statistics,
                number_of_rows=self.rows_since_update,
            ),
            statistics=jax.tree.map(jnp.zeros_like, self.state.statistics),
            loss=self.state.loss,
        )
        self.state = (
            state
            if self.batch_split_devices is None
            else self.batch_split_devices.replicated(state=state)
        )
        self.rows_since_update = 0


# %% torch


@dataclass(kw_only=True)
class TorchAdjustedExpectationMaximization(
    AdjustedExpectationMaximization[LayerType], TorchExpectationMaximization[LayerType]
):
    """
    :class:`AdjustedExpectationMaximization` with torch, updating on the host.
    """

    def start_training(
        self,
        *,
        arguments: tuple[torch.Tensor, ...],
        mini_batches: MiniBatches,
    ) -> TorchAdjustedExpectationMaximizationTraining[LayerType]:
        """
        :meth:`TorchExpectationMaximization.start_training` with updates on the host.
        """
        return TorchAdjustedExpectationMaximizationTraining(
            arguments=arguments,
            mini_batches=mini_batches,
            batches_per_update=self.batches_per_update,
            learning=self,
        )


@dataclass(kw_only=True)
class TorchAdjustedExpectationMaximizationTraining(
    TorchExpectationMaximizationTraining[LayerType]
):
    """
    Mini-batch training by :class:`TorchAdjustedExpectationMaximization`, whose updates
    run on the host.
    """

    learning: TorchAdjustedExpectationMaximization[LayerType]
    """
    The expectation maximization whose parameters are updated.
    """

    rows_since_update: int = field(default=0, init=False)
    """
    The number of rows whose statistics were accumulated since the last update.
    """

    def accumulate_statistics(self, *, batch: tuple[torch.Tensor, ...]) -> torch.Tensor:
        """
        :meth:`TorchExpectationMaximizationTraining.accumulate_statistics`, counting the
        rows of the batch.
        """
        self.rows_since_update += batch[0].shape[0]
        return super().accumulate_statistics(batch=batch)

    def accumulate_statistics_and_update(
        self, *, batch: tuple[torch.Tensor, ...]
    ) -> torch.Tensor:
        """
        :meth:`TorchExpectationMaximizationTraining.accumulate_statistics_and_update`,
        updating on the host.
        """
        loss = self.accumulate_statistics(batch=batch)
        self.update_parameters()
        return loss

    def update_parameters(self) -> None:
        """
        :meth:`TorchExpectationMaximizationTraining.update_parameters` from the adjusted
        statistics, on the host.
        """
        parameters = self.learning.parameters
        with torch.no_grad():
            maximized = self.learning.adjusted_maximized_parameters(
                parameters=[parameter.detach() for parameter in parameters],
                statistics=self.statistics,
                number_of_rows=self.rows_since_update,
            )
            for parameter, updated in zip(parameters, maximized):
                parameter.copy_(updated)
            for statistics in self.statistics:
                for array in statistics.arrays:
                    array.zero_()
        self.rows_since_update = 0


# %% learning the parameters of a circuit


@dataclass(kw_only=True)
class CircuitExpectationMaximization:
    """
    Learns the parameters of a layered circuit with jax or torch parameters in place, by
    tensorized expectation maximization until :attr:`convergence` stops it.
    """

    convergence: Convergence = field(
        default_factory=lambda: Convergence(maximum_epochs=10, patience=10)
    )
    """
    When training stops, after ten epochs by default.
    """

    batch_size: int | None = None
    """
    The number of rows per batch, all rows if ``None``.
    """

    batches_per_update: int | None = None
    """
    The batches after which the parameters are updated, once per epoch if ``None``.
    """

    step_size: float = 1.0
    """
    The share of the way to the maximizing parameters that an update moves.
    """

    statistics_adjustments: list[StatisticsAdjustment] = field(
        default_factory=lambda: [StateCountSmoothing()]
    )
    """
    The adjustments of the statistics before every update, in order; every fit starts
    from copies of them.
    """

    data_softening: DataSoftening | None = None
    """
    The softening of the data the statistics are counted on, none if ``None``.
    """

    regularization: MaximizationRegularization = field(
        default_factory=MaximizationRegularization
    )
    """
    How the maximization step keeps the learned weights and variances away from zero.
    """

    seed: int = 0
    """
    The seed of the order in which the rows are visited.
    """

    @staticmethod
    def convergence_after(*, epochs: int) -> Convergence:
        """
        :param epochs: The number of passes over the data.
        :return: The convergence that stops training after exactly that many epochs.
        """
        return Convergence(maximum_epochs=epochs, patience=epochs)

    @classmethod
    def full_batch(cls, *, epochs: int) -> Self:
        """
        :param epochs: The number of passes over the data.
        :return: Expectation maximization with one update of all rows per epoch, for
            that many epochs.
        """
        return cls(convergence=cls.convergence_after(epochs=epochs))

    @classmethod
    def mini_batch(cls, *, epochs: int) -> Self:
        """
        :param epochs: The number of passes over the data.
        :return: The mini-batch expectation maximization of :cite:`peharz2020einsum`,
            equation 6 of :cite:`liu2026rethinking`: a step of a tenth of the way after
            every batch of 512 rows.
        """
        return cls(
            convergence=cls.convergence_after(epochs=epochs),
            batch_size=512,
            batches_per_update=1,
            step_size=0.1,
        )

    @classmethod
    def anemone(cls, *, epochs: int) -> Self:
        """
        :param epochs: The number of passes over the data.
        :return: Anemone :cite:`liu2026rethinking` with the momentum and the learning
            rate of the paper, after every batch of 512 rows.
        """
        return cls(
            convergence=cls.convergence_after(epochs=epochs),
            batch_size=512,
            batches_per_update=1,
            statistics_adjustments=[
                Momentum(decay=0.9),
                StateCountSmoothing(),
                AnemoneMixing(),
            ],
        )

    def learner(self, *, root: Layer) -> TensorizedExpectationMaximization:
        """
        :param root: The root layer of the circuit.
        :return: The expectation maximization of the circuit with the array library of
            its parameters.
        :raises UndifferentiableBackendError: If that library does not differentiate.
        """
        settings = dict(
            layer=root,
            batches_per_update=self.batches_per_update,
            step_size=self.step_size,
            regularization=self.regularization,
        )
        adjusted_settings = dict(
            settings,
            statistics_adjustments=copy.deepcopy(self.statistics_adjustments),
            data_softening=self.data_softening,
        )
        is_adjusted = (
            bool(self.statistics_adjustments) or self.data_softening is not None
        )
        parameter = root.circuit_parameters[0]
        if is_jax_array(parameter):
            if is_adjusted:
                return JaxAdjustedExpectationMaximization(**adjusted_settings)
            return JaxExpectationMaximization(**settings)
        if is_torch_array(parameter):
            if is_adjusted:
                return TorchAdjustedExpectationMaximization(**adjusted_settings)
            return TorchExpectationMaximization(**settings)
        raise UndifferentiableBackendError(parameter_type=type(parameter))

    def fit(
        self, *, circuit: LayeredProbabilisticCircuit, data: SampleArray
    ) -> ConvergenceHistory:
        """
        Learn the parameters of the circuit, in place.

        :param circuit: A circuit whose parameters are jax or torch arrays.
        :param data: The rows, whose symbolic values are hashes of domain elements.
        :return: The losses of the epochs trained.
        """
        if self.convergence.maximum_epochs == 0:
            return ConvergenceHistory(epoch_losses=[], converged=False)
        run = self.training_run(
            learning=self.learner(root=circuit.root), circuit=circuit, data=data
        )
        circuit.root = run.trained_root
        return run.history

    def training_run(
        self,
        *,
        learning: TensorizedExpectationMaximization,
        circuit: LayeredProbabilisticCircuit,
        data: SampleArray,
    ) -> TrainingRun:
        """
        :param learning: The expectation maximization of the circuit's root.
        :param circuit: The circuit, which is left unchanged.
        :param data: The rows, whose symbolic values are hashes of domain elements.
        :return: The circuit's root trained until :attr:`convergence` stops, and the
            losses of its epochs.
        """
        root = circuit.root
        encoded = root.namespace.asarray(
            np.asarray(HasParameterArrays.host_copy(circuit.encoded(data))),
            dtype=root.floating_point_type,
            device=root.device,
        )
        return learning.minimize_until_converged(
            encoded,
            convergence=self.convergence,
            mini_batches=MiniBatches(batch_size=self.batch_size, seed=self.seed),
        )
