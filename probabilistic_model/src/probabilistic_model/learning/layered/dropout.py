"""
Mini-batch gradient training of layered circuits with the probabilistic dropout of
:cite:`peharz2020random`, section 3.3.
"""

from __future__ import annotations

# %% imports
import math
from dataclasses import dataclass, field

import jax
import jax.numpy as jnp
import numpy as np
import optax
from typing_extensions import Callable

from probabilistic_model.learning.exceptions import UnscheduledOptimizerError
from probabilistic_model.learning.training_schedule import (
    Convergence,
    ConvergenceHistory,
    LearningRatePlateau,
    MiniBatches,
    PlateauScheduledTraining,
)

from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    SampleArray,
)
from probabilistic_model.probabilistic_circuit.tensorized.has_parameter_arrays import (
    HasParameterArrays,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import (
    InnerLayer,
    Layer,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.tensorized.query_cache import (
    QueryCache,
    QueryCacheKey,
)


# %% dropout


@dataclass(kw_only=True)
class ProbabilisticDropout:
    """
    Drops, independently for every row, input distributions by marginalizing them and
    the children of every sum layer below the root by excluding them from the mixture.
    """

    input_probability: float = 0.0
    """
    The probability that an input distribution is marginalized for a row.
    """

    sum_probability: float = 0.0
    """
    The probability that a child of a sum unit is excluded for a row.
    """

    def log_likelihood(
        self, *, root: Layer, data: jax.Array, key: jax.Array
    ) -> jax.Array:
        """
        :param root: The root layer of the circuit with jax parameters.
        :param data: The encoded rows.
        :param key: The random key of the masks.
        :return: The log-likelihood of every row under the dropped-out circuit.
        """
        cache = QueryCache()
        layers = root.all_layers()
        keys = iter(jax.random.split(key, len(layers)))
        for layer in reversed(layers):
            layer_key = next(keys)
            if not isinstance(layer, InnerLayer):
                log_likelihoods = layer.log_likelihood_of_nodes(data, cache=cache)
                kept = jax.random.bernoulli(
                    layer_key, 1 - self.input_probability, log_likelihoods.shape
                )
                cache.results[self.cache_key(layer=layer)] = jnp.where(
                    kept, log_likelihoods, 0.0
                )
            elif isinstance(layer, SumLayer) and layer is not root:
                child_results = []
                for child_layer, child_key in zip(
                    layer.child_layers,
                    jax.random.split(layer_key, len(layer.child_layers)),
                ):
                    child_result = child_layer.log_likelihood_of_nodes(
                        data, cache=cache
                    )
                    kept = jax.random.bernoulli(
                        child_key, 1 - self.sum_probability, child_result.shape
                    )
                    child_results.append(jnp.where(kept, child_result, -jnp.inf))
                cache.results[self.cache_key(layer=layer)] = layer.log_weighted_sum(
                    child_results
                )
            else:
                layer.log_likelihood_of_nodes(data, cache=cache)
        return root.log_likelihood_of_nodes(data, cache=cache)[:, 0]

    @staticmethod
    def cache_key(*, layer: Layer) -> QueryCacheKey:
        """
        :param layer: A layer.
        :return: The key of the layer's log-likelihood in a :class:`QueryCache`.
        """
        return QueryCacheKey(type(layer).log_likelihood_of_nodes.__wrapped__, id(layer))


# %% mini-batch gradient descent


@dataclass(kw_only=True)
class MiniBatchGradientDescent:
    """
    Minimizes the average negative log-likelihood of a circuit with jax parameters by
    mini-batch gradient descent, under probabilistic dropout.
    """

    epochs: int = 10
    """
    The number of passes over the data when :attr:`convergence` is ``None``.
    """

    convergence: Convergence | None = None
    """
    When training stops, after exactly :attr:`epochs` if ``None``.
    """

    batch_size: int = 512
    """
    The number of rows per step.
    """

    optimizer: optax.GradientTransformation = field(
        default_factory=lambda: optax.adam(1e-2)
    )
    """
    The optimizer.
    """

    dropout: ProbabilisticDropout = field(default_factory=ProbabilisticDropout)
    """
    The dropout during training, none by default.
    """

    learning_rate_plateau: LearningRatePlateau | None = None
    """
    The schedule of the learning rate, which needs an optimizer built with
    :func:`optax.inject_hyperparams`; a constant learning rate if ``None``.
    """

    seed: int = 0
    """
    The seed of the order of the rows and of the dropout masks.
    """

    average_log_likelihoods: list[float] = field(default_factory=list, init=False)
    """
    The average log-likelihood of the training rows under dropout in every epoch.
    """

    history: ConvergenceHistory | None = field(default=None, init=False)
    """
    The losses of the epochs of the last :meth:`fit`.
    """

    def fit(
        self, *, circuit: LayeredProbabilisticCircuit, data: SampleArray
    ) -> LayeredProbabilisticCircuit:
        """
        Learn the parameters of the circuit, in place.

        :param circuit: A circuit whose parameters are jax arrays.
        :param data: The rows, whose symbolic values are hashes of domain elements.
        :return: The circuit.
        """
        encoded = np.asarray(HasParameterArrays.host_copy(circuit.encoded(data)))
        root = circuit.root
        impossible = [jnp.isneginf(parameter) for parameter in root.circuit_parameters]

        def loss(parameters: list[jax.Array], batch: jax.Array, key: jax.Array):
            log_likelihoods = self.dropout.log_likelihood(
                root=root.copy_with_circuit_parameters(parameters), data=batch, key=key
            )
            return -log_likelihoods.mean()

        def step(parameters, optimizer_state, batch, key):
            value, gradients = jax.value_and_grad(loss)(parameters, batch, key)
            updates, optimizer_state = self.optimizer.update(
                gradients, optimizer_state, parameters
            )
            parameters = root.projected_circuit_parameters(
                optax.apply_updates(parameters, updates)
            )
            parameters = [
                jnp.where(mask, -jnp.inf, parameter)
                for mask, parameter in zip(impossible, parameters)
            ]
            return parameters, optimizer_state, value

        training = DropoutTraining(
            arguments=(encoded,),
            mini_batches=MiniBatches(batch_size=self.batch_size, seed=self.seed),
            compiled_step=jax.jit(step),
            parameters=list(root.circuit_parameters),
            optimizer_state=self.optimizer.init(list(root.circuit_parameters)),
            key=jax.random.PRNGKey(self.seed),
            floating_point_type=root.floating_point_type,
            device=root.device,
            learning_rate_plateau=self.learning_rate_plateau,
        )
        convergence = (
            Convergence(tolerance=-math.inf, maximum_epochs=self.epochs)
            if self.convergence is None
            else self.convergence
        )
        self.history = convergence.history_until_converged(training=training)
        self.average_log_likelihoods.extend(
            -loss for loss in self.history.epoch_losses
        )
        circuit.root = root.copy_with_circuit_parameters(training.parameters)
        return circuit


# %% training on mini-batches


@dataclass(kw_only=True)
class DropoutTraining(PlateauScheduledTraining):
    """
    Mini-batch gradient descent under dropout, one compiled step per batch.
    """

    compiled_step: Callable
    """
    The step from parameters, optimizer state, batch and key to the new parameters,
    optimizer state and loss.
    """

    parameters: list[jax.Array]
    """
    The parameters after the last step.
    """

    optimizer_state: optax.OptState
    """
    The optimizer state after the last step.
    """

    key: jax.Array
    """
    The key the dropout masks of the next step are drawn from.
    """

    floating_point_type: jnp.dtype
    """
    The floating point type of the parameters.
    """

    device: jax.Device
    """
    The device of the parameters.
    """

    initial_learning_rate: jax.Array | None = field(init=False, default=None)
    """
    The learning rate before any reduction, ``None`` without a schedule.
    """

    def __post_init__(self) -> None:
        """
        Read the initial learning rate of a schedule.

        :raises UnscheduledOptimizerError: If a schedule is given for an optimizer
            whose learning rate cannot be changed.
        """
        if self.learning_rate_plateau is None:
            return
        if not isinstance(self.optimizer_state, optax.InjectHyperparamsState):
            raise UnscheduledOptimizerError()
        self.initial_learning_rate = self.optimizer_state.hyperparams["learning_rate"]

    def scale_learning_rate(self, *, scale: float) -> None:
        self.optimizer_state.hyperparams["learning_rate"] = (
            self.initial_learning_rate * scale
        )

    def step_on_batch(self, *, batch: tuple[np.ndarray, ...]) -> float:
        (rows,) = batch
        self.key, step_key = jax.random.split(self.key)
        self.parameters, self.optimizer_state, value = self.compiled_step(
            self.parameters,
            self.optimizer_state,
            jnp.asarray(rows, dtype=self.floating_point_type, device=self.device),
            step_key,
        )
        return float(value)
