"""
Mini-batch gradient training of layered circuits with the probabilistic dropout of
:cite:`peharz2020random`, section 3.3.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass, field

import jax
import jax.numpy as jnp
import numpy as np
import optax
from typing_extensions import Callable

from probabilistic_model.learning.parameters.training_schedule import (
    Convergence,
    ConvergenceHistory,
    MiniBatches,
    MiniBatchTraining,
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
                log_likelihoods = layer.log_likelihood_of_nodes(
                    events=data, cache=cache
                )
                kept = jax.random.bernoulli(
                    layer_key, 1 - self.input_probability, log_likelihoods.shape
                )
                cache.results[
                    QueryCacheKey.from_query(
                        query=type(layer).log_likelihood_of_nodes, layer=layer
                    )
                ] = jnp.where(kept, log_likelihoods, 0.0)
            elif isinstance(layer, SumLayer) and layer is not root:
                child_results = []
                for child_layer, child_key in zip(
                    layer.child_layers,
                    jax.random.split(layer_key, len(layer.child_layers)),
                ):
                    child_result = child_layer.log_likelihood_of_nodes(
                        events=data, cache=cache
                    )
                    kept = jax.random.bernoulli(
                        child_key, 1 - self.sum_probability, child_result.shape
                    )
                    child_results.append(jnp.where(kept, child_result, -jnp.inf))
                cache.results[
                    QueryCacheKey.from_query(
                        query=type(layer).log_likelihood_of_nodes, layer=layer
                    )
                ] = layer.log_weighted_sum(child_results=child_results)
            else:
                layer.log_likelihood_of_nodes(events=data, cache=cache)
        return root.log_likelihood_of_nodes(events=data, cache=cache)[:, 0]


# %% mini-batch gradient descent


@dataclass(kw_only=True)
class MiniBatchGradientDescent:
    """
    Minimizes the average negative log-likelihood of a circuit with jax parameters by
    mini-batch gradient descent under probabilistic dropout, until :attr:`convergence`
    stops it.
    """

    convergence: Convergence = field(default_factory=Convergence)
    """
    When training stops.
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

    seed: int = 0
    """
    The seed of the order of the rows and of the dropout masks.
    """

    def fit(
        self, *, circuit: LayeredProbabilisticCircuit, data: SampleArray
    ) -> ConvergenceHistory:
        """
        Learn the parameters of the circuit, in place.

        :param circuit: A circuit whose parameters are jax arrays.
        :param data: The rows, whose symbolic values are hashes of domain elements.
        :return: The average loss under dropout of every epoch trained.
        """
        root = circuit.root
        encoded = root.namespace.asarray(
            np.asarray(HasParameterArrays.host_copy(circuit.encoded(data))),
            dtype=root.floating_point_type,
            device=root.device,
        )
        training = DropoutGradientDescentTraining(
            arguments=(encoded,),
            mini_batches=MiniBatches(batch_size=self.batch_size, seed=self.seed),
            learning=self,
            root=root,
        )
        history = self.convergence.history_until_converged(training=training)
        circuit.root = root.copy_with_circuit_parameters(parameters=training.parameters)
        return history


@dataclass(kw_only=True)
class DropoutGradientDescentTraining(MiniBatchTraining):
    """
    The steps of a :class:`MiniBatchGradientDescent`, one compiled step per batch with a
    fresh dropout key.
    """

    learning: MiniBatchGradientDescent
    """
    The gradient descent whose steps are taken.
    """

    root: Layer
    """
    The root of the circuit at the parameters before training.
    """

    parameters: list[jax.Array] = field(init=False)
    """
    The parameters after the last step.
    """

    optimizer_state: optax.OptState = field(init=False)
    """
    The state of the optimizer after the last step.
    """

    key: jax.Array = field(init=False)
    """
    The random key the dropout key of the next step is split from.
    """

    impossible: list[jax.Array] = field(init=False)
    """
    Where every parameter array is ``-inf``, a probability of zero kept by every step.
    """

    compiled_step: Callable[..., tuple[list[jax.Array], optax.OptState, jax.Array]] = (
        field(init=False)
    )
    """
    :meth:`step`, compiled.
    """

    def __post_init__(self) -> None:
        """
        Start from the parameters of :attr:`root` and compile the step.
        """
        self.parameters = list(self.root.circuit_parameters)
        self.optimizer_state = self.learning.optimizer.init(self.parameters)
        self.key = jax.random.PRNGKey(self.learning.seed)
        self.impossible = [jnp.isneginf(parameter) for parameter in self.parameters]
        self.compiled_step = jax.jit(
            lambda parameters, optimizer_state, batch, key: self.step(
                parameters=parameters,
                optimizer_state=optimizer_state,
                batch=batch,
                key=key,
            )
        )

    def loss(
        self, *, parameters: list[jax.Array], batch: jax.Array, key: jax.Array
    ) -> jax.Array:
        """
        :param parameters: Arrays in the order of the circuit's parameters.
        :param batch: The encoded rows of the batch.
        :param key: The random key of the dropout.
        :return: The negative average log-likelihood of the rows under dropout.
        """
        log_likelihoods = self.learning.dropout.log_likelihood(
            root=self.root.copy_with_circuit_parameters(parameters=parameters),
            data=batch,
            key=key,
        )
        return -log_likelihoods.mean()

    def step(
        self,
        *,
        parameters: list[jax.Array],
        optimizer_state: optax.OptState,
        batch: jax.Array,
        key: jax.Array,
    ) -> tuple[list[jax.Array], optax.OptState, jax.Array]:
        """
        :param parameters: Arrays in the order of the circuit's parameters.
        :param optimizer_state: The state of the optimizer before the step.
        :param batch: The encoded rows of the batch.
        :param key: The random key of the dropout.
        :return: The projected parameters and the optimizer state after the step,
            and the loss before it; parameters of probability zero stay zero.
        """
        value, gradients = jax.value_and_grad(
            lambda differentiated: self.loss(
                parameters=differentiated, batch=batch, key=key
            )
        )(parameters)
        updates, updated_optimizer_state = self.learning.optimizer.update(
            gradients, optimizer_state, parameters
        )
        projected = self.root.projected_circuit_parameters(
            parameters=optax.apply_updates(parameters, updates)
        )
        return (
            [
                jnp.where(mask, -jnp.inf, parameter)
                for mask, parameter in zip(self.impossible, projected)
            ],
            updated_optimizer_state,
            value,
        )

    def step_on_batch(self, *, batch: tuple[jax.Array, ...]) -> jax.Array:
        [data] = batch
        self.key, step_key = jax.random.split(self.key)
        self.parameters, self.optimizer_state, value = self.compiled_step(
            self.parameters, self.optimizer_state, data, step_key
        )
        return value
