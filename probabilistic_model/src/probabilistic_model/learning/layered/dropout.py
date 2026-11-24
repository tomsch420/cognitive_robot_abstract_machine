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
import tqdm

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
    mini-batch gradient descent, under probabilistic dropout.
    """

    epochs: int = 10
    """
    The number of passes over the data.
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

    average_log_likelihoods: list[float] = field(default_factory=list, init=False)
    """
    The average log-likelihood of the training rows under dropout in every epoch.
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

        def loss(
            parameters: list[jax.Array], batch: jax.Array, key: jax.Array
        ) -> jax.Array:
            """
            :param parameters: Arrays in the order of the circuit's parameters.
            :param batch: The encoded rows of the batch.
            :param key: The random key of the dropout.
            :return: The negative average log-likelihood of the rows under dropout.
            """
            log_likelihoods = self.dropout.log_likelihood(
                root=root.copy_with_circuit_parameters(parameters=parameters),
                data=batch,
                key=key,
            )
            return -log_likelihoods.mean()

        def step(
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
            value, gradients = jax.value_and_grad(loss)(parameters, batch, key)
            updates, optimizer_state = self.optimizer.update(
                gradients, optimizer_state, parameters
            )
            parameters = root.projected_circuit_parameters(
                parameters=optax.apply_updates(parameters, updates)
            )
            parameters = [
                jnp.where(mask, -jnp.inf, parameter)
                for mask, parameter in zip(impossible, parameters)
            ]
            return parameters, optimizer_state, value

        compiled_step = jax.jit(step)
        parameters = list(root.circuit_parameters)
        optimizer_state = self.optimizer.init(parameters)
        random_number_generator = np.random.default_rng(self.seed)
        key = jax.random.PRNGKey(self.seed)
        for _ in tqdm.tqdm(range(self.epochs), desc="Gradient descent"):
            order = random_number_generator.permutation(len(encoded))
            summed = 0.0
            for start in range(0, len(encoded), self.batch_size):
                batch = encoded[order[start : start + self.batch_size]]
                key, step_key = jax.random.split(key)
                parameters, optimizer_state, value = compiled_step(
                    parameters,
                    optimizer_state,
                    jnp.asarray(batch, dtype=root.floating_point_type),
                    step_key,
                )
                summed -= float(value) * len(batch)
            self.average_log_likelihoods.append(summed / len(encoded))
        circuit.root = root.copy_with_circuit_parameters(parameters=parameters)
        return circuit
