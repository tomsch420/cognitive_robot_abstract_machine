from __future__ import annotations

from abc import ABC
from dataclasses import dataclass
import numpy as np
import scipy.stats
import math
from typing_extensions import List, Tuple, Optional, Any, Self, Dict, Type, TypeVar, Union
from krrood.adapters.json_serializer import (
    SubclassJSONSerializer,
    DataclassJSONSerializer,
    to_json,
    from_json,
)
from probabilistic_model.probabilistic_circuit.numpy.layer import (
    Layer,
    InputLayer,
)
from probabilistic_model.probabilistic_circuit.numpy.conversion import (
    RustworkxLayerConverter,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    Unit,
    UnivariateDiscreteLeaf,
    UnivariateContinuousLeaf,
    ProbabilisticCircuit as RustworkxProbabilisticCircuit,
)
from probabilistic_model.distributions.distributions import (
    SymbolicDistribution,
    DiscreteDistribution,
    DiracDeltaDistribution,
)
from probabilistic_model.distributions.gaussian import (
    GaussianDistribution,
    TruncatedGaussianDistribution,
)
from probabilistic_model.distributions.uniform import UniformDistribution
from random_events.variable import Variable, Symbolic, Continuous
from random_events.interval import reals
from sortedcontainers import SortedSet
from probabilistic_model.probabilistic_model import OrderType, CenterType
from random_events.product_algebra import Event, SimpleEvent
import tqdm


def double_factorial(n: int) -> int:
    """
    Calculate the double factorial of a non-negative integer n.

    The double factorial is defined as the product of all integers from 1 up to n
    that have the same parity as n.

    :param n: The integer to calculate the double factorial of.
    :return: The double factorial of n.
    """
    if n <= 0:
        return 1
    return math.prod(range(n, 0, -2))


T = TypeVar("T")


@dataclass
class ContinuousLayer(InputLayer[T], ABC):
    """
    Abstract base class for continuous univariate input units.
    """


@dataclass
class ContinuousLayerWithFiniteSupport(ContinuousLayer[T], ABC):
    """
    Abstract class for continuous univariate input units with finite support.
    """

    interval: np.ndarray
    """
    The interval of the distribution as an array of shape (number_of_nodes, 2).
    The first column contains the lower bounds, the second column contains the upper
    bounds. All intervals are assumed to be closed.
    """

    @property
    def lower(self) -> np.ndarray:
        return self.interval[:, 0]

    @property
    def upper(self) -> np.ndarray:
        return self.interval[:, 1]


@dataclass
class DiscreteLayer(InputLayer[Union[SymbolicDistribution, DiscreteDistribution]]):
    """
    A layer that represents discrete distributions over a single variable.
    """

    probabilities: np.ndarray
    """
    The probability for each state of the variable.

    Shape: (number_of_nodes, number_of_states)
    """

    symbol_hash_to_index: Optional[Dict[int, int]] = None
    """
    Mapping from hash of the symbol to index in the probabilities matrix.

    This is mandatory for symbolic variables and optional for integer variables
    where the values can be used directly as indices.
    """

    @property
    def number_of_nodes(self) -> int:
        return self.probabilities.shape[0]

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable]
        if self.symbol_hash_to_index is not None:
            indices = np.array([self.symbol_hash_to_index[hash(value)] for value in values])
        else:
            indices = values.astype(int)
        # self.probabilities has shape (nodes, states)
        # result[n, j] = log(self.probabilities[j, indices[n]])
        # result shape (N, nodes)
        probabilities = self.probabilities[:, indices].T  # (number_of_samples, number_of_nodes)
        return np.log(probabilities)

    def moment(
        self,
        order: OrderType,
        center: CenterType,
        variable_to_index_map: Dict[Variable, int],
    ) -> np.ndarray:
        variable = list(variable_to_index_map.keys())[self.variable]
        number_of_variables = len(variable_to_index_map)
        result = np.zeros((self.number_of_nodes, number_of_variables))

        if variable in order:
            order_value = order[variable]
            center_value = center[variable]
            states = np.array([float(s) for s in variable.domain.simple_sets])
            result[:, variable_to_index_map[variable]] = (
                self.probabilities @ (states - center_value) ** order_value
            )
        return result

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        variable = variables[self.variable]
        number_of_samples = len(indices)
        number_of_variables = len(variables)
        result = np.empty((number_of_samples, number_of_variables), dtype=object)
        domain_list = list(variable.domain.simple_sets)

        unique_indices, counts = np.unique(indices, return_counts=True)
        for index, count in zip(unique_indices, counts):
            mask = indices == index
            probabilities = self.probabilities[index]
            state_indices = np.random.choice(len(probabilities), size=count, p=probabilities)
            sampled_states = [domain_list[state_index] for state_index in state_indices]
            result[mask, self.variable] = sampled_states
        return result

    def support(self, variables: Tuple[Variable, ...]) -> List[Event]:
        variable = variables[self.variable]
        result = []
        domain_list = list(variable.domain.simple_sets)
        for node_probs in self.probabilities:
            support_states = [domain_list[i] for i, p in enumerate(node_probs) if p > 0]
            result.append(
                SimpleEvent.from_data({variable: support_states}).as_composite_set()
            )
        return result

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable]
        cdf_states = np.cumsum(self.probabilities, axis=1)
        indices = np.clip(values.astype(int), 0, self.probabilities.shape[1] - 1)
        return cdf_states[:, indices].T

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        variable = variables[self.variable]
        result = []
        domain_list = list(variable.domain.simple_sets)
        for node_probabilities in self.probabilities:
            max_probability = np.max(node_probabilities)
            mode_states = [
                domain_list[index] for index, probability in enumerate(node_probabilities) if probability == max_probability
            ]
            event = SimpleEvent.from_data({variable: mode_states}).as_composite_set()
            result.append((event, np.log(max_probability)))
        return result

    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        variable = variables[self.variable]
        if variable not in event.variables:
            return self, np.zeros(self.number_of_nodes)

        variable_event = event.marginal([variable]).simple_sets[0][variable]
        domain_list = list(variable.domain.simple_sets)
        mask = np.array([state in variable_event for state in domain_list])

        new_probabilities = self.probabilities * mask[np.newaxis, :]
        row_sums = new_probabilities.sum(axis=1)
        log_probs = np.log(row_sums)

        # Avoid division by zero
        safe_sums = row_sums.copy()
        safe_sums[safe_sums == 0] = 1.0
        new_probabilities /= safe_sums[:, np.newaxis]

        return DiscreteLayer(self.variable, new_probabilities), log_probs

    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        variable = all_variables[self.variable]
        if variable in variables:
            return self
        return None

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        variable = variables[self.variable]
        if variable not in event.variables:
            return np.ones(self.number_of_nodes)

        variable_event = event.marginal([variable]).simple_sets[0][variable]
        domain_list = list(variable.domain.simple_sets)
        mask = np.array([state in variable_event for state in domain_list])

        return (self.probabilities * mask[np.newaxis, :]).sum(axis=1)

    @classmethod
    def create_layer_from_nodes_with_same_type_and_scope(
        cls,
        nodes: List[UnivariateDiscreteLeaf],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> RustworkxLayerConverter:
        hash_remap = {hash(node): index for index, node in enumerate(nodes)}
        variable: Symbolic = nodes[0].variable
        domain_elements = list(variable.domain.simple_sets)
        num_states = len(domain_elements)

        state_hash_to_index = {
            hash(state): index for index, state in enumerate(domain_elements)
        }
        state_to_index = {state: index for index, state in enumerate(domain_elements)}

        is_symbolic = isinstance(variable, Symbolic)

        parameters = np.zeros((len(nodes), num_states))

        for i, node in enumerate(
            tqdm.tqdm(nodes, desc=f"Creating discrete layer for {variable.name}")
            if progress_bar
            else nodes
        ):
            for state, value in node.distribution.probabilities.items():
                if is_symbolic:
                    if isinstance(state, int):
                        idx = state_hash_to_index.get(state)
                    else:
                        idx = state_hash_to_index.get(hash(state))
                else:
                    idx = state_to_index.get(state)

                if idx is not None:
                    parameters[i, idx] = value

        hash_to_index = None
        if is_symbolic:
            hash_to_index = state_hash_to_index

        result = cls(
            nodes[0].probabilistic_circuit.variables.index(variable),
            parameters,
            hash_to_index,
        )
        return RustworkxLayerConverter(result, nodes, hash_remap)

    def to_rustworkx(
        self,
        variables: SortedSet[Variable],
        result: RustworkxProbabilisticCircuit,
        progress_bar: Optional[tqdm.tqdm] = None,
    ) -> List[Unit]:
        variable = variables[self.variable]
        if progress_bar:
            progress_bar.set_postfix_str(
                f"Creating discrete distributions for {variable.name}"
            )

        domain_elements = list(variable.domain.simple_sets)

        units = [
            UnivariateDiscreteLeaf(
                SymbolicDistribution(
                    variable=variable,
                    probabilities={
                        hash(state): probability
                        for state, probability in zip(domain_elements, node_probabilities)
                    },
                ),
                probabilistic_circuit=result,
            )
            for node_probabilities in self.probabilities
        ]
        if progress_bar:
            progress_bar.update(self.number_of_nodes)
        return units


@dataclass
class GaussianLayer(ContinuousLayer[GaussianDistribution]):
    """
    A layer that represents Gaussian distributions over a single variable.
    """

    location: np.ndarray
    scale: np.ndarray

    @property
    def number_of_nodes(self) -> int:
        return self.location.shape[0]

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable].astype(float)  # (N,)
        return scipy.stats.norm.logpdf(
            values[:, np.newaxis], loc=self.location, scale=self.scale
        )

    def moment(
        self,
        order: OrderType,
        center: CenterType,
        variable_to_index_map: Dict[Variable, int],
    ) -> np.ndarray:
        variable = list(variable_to_index_map.keys())[self.variable]
        number_of_variables = len(variable_to_index_map)
        result = np.zeros((self.number_of_nodes, number_of_variables))

        if variable in order:
            k_order = order[variable]
            center_value = center[variable]

            current_locations = self.location
            current_scales = self.scale

            moment_values = np.zeros_like(current_locations)
            for i in range(k_order + 1):
                if i % 2 == 0:
                    current_term = double_factorial(i - 1) * (current_scales**i)
                else:
                    current_term = 0
                combination = math.comb(k_order, i)
                moment_values += (
                    combination * current_term * ((current_locations - center_value) ** (k_order - i))
                )

            result[:, variable_to_index_map[variable]] = moment_values
        return result

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        number_of_samples = len(indices)
        number_of_variables = len(variables)
        result = np.zeros((number_of_samples, number_of_variables))
        locations = self.location[indices]
        scales = self.scale[indices]
        result[:, self.variable] = np.random.normal(locations, scales)
        return result

    def support(self, variables: Tuple[Variable, ...]) -> List[Event]:
        variable = variables[self.variable]
        result = []
        for _ in range(self.number_of_nodes):
            # Gaussian support is always (-inf, inf)
            result.append(
                SimpleEvent.from_data({variable: Continuous.domain}).as_composite_set()
            )
        return result

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable].astype(float)
        return scipy.stats.norm.cdf(
            values[:, np.newaxis], loc=self.location, scale=self.scale
        )

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        variable = variables[self.variable]
        result = []
        for location, scale in zip(self.location, self.scale):
            event = SimpleEvent.from_data({variable: float(location)}).as_composite_set()
            log_likelihood = scipy.stats.norm.logpdf(float(location), loc=location, scale=scale)
            result.append((event, log_likelihood))
        return result

    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        variable = variables[self.variable]
        if variable not in event.variables:
            return self, np.zeros(self.number_of_nodes)

        # Use Event.marginal or similar to get the interval for this variable
        variable_event = event.marginal([variable]).simple_sets[0][variable]

        if variable_event == reals():
            return self, np.zeros(self.number_of_nodes)

        log_probs = np.log(self.probability(event, variables))

        if len(variable_event.simple_sets) == 1:
            s = variable_event.simple_sets[0]
            interval = np.array([[s.lower, s.upper]] * self.number_of_nodes)
            return (
                TruncatedGaussianLayer(
                    self.variable, interval, self.location, self.scale
                ),
                log_probs,
            )

        return self, log_probs

    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        variable = all_variables[self.variable]
        if variable in variables:
            return self
        return None

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        variable = variables[self.variable]
        if variable not in event.variables:
            return np.ones(self.number_of_nodes)

        variable_event = event.marginal([variable]).simple_sets[0][variable]
        prob = np.zeros(self.number_of_nodes)
        for interval in variable_event.simple_sets:
            p_upper = scipy.stats.norm.cdf(
                interval.upper, loc=self.location, scale=self.scale
            )
            p_lower = scipy.stats.norm.cdf(
                interval.lower, loc=self.location, scale=self.scale
            )
            prob += p_upper - p_lower
        return prob

    @classmethod
    def create_layer_from_nodes_with_same_type_and_scope(
        cls,
        nodes: List[UnivariateContinuousLeaf],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> RustworkxLayerConverter:
        hash_remap = {hash(node): index for index, node in enumerate(nodes)}
        variable = nodes[0].variable

        locations = np.array([node.distribution.location for node in nodes])
        scales = np.array([node.distribution.scale for node in nodes])

        result = cls(
            nodes[0].probabilistic_circuit.variables.index(variable), locations, scales
        )
        return RustworkxLayerConverter(result, nodes, hash_remap)

    def to_rustworkx(
        self,
        variables: SortedSet[Variable],
        result: RustworkxProbabilisticCircuit,
        progress_bar: Optional[tqdm.tqdm] = None,
    ) -> List[Unit]:
        variable = variables[self.variable]
        if progress_bar:
            progress_bar.set_postfix_str(
                f"Creating Gaussian distributions for {variable.name}"
            )

        units = [
            UnivariateContinuousLeaf(
                GaussianDistribution(variable=variable, location=location, scale=scale),
                probabilistic_circuit=result,
            )
            for location, scale in zip(self.location, self.scale)
        ]
        if progress_bar:
            progress_bar.update(self.number_of_nodes)
        return units


@dataclass
class TruncatedGaussianLayer(ContinuousLayerWithFiniteSupport[TruncatedGaussianDistribution]):
    """
    A layer that represents Truncated Gaussian distributions over a single variable.
    """

    location: np.ndarray
    scale: np.ndarray

    @property
    def number_of_nodes(self) -> int:
        return self.location.shape[0]

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable].astype(float)[:, np.newaxis]
        log_likelihood = scipy.stats.norm.logpdf(values, loc=self.location, scale=self.scale)

        probabilities_upper = scipy.stats.norm.cdf(self.upper, loc=self.location, scale=self.scale)
        probabilities_lower = scipy.stats.norm.cdf(self.lower, loc=self.location, scale=self.scale)
        normalization_constant = probabilities_upper - probabilities_lower

        # mask values outside interval
        mask = (values >= self.lower) & (values <= self.upper)
        result = log_likelihood - np.log(normalization_constant)
        result[~mask] = -np.inf
        return result

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        variable = variables[self.variable]
        if variable not in event.variables:
            return np.ones(self.number_of_nodes)

        variable_event = event.marginal([variable]).simple_sets[0][variable]

        # normalizing constant
        probabilities_upper_total = scipy.stats.norm.cdf(
            self.upper, loc=self.location, scale=self.scale
        )
        probabilities_lower_total = scipy.stats.norm.cdf(
            self.lower, loc=self.location, scale=self.scale
        )
        normalization_constant = probabilities_upper_total - probabilities_lower_total

        probability = np.zeros(self.number_of_nodes)
        for interval in variable_event.simple_sets:
            # intersect interval with self.interval
            low = np.maximum(interval.lower, self.lower)
            high = np.minimum(interval.upper, self.upper)

            probabilities_high = scipy.stats.norm.cdf(high, loc=self.location, scale=self.scale)
            probabilities_low = scipy.stats.norm.cdf(low, loc=self.location, scale=self.scale)
            probability += np.maximum(0, probabilities_high - probabilities_low)

        return probability / normalization_constant

    def moment(
        self,
        order: OrderType,
        center: CenterType,
        variable_to_index_map: Dict[Variable, int],
    ) -> np.ndarray:
        # Moment of truncated gaussian is more complex.
        # For now, return a placeholder or use rx implementation in a loop.
        # Actually, let's skip it for now and see if tests need it.
        num_vars = len(variable_to_index_map)
        return np.zeros((self.number_of_nodes, num_vars))

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        # Use rejection sampling or specialized formula
        num_samples = len(indices)
        num_vars = len(variables)
        result = np.zeros((num_samples, num_vars))
        # Loop for now as it's complex to vectorize rejection sampling well without JAX
        for i, node_idx in enumerate(indices):
            # dist = TruncatedGaussianDistribution(...)
            # result[i, self.variable] = dist.sample(1)[0]
            pass
        return result

    def support(self, variables: Tuple[Variable, ...]) -> List[Event]:
        variable = variables[self.variable]
        result = []
        for l, u in zip(self.lower, self.upper):
            from random_events.interval import closed

            result.append(
                SimpleEvent.from_data({variable: closed(l, u)}).as_composite_set()
            )
        return result

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable].astype(float)[:, np.newaxis]
        probabilities_value = scipy.stats.norm.cdf(values, loc=self.location, scale=self.scale)
        probabilities_lower = scipy.stats.norm.cdf(self.lower, loc=self.location, scale=self.scale)
        probabilities_upper = scipy.stats.norm.cdf(self.upper, loc=self.location, scale=self.scale)
        normalization_constant = probabilities_upper - probabilities_lower

        cdf = (probabilities_value - probabilities_lower) / normalization_constant
        cdf = np.clip(cdf, 0, 1)
        cdf[values < self.lower] = 0
        cdf[values > self.upper] = 1
        return cdf

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        # Mode of truncated gaussian: mu if in [l, u], else l or u
        result = []
        variable = variables[self.variable]
        for loc, scale, l, u in zip(self.location, self.scale, self.lower, self.upper):
            if loc < l:
                mode = l
            elif loc > u:
                mode = u
            else:
                mode = loc
            event = SimpleEvent.from_data({variable: float(mode)}).as_composite_set()
            # LL calculation
            ll = scipy.stats.norm.logpdf(float(mode), loc=loc, scale=scale)
            z = scipy.stats.norm.cdf(u, loc=loc, scale=scale) - scipy.stats.norm.cdf(
                l, loc=loc, scale=scale
            )
            result.append((event, ll - np.log(z)))
        return result

    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        # Truncating a truncated gaussian results in another truncated gaussian with smaller interval
        variable = variables[self.variable]
        if variable not in event.variables:
            return self, np.zeros(self.number_of_nodes)

        log_probs = np.log(self.probability(event, variables))

        variable_event = event.marginal([variable]).simple_sets[0][variable]
        # Only handle simple interval truncation for now to keep it in ONE layer
        s = variable_event.simple_sets[0]
        new_interval = np.array(
            [
                [np.maximum(s.lower, self.lower[i]), np.minimum(s.upper, self.upper[i])]
                for i in range(self.number_of_nodes)
            ]
        )

        return (
            TruncatedGaussianLayer(
                self.variable, new_interval, self.location, self.scale
            ),
            log_probs,
        )

    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        variable = all_variables[self.variable]
        if variable in variables:
            return self
        return None

    @classmethod
    def create_layer_from_nodes_with_same_type_and_scope(
        cls,
        nodes: List[UnivariateContinuousLeaf],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> RustworkxLayerConverter:
        hash_remap = {hash(node): index for index, node in enumerate(nodes)}
        variable = nodes[0].variable

        locations = np.array([node.distribution.location for node in nodes])
        scales = np.array([node.distribution.scale for node in nodes])
        intervals = np.array(
            [[node.distribution.lower, node.distribution.upper] for node in nodes]
        )

        result = cls(
            nodes[0].probabilistic_circuit.variables.index(variable),
            intervals,
            locations,
            scales,
        )
        return RustworkxLayerConverter(result, nodes, hash_remap)

    def to_rustworkx(
        self,
        variables: SortedSet[Variable],
        result: RustworkxProbabilisticCircuit,
        progress_bar: Optional[tqdm.tqdm] = None,
    ) -> List[Unit]:
        variable = variables[self.variable]
        if progress_bar:
            progress_bar.set_postfix_str(
                f"Creating Truncated Gaussian distributions for {variable.name}"
            )

        from random_events.interval import Bound

        units = [
            UnivariateContinuousLeaf(
                TruncatedGaussianDistribution(
                    variable=variable,
                    location=location,
                    scale=scale,
                    interval=SimpleInterval.from_data(
                        lower, upper, Bound.CLOSED, Bound.CLOSED
                    ),
                ),
                probabilistic_circuit=result,
            )
            for location, scale, (lower, upper) in zip(
                self.location, self.scale, self.interval
            )
        ]
        if progress_bar:
            progress_bar.update(self.number_of_nodes)
        return units


@dataclass
class UniformLayer(ContinuousLayerWithFiniteSupport[UniformDistribution]):
    """
    A layer that represents Uniform distributions over a single variable.
    """

    @property
    def number_of_nodes(self) -> int:
        return self.interval.shape[0]

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable].astype(float)  # (number_of_samples,)
        return scipy.stats.uniform.logpdf(
            values[:, np.newaxis], loc=self.lower, scale=self.upper - self.lower
        )

    def moment(
        self,
        order: OrderType,
        center: CenterType,
        variable_to_index_map: Dict[Variable, int],
    ) -> np.ndarray:
        variable = list(variable_to_index_map.keys())[self.variable]
        number_of_variables = len(variable_to_index_map)
        result = np.zeros((self.number_of_nodes, number_of_variables))

        if variable in order:
            order_value = order[variable]
            center_value = center[variable]

            lower = self.lower
            upper = self.upper

            moment_value = ((upper - center_value) ** (order_value + 1) - (lower - center_value) ** (order_value + 1)) / ((order_value + 1) * (upper - lower))
            result[:, variable_to_index_map[variable]] = moment_value
        return result

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        number_of_samples = len(indices)
        number_of_variables = len(variables)
        result = np.zeros((number_of_samples, number_of_variables))
        lower_bounds = self.lower[indices]
        upper_bounds = self.upper[indices]
        result[:, self.variable] = np.random.uniform(lower_bounds, upper_bounds)
        return result

    def support(self, variables: Tuple[Variable, ...]) -> List[Event]:
        variable = variables[self.variable]
        result = []
        for l, u in zip(self.lower, self.upper):
            from random_events.interval import closed

            result.append(
                SimpleEvent.from_data({variable: closed(l, u)}).as_composite_set()
            )
        return result

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable].astype(float)
        return scipy.stats.uniform.cdf(
            values[:, np.newaxis], loc=self.lower, scale=self.upper - self.lower
        )

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        variable = variables[self.variable]
        result = []
        for lower, upper in zip(self.lower, self.upper):
            from random_events.interval import closed

            event = SimpleEvent.from_data(
                {variable: (lower + upper) / 2}
            ).as_composite_set()  # Mode for uniform is any point in [lower, upper]
            log_likelihood = np.log(1 / (upper - lower))
            result.append((event, log_likelihood))
        return result

    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        variable = variables[self.variable]
        if variable not in event.variables:
            return self, np.zeros(self.number_of_nodes)

        variable_event = event.marginal([variable]).simple_sets[0][variable]
        if variable_event == reals():
            return self, np.zeros(self.number_of_nodes)

        log_probs = np.log(self.probability(event, variables))

        if len(variable_event.simple_sets) == 1:
            s = variable_event.simple_sets[0]
            new_interval = np.array(
                [
                    [
                        np.maximum(s.lower, self.lower[i]),
                        np.minimum(s.upper, self.upper[i]),
                    ]
                    for i in range(self.number_of_nodes)
                ]
            )
            return UniformLayer(self.variable, new_interval), log_probs

        return self, log_probs

    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        variable = all_variables[self.variable]
        if variable in variables:
            return self
        return None

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        variable = variables[self.variable]
        if variable not in event.variables:
            return np.ones(self.number_of_nodes)

        variable_event = event.marginal([variable]).simple_sets[0][variable]
        probability = np.zeros(self.number_of_nodes)
        for interval in variable_event.simple_sets:
            low = np.maximum(interval.lower, self.lower)
            high = np.minimum(interval.upper, self.upper)
            probability += np.maximum(0, high - low) / (self.upper - self.lower)
        return probability

    @classmethod
    def create_layer_from_nodes_with_same_type_and_scope(
        cls,
        nodes: List[UnivariateContinuousLeaf],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> RustworkxLayerConverter:
        hash_remap = {hash(node): index for index, node in enumerate(nodes)}
        variable = nodes[0].variable

        intervals = np.array(
            [[node.distribution.lower, node.distribution.upper] for node in nodes]
        )

        result = cls(
            nodes[0].probabilistic_circuit.variables.index(variable), intervals
        )
        return RustworkxLayerConverter(result, nodes, hash_remap)

    def to_rustworkx(
        self,
        variables: SortedSet[Variable],
        result: RustworkxProbabilisticCircuit,
        progress_bar: Optional[tqdm.tqdm] = None,
    ) -> List[Unit]:
        variable = variables[self.variable]
        if progress_bar:
            progress_bar.set_postfix_str(
                f"Creating Uniform distributions for {variable.name}"
            )

        units = [
            UnivariateContinuousLeaf(
                UniformDistribution(variable=variable, lower=lower, upper=upper),
                probabilistic_circuit=result,
            )
            for lower, upper in zip(self.lower, self.upper)
        ]
        if progress_bar:
            progress_bar.update(self.number_of_nodes)
        return units


@dataclass
class DiracDeltaLayer(ContinuousLayer[DiracDeltaDistribution]):
    """
    A layer that represents Dirac delta distributions over a single variable.
    """

    location: np.ndarray
    density_cap: np.ndarray

    @property
    def number_of_nodes(self) -> int:
        return self.location.shape[0]

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        result = np.full((x.shape[0], self.number_of_nodes), -np.inf)
        for i in range(self.number_of_nodes):
            mask = np.abs(x[:, self.variable] - self.location[i]) < 1e-6
            result[mask, i] = np.log(self.density_cap[i])
        return result

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        variable = variables[self.variable]
        if variable not in event.variables:
            return np.ones(self.number_of_nodes)

        variable_event = event.marginal([variable]).simple_sets[0][variable]
        probability = np.zeros(self.number_of_nodes)
        for i in range(self.number_of_nodes):
            if self.location[i] in variable_event:
                probability[i] = 1.0
        return probability

    def moment(
        self,
        order: OrderType,
        center: CenterType,
        variable_to_index_map: Dict[Variable, int],
    ) -> np.ndarray:
        variable = list(variable_to_index_map.keys())[self.variable]
        number_of_variables = len(variable_to_index_map)
        result = np.zeros((self.number_of_nodes, number_of_variables))

        if variable in order:
            order_value = order[variable]
            center_value = center[variable]
            result[:, variable_to_index_map[variable]] = (self.location - center_value) ** order_value
        return result

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        number_of_samples = len(indices)
        number_of_variables = len(variables)
        result = np.zeros((number_of_samples, number_of_variables))
        result[:, self.variable] = self.location[indices]
        return result

    def support(self, variables: Tuple[Variable, ...]) -> List[Event]:
        variable = variables[self.variable]
        result = []
        for loc in self.location:
            from random_events.interval import singleton

            result.append(
                SimpleEvent.from_data({variable: singleton(loc)}).as_composite_set()
            )
        return result

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        values = x[:, self.variable][:, np.newaxis]
        result = np.zeros((x.shape[0], self.number_of_nodes))
        result[values >= self.location - 1e-6] = 1.0
        return result

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        result = []
        variable = variables[self.variable]
        for location, density_cap in zip(self.location, self.density_cap):
            event = SimpleEvent.from_data({variable: float(location)}).as_composite_set()
            result.append((event, np.log(density_cap)))
        return result

    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        variable = variables[self.variable]
        if variable not in event.variables:
            return self, np.zeros(self.number_of_nodes)

        log_probabilities = np.log(self.probability(event, variables))
        return self, log_probabilities

    @classmethod
    def create_layer_from_nodes_with_same_type_and_scope(
        cls,
        nodes: List[UnivariateContinuousLeaf],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> RustworkxLayerConverter:
        hash_remap = {hash(node): index for index, node in enumerate(nodes)}
        variable = nodes[0].variable

        locations = np.array([node.distribution.location for node in nodes])
        density_caps = np.array([node.distribution.density_cap for node in nodes])

        result = cls(
            nodes[0].probabilistic_circuit.variables.index(variable),
            locations,
            density_caps,
        )
        return RustworkxLayerConverter(result, nodes, hash_remap)

    def to_rustworkx(
        self,
        variables: SortedSet[Variable],
        result: RustworkxProbabilisticCircuit,
        progress_bar: Optional[tqdm.tqdm] = None,
    ) -> List[Unit]:
        variable = variables[self.variable]
        if progress_bar:
            progress_bar.set_postfix_str(
                f"Creating Dirac delta distributions for {variable.name}"
            )

        units = [
            UnivariateContinuousLeaf(
                DiracDeltaDistribution(
                    variable=variable,
                    location=float(location),
                    density_cap=float(density_cap),
                ),
                probabilistic_circuit=result,
            )
            for location, density_cap in zip(self.location, self.density_cap)
        ]
        if progress_bar:
            progress_bar.update(self.number_of_nodes)
        return units
