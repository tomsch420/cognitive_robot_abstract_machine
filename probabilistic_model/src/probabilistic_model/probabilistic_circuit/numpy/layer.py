from __future__ import annotations

import inspect
from abc import abstractmethod, ABC
from dataclasses import dataclass, field
import numpy as np
import scipy.sparse
from krrood.adapters.json_serializer import (
    SubclassJSONSerializer,
    recursive_subclasses,
    DataclassJSONSerializer,
)
from typing import (
    List,
    Tuple,
    Optional,
    Any,
    Dict,
    Union,
    Type,
    Iterator,
    Generic,
)
from typing_extensions import (
    Self,
    get_origin,
    get_args,
    TypeVar,
)
from krrood.patterns.subclass_safe_generic import SubClassSafeGeneric
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    Unit,
    SumUnit,
    ProductUnit,
    ProbabilisticCircuit as RustworkxProbabilisticCircuit,
)
from probabilistic_model.exceptions import ShapeMismatchError
from random_events.variable import Variable
from sortedcontainers import SortedSet
from probabilistic_model.probabilistic_model import OrderType, CenterType
from random_events.product_algebra import Event
import tqdm


from probabilistic_model.probabilistic_circuit.numpy.conversion import RustworkxLayerConverter


T = TypeVar("T")


@dataclass
class Layer(Generic[T], SubClassSafeGeneric, SubclassJSONSerializer, ABC):
    """
    Abstract class for Layers of a layered circuit.

    Layers have the same scope (set of variables) for every node in them.
    """

    variables: np.ndarray = field(default=None, init=False)
    """
    The variable indices of the layer.
    """

    def to_json(self, **kwargs) -> Dict[str, Any]:
        return DataclassJSONSerializer.to_json(self, **kwargs)

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        return DataclassJSONSerializer.from_json(data, cls, **kwargs)


    @abstractmethod
    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        """
        Calculate the log-likelihood of the nodes in the layer.

        :param x: The input data of shape (N, D).
        :return: The log-likelihood of every node in the layer for every sample in x.
            Shape (N, number_of_nodes).
        """

    @property
    @abstractmethod
    def number_of_nodes(self) -> int:
        """
        :return: The number of nodes in the layer.
        """

    def all_layers(self) -> List[Layer]:
        """
        :return: A list of all layers in the circuit.
        """
        return [self]

    @property
    def number_of_components(self) -> int:
        """
        :return: The number of components (leaves + edges) of the entire circuit
        """
        return self.number_of_nodes

    @classmethod
    def rustworkx_classes(cls) -> Tuple[Type[Unit], ...]:
        """
        :return: The rustworkx classes that this layer represents.
        """
        parameters = cls.get_generic_type_parameters()
        if not parameters:
            return ()

        bound_type = parameters[0]
        if get_origin(bound_type) is Union:
            return get_args(bound_type)
        return (bound_type,)


    @classmethod
    @abstractmethod
    def create_layer_from_nodes_with_same_type_and_scope(
        cls,
        nodes: List[Unit],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> RustworkxLayerConverter:
        """
        Create a layer from a list of nodes with the same type and scope.
        """

    @abstractmethod
    def moment(
        self,
        order: OrderType,
        center: CenterType,
        variable_to_index_map: Dict[Variable, int],
    ) -> np.ndarray:
        """
        Calculate the moments of the nodes in the layer.

        :param order: The order of the moments.
        :param center: The center of the moments.
        :param variable_to_index_map: A map from variables to their indices in the
            result array.
        :return: The moments of every node in the layer. Shape (number_of_nodes,
            number_of_variables).
        """

    @abstractmethod
    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        """
        Sample from the nodes in the layer.

        :param indices: The indices of the nodes to sample from. Shape (N,).
        :param variables: The variables of the circuit.
        :return: The samples. Shape (N, D).
        """

    @abstractmethod
    def support(self, variables: Tuple[Variable, ...]) -> List[Event]:
        """
        :param variables: The variables of the circuit.
        :return: A list of the supports of every node in the layer.
        """

    @abstractmethod
    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        """
        Calculate the cumulative distribution function of the nodes in the layer.

        :param x: The input data of shape (N, D).
        :return: The CDF of every node in the layer. Shape (N, number_of_nodes).
        """

    @abstractmethod
    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        """
        :param variables: The variables of the circuit.
        :return: A list of the log-modes and their log-likelihoods of every node in the layer.
        """

    @abstractmethod
    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        """
        :param event: The event to truncate to.
        :param variables: The variables of the circuit.
        :return: The truncated layer and the log-probabilities of the nodes.
        """

    @abstractmethod
    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        """
        :param event: The event to calculate the probability of.
        :param variables: The variables of the circuit.
        :return: The probability of the event for every node in the layer. Shape (number_of_nodes,).
        """

    @abstractmethod
    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        """
        :param variables: The variables to marginalize to.
        :param all_variables: The variables of the circuit.
        :return: The marginal layer.
        """


@dataclass
class InnerLayer(Layer[T], ABC):
    """
    Abstract Base Class for inner layers.
    """

    child_layers: List[Layer]
    """
    The child layers of this layer.
    """

    def all_layers(self) -> List[Layer]:
        result = [self]
        for child_layer in self.child_layers:
            result.extend(child_layer.all_layers())
        return result



@dataclass
class InputLayer(Layer[T], ABC):
    """
    Abstract base class for univariate input units.
    """

    input_variable: int

    def __post_init__(self):
        self.variables = np.array([self.input_variable])

    @property
    def variable(self) -> int:
        return self.variables[0]

    @property
    @abstractmethod
    def number_of_nodes(self) -> int:
        """
        :return: The number of nodes in the layer.
        """


@dataclass
class SumLayer(InnerLayer[T], ABC):
    """
    Abstract base class for sum layers.
    """

    def __post_init__(self):
        if self.variables is None:
            self.variables = self.child_layers[0].variables


@dataclass
class SparseSumLayer(SumLayer[SumUnit]):
    """
    A SumLayer that uses SciPy Sparse matrices for weights.
    """

    weights: List[scipy.sparse.csr_matrix]
    """
    The weights of the sum units for each child layer.

    Each element is a sparse matrix of shape (number_of_nodes,
    child_layer.number_of_nodes).
    """

    @property
    def number_of_nodes(self) -> int:
        return self.weights[0].shape[0]

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        child_log_likelihoods = [child.log_likelihood_of_nodes(x) for child in self.child_layers]

        # Stability trick: subtract max
        maximum_log_likelihood = np.max([np.max(child_ll, axis=1) for child_ll in child_log_likelihoods], axis=0)

        # mask for samples where all children have -inf log likelihood
        inf_mask = maximum_log_likelihood == -np.inf

        total_probability = np.zeros((x.shape[0], self.number_of_nodes))

        # for samples with -inf, we don't care about the exp value as long as it's not nan
        # we can just use 0 as m for those samples
        safe_maximum = np.where(inf_mask, 0.0, maximum_log_likelihood)

        for child_log_likelihood, weights in zip(child_log_likelihoods, self.weights):
            probability = np.exp(child_log_likelihood - safe_maximum[:, np.newaxis])
            total_probability += probability @ weights.T

        result = np.log(total_probability) + safe_maximum[:, np.newaxis]
        result[inf_mask, :] = -np.inf
        return result

    def moment(
        self,
        order: OrderType,
        center: CenterType,
        variable_to_index_map: Dict[Variable, int],
    ) -> np.ndarray:
        child_moments = [
            child.moment(order, center, variable_to_index_map)
            for child in self.child_layers
        ]
        number_of_variables = len(variable_to_index_map)
        result = np.zeros((self.number_of_nodes, number_of_variables))
        for child_moment, weights in zip(child_moments, self.weights):
            result += weights @ child_moment
        return result

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        number_of_samples = len(indices)
        number_of_variables = len(variables)

        # Concatenate weights from all child layers
        concatenated_weights = scipy.sparse.hstack(self.weights).tocsr()

        unique_indices, counts = np.unique(indices, return_counts=True)
        new_child_indices = np.empty(number_of_samples, dtype=int)

        for index, count in zip(unique_indices, counts):
            mask = indices == index
            row = concatenated_weights[index, :]
            if row.nnz == 0:
                new_child_indices[mask] = 0
                continue
            new_child_indices[mask] = np.random.choice(
                row.indices, size=count, p=row.data / row.data.sum()
            )

        child_layer_offsets = np.cumsum(
            [0] + [child.number_of_nodes for child in self.child_layers]
        )

        from random_events.variable import Continuous, Integer

        has_symbolic = any(not isinstance(variable, (Continuous, Integer)) for variable in variables)
        dtype = object if has_symbolic else float

        result = np.zeros((number_of_samples, number_of_variables), dtype=dtype)
        for i, (child, start, end) in enumerate(
            zip(self.child_layers, child_layer_offsets[:-1], child_layer_offsets[1:])
        ):
            mask = (new_child_indices >= start) & (new_child_indices < end)
            if np.any(mask):
                local_indices = new_child_indices[mask] - start
                result[mask] = child.sample(local_indices, variables)

        return result

    def support(self, variables: Tuple[Variable, ...]) -> List[Event]:
        child_supports = [child.support(variables) for child in self.child_layers]
        result = []
        for i in range(self.number_of_nodes):
            node_support = None
            for layer_index, weight_matrix in enumerate(self.weights):
                row = weight_matrix[i, :]
                for child_index in row.indices:
                    support_event = child_supports[layer_index][child_index]
                    if node_support is None:
                        node_support = support_event.__deepcopy__()
                    else:
                        node_support |= support_event
            result.append(node_support)
        return result

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        child_cdfs = [
            child.cumulative_distribution_function(x) for child in self.child_layers
        ]
        result = np.zeros((x.shape[0], self.number_of_nodes))
        for child_cdf, weights in zip(child_cdfs, self.weights):
            result += child_cdf @ weights.T
        return result

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        child_log_modes = [child.log_mode(variables) for child in self.child_layers]
        result = []
        for i in range(self.number_of_nodes):
            best_log_likelihood = -np.inf
            best_event = None

            for layer_index, weight_matrix in enumerate(self.weights):
                row = weight_matrix[i, :]
                for child_index in row.indices:
                    weight = weight_matrix[i, child_index]
                    event, log_likelihood = child_log_modes[layer_index][child_index]
                    combined_log_likelihood = np.log(weight) + log_likelihood
                    if combined_log_likelihood > best_log_likelihood:
                        best_log_likelihood = combined_log_likelihood
                        best_event = event
                    elif combined_log_likelihood == best_log_likelihood and best_event is not None:
                        best_event |= event
            result.append((best_event, best_log_likelihood))
        return result

    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        # Recursively truncate children
        truncated_children_results = [
            child.log_truncated(event, variables) for child in self.child_layers
        ]

        new_child_layers = []
        child_log_probabilities = []
        for child, log_probabilities in truncated_children_results:
            new_child_layers.append(child)
            child_log_probabilities.append(log_probabilities)

        # Update weights: new_weight = old_weight * exp(child_log_probability)
        new_weights = []
        overall_log_probabilities = np.full(self.number_of_nodes, -np.inf)

        for weight_matrix, child_log_probability in zip(self.weights, child_log_probabilities):
            # weight_matrix is (number_of_nodes, number_of_child_nodes)
            # child_log_probability is (number_of_child_nodes,)
            # weight_matrix_new = weight_matrix * diag(exp(child_log_probability))
            weight_matrix_new = weight_matrix.multiply(np.exp(child_log_probability))
            new_weights.append(weight_matrix_new)

            # Update overall log probabilities
            # row_sums = sum_j old_weight_j * exp(child_log_probability_j)
            row_sums = np.array(weight_matrix_new.sum(axis=1)).flatten()
            row_log_sums = np.full_like(row_sums, -np.inf)
            mask = row_sums > 0
            row_log_sums[mask] = np.log(row_sums[mask])
            overall_log_probabilities = np.logaddexp(overall_log_probabilities, row_log_sums)

        # Normalize new weights
        final_weights = []
        safe_sums = np.exp(overall_log_probabilities)
        safe_sums[safe_sums == 0] = 1.0
        inverse_sums = 1.0 / safe_sums
        for weight_matrix_new in new_weights:
            from scipy.sparse import diags

            weight_matrix_final = diags(inverse_sums) @ weight_matrix_new
            final_weights.append(weight_matrix_final)

        return SparseSumLayer(new_child_layers, final_weights), overall_log_probabilities

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        child_probabilities = [
            child.probability(event, variables) for child in self.child_layers
        ]
        result = np.zeros(self.number_of_nodes)
        for child_probability, weight_matrix in zip(child_probabilities, self.weights):
            result += weight_matrix @ child_probability
        return result

    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        new_child_layers = []
        new_weights = []
        for child, weight_matrix in zip(self.child_layers, self.weights):
            marginal_layer = child.marginal(variables, all_variables)
            if marginal_layer is not None:
                new_child_layers.append(marginal_layer)
                new_weights.append(weight_matrix)

        if not new_child_layers:
            return None

        return SparseSumLayer(new_child_layers, new_weights)

    @property
    def number_of_components(self) -> int:
        return sum([child_layer.number_of_components for child_layer in self.child_layers]) + sum(
            [weight_matrix.nnz for weight_matrix in self.weights]
        )

    @classmethod
    def create_layer_from_nodes_with_same_type_and_scope(
        cls,
        nodes: List[SumUnit],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> RustworkxLayerConverter:

        result_hash_remap = {hash(node): index for index, node in enumerate(nodes)}
        variables = np.array(
            [
                nodes[0].probabilistic_circuit.variables.index(variable)
                for variable in nodes[0].variables
            ]
        )

        number_of_nodes = len(nodes)

        # filter the child layers to only contain layers with the same scope as this one
        filtered_child_layers = [
            child_layer
            for child_layer in child_layers
            if np.array_equal(child_layer.layer.variables, variables)
        ]
        weights = []

        # for every possible child layer
        for child_layer in filtered_child_layers:

            # initialize indices and values for sparse weight matrix
            rows = []
            cols = []
            data = []

            # gather indices and log weights
            for index, node in enumerate(
                tqdm.tqdm(nodes, desc="Calculating weights for sum layer")
                if progress_bar
                else nodes
            ):
                for log_weight, subcircuit in node.log_weighted_subcircuits:
                    if hash(subcircuit) in child_layer.hash_remap:
                        rows.append(index)
                        cols.append(child_layer.hash_remap[hash(subcircuit)])
                        data.append(np.exp(log_weight))

            # assemble sparse weight matrix
            weights.append(
                scipy.sparse.csr_matrix(
                    (data, (rows, cols)),
                    shape=(number_of_nodes, child_layer.layer.number_of_nodes),
                )
            )

        sum_layer = cls([child_layer.layer for child_layer in filtered_child_layers], weights)
        return RustworkxLayerConverter(sum_layer, nodes, result_hash_remap)

    def to_rustworkx(
        self,
        variables: SortedSet[Variable],
        result: RustworkxProbabilisticCircuit,
        progress_bar: Optional[tqdm.tqdm] = None,
    ) -> List[Unit]:

        variables_ = [variables[i] for i in self.variables]

        if progress_bar:
            progress_bar.set_postfix_str(
                f"Parsing Sum Layer for variables {variables_}"
            )

        units = [
            SumUnit(probabilistic_circuit=result) for _ in range(self.number_of_nodes)
        ]

        child_layer_rustworkx = [
            child_layer.to_rustworkx(variables, result, progress_bar) for child_layer in self.child_layers
        ]

        for weight_matrix, child_layer in zip(self.weights, child_layer_rustworkx):
            # extract the weights for the child layer
            coordinate_matrix = weight_matrix.tocoo()
            for row, column, weight in zip(coordinate_matrix.row, coordinate_matrix.col, coordinate_matrix.data):
                units[row].add_subcircuit(child_layer[column], np.log(weight))
                if progress_bar:
                    progress_bar.update()

        return units

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["weights"] = []
        for weight_matrix in self.weights:
            coordinate_matrix = weight_matrix.tocoo()
            result["weights"].append(
                {
                    "data": coordinate_matrix.data.tolist(),
                    "row": coordinate_matrix.row.tolist(),
                    "col": coordinate_matrix.col.tolist(),
                    "shape": coordinate_matrix.shape,
                }
            )
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        child_layers = [Layer.from_json(child_layer, **kwargs) for child_layer in data["child_layers"]]
        weights = []
        for weight_data in data["weights"]:
            weight_matrix = scipy.sparse.coo_matrix(
                (weight_data["data"], (weight_data["row"], weight_data["col"])), shape=weight_data["shape"]
            ).tocsr()
            weights.append(weight_matrix)
        return cls(child_layers, weights)


@dataclass
class ProductLayer(InnerLayer[ProductUnit]):
    """
    A layer that represents the product of multiple other units.
    """

    edges: np.ndarray
    """
    The edges from this layer to child layers.

    Shape: (len(child_layers), number_of_nodes)
    Each entry edges[i, j] is the index of the node in child_layers[i] that node j in this layer uses.
    """

    @property
    def number_of_nodes(self) -> int:
        return self.edges.shape[1]

    def __post_init__(self):
        if self.variables is None:
            variables = np.concatenate([child_layer.variables for child_layer in self.child_layers])
            self.variables = np.sort(np.unique(variables))

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        number_of_samples = x.shape[0]
        result = np.zeros((number_of_samples, self.number_of_nodes))

        for child_layer, edge_indices in zip(self.child_layers, self.edges):
            child_log_likelihood = child_layer.log_likelihood_of_nodes(x)
            result += child_log_likelihood[:, edge_indices]

        return result

    def moment(
        self,
        order: OrderType,
        center: CenterType,
        variable_to_index_map: Dict[Variable, int],
    ) -> np.ndarray:
        child_moments = [
            child_layer.moment(order, center, variable_to_index_map)
            for child_layer in self.child_layers
        ]
        number_of_variables = len(variable_to_index_map)
        result = np.zeros((self.number_of_nodes, number_of_variables))
        for child_moment, edge_indices in zip(child_moments, self.edges):
            result += child_moment[edge_indices, :]
        return result

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        number_of_samples = len(indices)
        number_of_variables = len(variables)

        from random_events.variable import Continuous, Integer

        has_symbolic = any(not isinstance(variable, (Continuous, Integer)) for variable in variables)
        dtype = object if has_symbolic else float
        result = np.zeros((number_of_samples, number_of_variables), dtype=dtype)

        for child_layer, edge_indices in zip(self.child_layers, self.edges):
            local_indices = edge_indices[indices]
            child_sample = child_layer.sample(local_indices, variables)
            for variable_index in child_layer.variables:
                result[:, variable_index] = child_sample[:, variable_index]
        return result

    def support(self, variables: Tuple[Variable, ...]) -> List[Event]:
        child_supports = [child.support(variables) for child in self.child_layers]
        result = []
        circuit_variables = SortedSet(variables)
        for j in range(self.number_of_nodes):
            node_support = child_supports[0][self.edges[0, j]]
            node_support.fill_missing_variables(circuit_variables)
            for i in range(1, len(self.child_layers)):
                node_support &= child_supports[i][self.edges[i, j]]
            result.append(node_support)
        return result

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        number_of_samples = x.shape[0]
        result = np.ones((number_of_samples, self.number_of_nodes))
        for child_layer, edge_indices in zip(self.child_layers, self.edges):
            child_cdf = child_layer.cumulative_distribution_function(x)
            result *= child_cdf[:, edge_indices]
        return result

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        child_log_modes = [child_layer.log_mode(variables) for child_layer in self.child_layers]
        result = []
        for j in range(self.number_of_nodes):
            mode_event, mode_log_likelihood = child_log_modes[0][self.edges[0, j]]
            for i in range(1, len(self.child_layers)):
                other_event, other_log_likelihood = child_log_modes[i][self.edges[i, j]]
                mode_event &= other_event
                mode_log_likelihood += other_log_likelihood
            result.append((mode_event, mode_log_likelihood))
        return result

    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        truncated_children_results = [
            child.log_truncated(event, variables) for child in self.child_layers
        ]

        new_child_layers = []
        child_log_probabilities = []
        for child, log_probabilities in truncated_children_results:
            new_child_layers.append(child)
            child_log_probabilities.append(log_probabilities)

        # New ProductLayer
        new_layer = ProductLayer(new_child_layers, self.edges)

        # Overall log probabilities: sum of child log probabilities
        overall_log_probabilities = np.zeros(self.number_of_nodes)
        for i, child_log_probabilities_in_layer in enumerate(child_log_probabilities):
            overall_log_probabilities += child_log_probabilities_in_layer[self.edges[i, :]]

        return new_layer, overall_log_probabilities

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        child_probabilities = [
            child_layer.probability(event, variables) for child_layer in self.child_layers
        ]
        result = np.ones(self.number_of_nodes)
        for child_probability, edge_indices in zip(child_probabilities, self.edges):
            result *= child_probability[edge_indices]
        return result

    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        new_child_layers = []
        new_edge_indices = []
        for i, child_layer in enumerate(self.child_layers):
            marginal_layer = child_layer.marginal(variables, all_variables)
            if marginal_layer is not None:
                new_child_layers.append(marginal_layer)
                new_edge_indices.append(i)

        if not new_child_layers:
            return None

        return ProductLayer(new_child_layers, self.edges[new_edge_indices, :])

    @classmethod
    def create_layer_from_nodes_with_same_type_and_scope(
        cls,
        nodes: List[ProductUnit],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> RustworkxLayerConverter:

        hash_remap = {hash(node): index for index, node in enumerate(nodes)}
        number_of_nodes = len(nodes)

        edges = np.zeros((len(child_layers), number_of_nodes), dtype=int)

        if progress_bar:
            progress_bar_instance = tqdm.tqdm(total=number_of_nodes, desc="Assembling Product Layer")
        # for every node in the nodes for this layer
        for node_index, node in enumerate(nodes):

            # for every child layer
            for child_layer_index, child_layer in enumerate(child_layers):
                child_layer_variables = SortedSet(
                    [
                        node.probabilistic_circuit.variables[index]
                        for index in child_layer.layer.variables
                    ]
                )

                # for every subcircuit
                for subcircuit in node.subcircuits:
                    # if the scopes are compatible
                    if child_layer_variables == subcircuit.variables:
                        # add the edge
                        edges[child_layer_index, node_index] = child_layer.hash_remap[
                            hash(subcircuit)
                        ]
            if progress_bar:
                progress_bar_instance.update(1)

        layer = cls([child_layer.layer for child_layer in child_layers], edges)
        return RustworkxLayerConverter(layer, nodes, hash_remap)

    def to_rustworkx(
        self,
        variables: SortedSet[Variable],
        result: RustworkxProbabilisticCircuit,
        progress_bar: Optional[tqdm.tqdm] = None,
    ) -> List[Unit]:

        if result is None:
            result = RustworkxProbabilisticCircuit()

        variables_ = [variables[i] for i in self.variables]
        if progress_bar:
            progress_bar.set_postfix_str(
                f"Parsing Product Layer of variables {variables_}"
            )

        units = [
            ProductUnit(probabilistic_circuit=result)
            for _ in range(self.number_of_nodes)
        ]

        child_layer_rustworkx = [
            child.to_rustworkx(variables, result, progress_bar)
            for child in self.child_layers
        ]

        for child_layer_index, child_layer in enumerate(child_layer_rustworkx):
            for node_index in range(self.number_of_nodes):
                child_node_index = self.edges[child_layer_index, node_index]
                units[node_index].add_subcircuit(child_layer[child_node_index])
                if progress_bar:
                    progress_bar.update()

        return units

