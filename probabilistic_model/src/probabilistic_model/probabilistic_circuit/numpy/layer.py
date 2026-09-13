from __future__ import annotations

import inspect
from abc import abstractmethod, ABC
from dataclasses import dataclass
import numpy as np
import scipy.sparse
from krrood.adapters.json_serializer import SubclassJSONSerializer, recursive_subclasses
from typing_extensions import (
    List,
    Tuple,
    Optional,
    Any,
    Self,
    Dict,
    Union,
    Type,
    Iterator,
)
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


def inverse_class_of(clazz: Type[Unit]) -> Type[Layer]:
    """
    Get the layered circuit layer class for a rustworkx unit class.
    """
    for subclass in recursive_subclasses(Layer):
        if not inspect.isabstract(subclass):
            if issubclass(clazz, subclass.rustworkx_classes()):
                return subclass

    raise TypeError(f"Could not find class for {clazz}")


@dataclass
class RustworkxLayerConverter:
    """
    Class used for conversion from a probabilistic circuit in rustworkx to a layered
    circuit in numpy.
    """

    layer: Layer
    nodes: List[Unit]
    hash_remap: Dict[int, int]


class Layer(SubclassJSONSerializer, ABC):
    """
    Abstract class for Layers of a layered circuit.

    Layers have the same scope (set of variables) for every node in them.
    """

    _variables: Optional[np.ndarray] = None
    """
    The variable indices of the layer.
    """

    @property
    def variables(self) -> np.ndarray:
        return self._variables

    @variables.setter
    def variables(self, value: np.ndarray):
        self._variables = value

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
    @abstractmethod
    def rustworkx_classes(cls) -> Tuple[Type[Unit], ...]:
        """
        :return: The rustworkx classes that this layer represents.
        """

    @staticmethod
    def create_layers_from_nodes(
        nodes: List[Unit],
        child_layers: List[RustworkxLayerConverter],
        progress_bar: bool = True,
    ) -> List[RustworkxLayerConverter]:
        """
        Create a layer from a list of nodes.
        """
        result = []

        unique_types = set(
            type(node) if not node.is_leaf else type(node.distribution)
            for node in nodes
        )
        for unique_type in unique_types:
            nodes_of_current_type = [
                node
                for node in nodes
                if (
                    isinstance(node, unique_type)
                    if not node.is_leaf
                    else isinstance(node.distribution, unique_type)
                )
            ]

            if nodes[0].is_leaf:
                unique_type = type(nodes_of_current_type[0].distribution)

            layer_type = inverse_class_of(unique_type)

            scopes = [tuple(node.variables) for node in nodes_of_current_type]
            unique_scopes = set(scopes)
            for scope in unique_scopes:
                nodes_of_current_type_and_scope = [
                    node
                    for node in nodes_of_current_type
                    if tuple(node.variables) == scope
                ]

                layer = layer_type.create_layer_from_nodes_with_same_type_and_scope(
                    nodes_of_current_type_and_scope, child_layers, progress_bar
                )
                result.append(layer)

        return result

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


class InnerLayer(Layer, ABC):
    """
    Abstract Base Class for inner layers.
    """

    child_layers: List[Layer]
    """
    The child layers of this layer.
    """

    def __init__(self, child_layers: List[Layer]):
        super().__init__()
        self.child_layers = child_layers

    def all_layers(self) -> List[Layer]:
        result = [self]
        for child_layer in self.child_layers:
            result.extend(child_layer.all_layers())
        return result

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["child_layers"] = [
            child_layer.to_json() for child_layer in self.child_layers
        ]
        return result


class InputLayer(Layer, ABC):
    """
    Abstract base class for univariate input units.
    """

    def __init__(self, variable: int):
        super().__init__()
        self._variables = np.array([variable])

    @property
    def variable(self) -> int:
        return self._variables[0]

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["variable"] = int(self.variable)
        return result

    @property
    def number_of_nodes(self) -> int:
        return 1


class SumLayer(InnerLayer, ABC):
    """
    Abstract base class for sum layers.
    """

    def __init__(self, child_layers: List[Layer]):
        super().__init__(child_layers)

    @property
    def variables(self) -> np.ndarray:
        if self._variables is None:
            self._variables = self.child_layers[0].variables
        return self._variables

    @classmethod
    def rustworkx_classes(cls) -> Tuple[Type[Unit], ...]:
        return (SumUnit,)


class SparseSumLayer(SumLayer):
    """
    A SumLayer that uses SciPy Sparse matrices for weights.
    """

    weights: List[scipy.sparse.csr_matrix]
    """
    The weights of the sum units for each child layer.

    Each element is a sparse matrix of shape (number_of_nodes,
    child_layer.number_of_nodes).
    """

    def __init__(
        self, child_layers: List[Layer], weights: List[scipy.sparse.csr_matrix]
    ):
        super().__init__(child_layers)
        self.weights = weights

    @property
    def number_of_nodes(self) -> int:
        return self.weights[0].shape[0]

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        child_lls = [child.log_likelihood_of_nodes(x) for child in self.child_layers]

        # Stability trick: subtract max
        m = np.max([np.max(cll, axis=1) for cll in child_lls], axis=0)

        # mask for samples where all children have -inf log likelihood
        inf_mask = m == -np.inf

        total_prob = np.zeros((x.shape[0], self.number_of_nodes))

        # for samples with -inf, we don't care about the exp value as long as it's not nan
        # we can just use 0 as m for those samples
        safe_m = np.where(inf_mask, 0.0, m)

        for cll, w in zip(child_lls, self.weights):
            prob = np.exp(cll - safe_m[:, np.newaxis])
            total_prob += prob @ w.T

        result = np.log(total_prob) + safe_m[:, np.newaxis]
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
        num_vars = len(variable_to_index_map)
        result = np.zeros((self.number_of_nodes, num_vars))
        for cm, w in zip(child_moments, self.weights):
            result += w @ cm
        return result

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        num_samples = len(indices)
        num_vars = len(variables)

        # Concatenate weights from all child layers
        concatenated_weights = scipy.sparse.hstack(self.weights).tocsr()

        unique_indices, counts = np.unique(indices, return_counts=True)
        new_child_indices = np.empty(num_samples, dtype=int)

        for idx, count in zip(unique_indices, counts):
            mask = indices == idx
            row = concatenated_weights[idx, :]
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

        has_symbolic = any(not isinstance(v, (Continuous, Integer)) for v in variables)
        dtype = object if has_symbolic else float

        result = np.zeros((num_samples, num_vars), dtype=dtype)
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
            for layer_idx, w in enumerate(self.weights):
                row = w[i, :]
                for child_idx in row.indices:
                    s = child_supports[layer_idx][child_idx]
                    if node_support is None:
                        node_support = s.__deepcopy__()
                    else:
                        node_support |= s
            result.append(node_support)
        return result

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        child_cdfs = [
            child.cumulative_distribution_function(x) for child in self.child_layers
        ]
        result = np.zeros((x.shape[0], self.number_of_nodes))
        for ccdf, w in zip(child_cdfs, self.weights):
            result += ccdf @ w.T
        return result

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        child_log_modes = [child.log_mode(variables) for child in self.child_layers]
        result = []
        for i in range(self.number_of_nodes):
            best_log_likelihood = -np.inf
            best_event = None

            for layer_idx, w in enumerate(self.weights):
                row = w[i, :]
                for child_idx in row.indices:
                    weight = w[i, child_idx]
                    event, ll = child_log_modes[layer_idx][child_idx]
                    combined_ll = np.log(weight) + ll
                    if combined_ll > best_log_likelihood:
                        best_log_likelihood = combined_ll
                        best_event = event
                    elif combined_ll == best_log_likelihood and best_event is not None:
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
        child_log_probs = []
        for child, log_probs in truncated_children_results:
            new_child_layers.append(child)
            child_log_probs.append(log_probs)

        # Update weights: new_weight = old_weight * exp(child_log_prob)
        new_weights = []
        overall_log_probs = np.full(self.number_of_nodes, -np.inf)

        for w, clp in zip(self.weights, child_log_probs):
            # w is (num_nodes, child_nodes)
            # clp is (child_nodes,)
            # w_new = w * diag(exp(clp))
            w_new = w.multiply(np.exp(clp))
            new_weights.append(w_new)

            # Update overall log probabilities
            # row_sums = sum_j old_weight_j * exp(child_log_prob_j)
            row_sums = np.array(w_new.sum(axis=1)).flatten()
            row_log_sums = np.full_like(row_sums, -np.inf)
            mask = row_sums > 0
            row_log_sums[mask] = np.log(row_sums[mask])
            overall_log_probs = np.logaddexp(overall_log_probs, row_log_sums)

        # Normalize new weights
        final_weights = []
        safe_sums = np.exp(overall_log_probs)
        safe_sums[safe_sums == 0] = 1.0
        inv_sums = 1.0 / safe_sums
        for w_new in new_weights:
            from scipy.sparse import diags

            w_final = diags(inv_sums) @ w_new
            final_weights.append(w_final)

        return SparseSumLayer(new_child_layers, final_weights), overall_log_probs

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        child_probs = [
            child.probability(event, variables) for child in self.child_layers
        ]
        result = np.zeros(self.number_of_nodes)
        for cp, w in zip(child_probs, self.weights):
            result += w @ cp
        return result

    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        new_child_layers = []
        new_weights = []
        for child, w in zip(self.child_layers, self.weights):
            m = child.marginal(variables, all_variables)
            if m is not None:
                new_child_layers.append(m)
                new_weights.append(w)

        if not new_child_layers:
            return None

        return SparseSumLayer(new_child_layers, new_weights)

    @property
    def number_of_components(self) -> int:
        return sum([cl.number_of_components for cl in self.child_layers]) + sum(
            [w.nnz for w in self.weights]
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

        sum_layer = cls([cl.layer for cl in filtered_child_layers], weights)
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
            cl.to_rustworkx(variables, result, progress_bar) for cl in self.child_layers
        ]

        for weight_matrix, child_layer in zip(self.weights, child_layer_rustworkx):
            # extract the weights for the child layer
            coo = weight_matrix.tocoo()
            for row, col, weight in zip(coo.row, coo.col, coo.data):
                units[row].add_subcircuit(child_layer[col], np.log(weight))
                if progress_bar:
                    progress_bar.update()

        return units

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["weights"] = []
        for w in self.weights:
            w_coo = w.tocoo()
            result["weights"].append(
                {
                    "data": w_coo.data.tolist(),
                    "row": w_coo.row.tolist(),
                    "col": w_coo.col.tolist(),
                    "shape": w_coo.shape,
                }
            )
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        child_layers = [Layer.from_json(cl, **kwargs) for cl in data["child_layers"]]
        weights = []
        for w_data in data["weights"]:
            w = scipy.sparse.coo_matrix(
                (w_data["data"], (w_data["row"], w_data["col"])), shape=w_data["shape"]
            ).tocsr()
            weights.append(w)
        return cls(child_layers, weights)


class ProductLayer(InnerLayer):
    """
    A layer that represents the product of multiple other units.
    """

    edges: np.ndarray
    """
    The edges from this layer to child layers.

    Shape: (len(child_layers), number_of_nodes)
    Each entry edges[i, j] is the index of the node in child_layers[i] that node j in this layer uses.
    """

    def __init__(self, child_layers: List[Layer], edges: np.ndarray):
        super().__init__(child_layers)
        self.edges = edges

    @property
    def number_of_nodes(self) -> int:
        return self.edges.shape[1]

    @classmethod
    def rustworkx_classes(cls) -> Tuple[Type[Unit], ...]:
        return (ProductUnit,)

    @property
    def variables(self) -> np.ndarray:
        if self._variables is None:
            variables = np.concatenate([child.variables for child in self.child_layers])
            self._variables = np.unique(variables)
        return self._variables

    def log_likelihood_of_nodes(self, x: np.ndarray) -> np.ndarray:
        N = x.shape[0]
        result = np.zeros((N, self.number_of_nodes))

        for child, edge_indices in zip(self.child_layers, self.edges):
            child_ll = child.log_likelihood_of_nodes(x)
            result += child_ll[:, edge_indices]

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
        num_vars = len(variable_to_index_map)
        result = np.zeros((self.number_of_nodes, num_vars))
        for cm, edge_indices in zip(child_moments, self.edges):
            result += cm[edge_indices, :]
        return result

    def sample(
        self, indices: np.ndarray, variables: Tuple[Variable, ...]
    ) -> np.ndarray:
        num_samples = len(indices)
        num_vars = len(variables)

        from random_events.variable import Continuous, Integer

        has_symbolic = any(not isinstance(v, (Continuous, Integer)) for v in variables)
        dtype = object if has_symbolic else float
        result = np.zeros((num_samples, num_vars), dtype=dtype)

        for child, edge_indices in zip(self.child_layers, self.edges):
            local_indices = edge_indices[indices]
            child_sample = child.sample(local_indices, variables)
            for var_idx in child.variables:
                result[:, var_idx] = child_sample[:, var_idx]
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
        N = x.shape[0]
        result = np.ones((N, self.number_of_nodes))
        for child, edge_indices in zip(self.child_layers, self.edges):
            child_cdf = child.cumulative_distribution_function(x)
            result *= child_cdf[:, edge_indices]
        return result

    def log_mode(self, variables: Tuple[Variable, ...]) -> List[Tuple[Event, float]]:
        child_log_modes = [child.log_mode(variables) for child in self.child_layers]
        result = []
        for j in range(self.number_of_nodes):
            mode_event, mode_ll = child_log_modes[0][self.edges[0, j]]
            for i in range(1, len(self.child_layers)):
                other_event, other_ll = child_log_modes[i][self.edges[i, j]]
                mode_event &= other_event
                mode_ll += other_ll
            result.append((mode_event, mode_ll))
        return result

    def log_truncated(
        self, event: Event, variables: Tuple[Variable, ...]
    ) -> Tuple[Optional[Layer], np.ndarray]:
        truncated_children_results = [
            child.log_truncated(event, variables) for child in self.child_layers
        ]

        new_child_layers = []
        child_log_probs = []
        for child, log_probs in truncated_children_results:
            new_child_layers.append(child)
            child_log_probs.append(log_probs)

        # New ProductLayer
        new_layer = ProductLayer(new_child_layers, self.edges)

        # Overall log probs: sum of child log probs
        overall_log_probs = np.zeros(self.number_of_nodes)
        for i, clp in enumerate(child_log_probs):
            overall_log_probs += clp[self.edges[i, :]]

        return new_layer, overall_log_probs

    def probability(self, event: Event, variables: Tuple[Variable, ...]) -> np.ndarray:
        child_probs = [
            child.probability(event, variables) for child in self.child_layers
        ]
        result = np.ones(self.number_of_nodes)
        for cp, edge_indices in zip(child_probs, self.edges):
            result *= cp[edge_indices]
        return result

    def marginal(
        self, variables: Iterable[Variable], all_variables: Tuple[Variable, ...]
    ) -> Optional[Layer]:
        new_child_layers = []
        new_edge_indices = []
        for i, child in enumerate(self.child_layers):
            m = child.marginal(variables, all_variables)
            if m is not None:
                new_child_layers.append(m)
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
            pbar = tqdm.tqdm(total=number_of_nodes, desc="Assembling Product Layer")
        # for every node in the nodes for this layer
        for node_index, node in enumerate(nodes):

            # for every child layer
            for child_layer_index, child_layer in enumerate(child_layers):
                cl_variables = SortedSet(
                    [
                        node.probabilistic_circuit.variables[index]
                        for index in child_layer.layer.variables
                    ]
                )

                # for every subcircuit
                for subcircuit in node.subcircuits:
                    # if the scopes are compatible
                    if cl_variables == subcircuit.variables:
                        # add the edge
                        edges[child_layer_index, node_index] = child_layer.hash_remap[
                            hash(subcircuit)
                        ]
            if progress_bar:
                pbar.update(1)

        layer = cls([cl.layer for cl in child_layers], edges)
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

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["edges"] = self.edges.tolist()
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        child_layers = [Layer.from_json(cl, **kwargs) for cl in data["child_layers"]]
        edges = np.array(data["edges"])
        return cls(child_layers, edges)
