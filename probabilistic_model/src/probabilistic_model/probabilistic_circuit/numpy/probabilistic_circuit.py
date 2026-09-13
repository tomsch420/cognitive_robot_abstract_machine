from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np
from krrood.adapters.json_serializer import SubclassJSONSerializer, to_json, from_json
from random_events.variable import Variable
from sortedcontainers import SortedSet
from typing_extensions import Tuple, Self, List, Optional, Dict, Any
import tqdm

from probabilistic_model.probabilistic_circuit.numpy.layer import (
    Layer,
    RustworkxLayerConverter,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit as RustworkxProbabilisticCircuit,
)
from probabilistic_model.probabilistic_model import (
    ProbabilisticModel,
    OrderType,
    CenterType,
    MomentType,
)
from random_events.product_algebra import Event, SimpleEvent


@dataclass
class ProbabilisticCircuit(SubclassJSONSerializer, ProbabilisticModel):
    """
    A probabilistic circuit as wrapper for a layered probabilistic model using NumPy.
    """

    circuit_variables: SortedSet
    """
    The variables of the circuit.
    """

    root: Layer
    """
    The root layer of the circuit.
    """

    @property
    def variables(self) -> Tuple[Variable, ...]:
        return tuple(self.circuit_variables)

    @property
    def support(self) -> Event:
        return self.root.support(self.variables)[0]

    def log_likelihood(self, x: np.ndarray) -> np.ndarray:
        return self.root.log_likelihood_of_nodes(x)[:, 0]

    def probability_of_simple_event(self, event: SimpleEvent) -> float:
        return self.root.probability(event.as_composite_set(), self.variables)[0]

    def moment(self, order: OrderType, center: CenterType) -> MomentType:
        variable_to_index_map = {var: i for i, var in enumerate(self.circuit_variables)}
        moments = self.root.moment(order, center, variable_to_index_map)
        root_moments = moments[0]
        return MomentType(
            {var: root_moments[i] for var, i in variable_to_index_map.items()}
        )

    def sample(self, amount: int) -> np.ndarray:
        indices = np.zeros(amount, dtype=int)
        return self.root.sample(indices, self.variables)

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        return self.root.cumulative_distribution_function(x)[:, 0]

    def log_mode(self) -> Tuple[Event, float]:
        return self.root.log_mode(self.variables)[0]

    def marginal(self, variables: Iterable[Variable]) -> Optional[ProbabilisticCircuit]:
        new_root = self.root.marginal(variables, self.variables)
        if new_root is None:
            return None
        new_vars = SortedSet([v for v in self.circuit_variables if v in variables])
        return ProbabilisticCircuit(new_vars, new_root)

    def log_truncated(
        self, event: Event, singleton_allowed: bool = False
    ) -> Tuple[Optional[ProbabilisticCircuit], float]:
        new_root, log_probs = self.root.log_truncated(event, self.variables)
        if new_root is None or log_probs[0] == -np.inf:
            return None, -np.inf
        return ProbabilisticCircuit(self.circuit_variables, new_root), log_probs[0]

    def log_conditional(
        self, point: Dict[Variable, Any]
    ) -> Tuple[Optional[ProbabilisticCircuit], float]:
        event = SimpleEvent.from_data(point).as_composite_set()
        return self.log_truncated(event, singleton_allowed=True)

    @classmethod
    def from_rustworkx(
        cls, pc: RustworkxProbabilisticCircuit, progress_bar: bool = False
    ) -> ProbabilisticCircuit:
        """
        Convert a probabilistic circuit to a layered circuit.

        The result expresses the same distribution as `pc`.

        :param pc: The probabilistic circuit.
        :param progress_bar: Whether to show a progress bar.
        :return: The layered circuit.
        """
        # group nodes by depth
        layer_to_nodes_map = {index: layer for index, layer in enumerate(pc.layers)}
        reversed_layers_to_nodes_map = dict(reversed(layer_to_nodes_map.items()))

        # create layers from nodes
        child_layers: List[RustworkxLayerConverter] = []
        for layer_index, nodes in (
            tqdm.tqdm(reversed_layers_to_nodes_map.items(), desc="Creating Layers")
            if progress_bar
            else reversed_layers_to_nodes_map.items()
        ):
            child_layers = Layer.create_layers_from_nodes(
                nodes, child_layers, progress_bar
            )
        root = child_layers[0].layer

        return cls(pc.variables, root)

    def to_rustworkx(self, progress_bar: bool = True) -> RustworkxProbabilisticCircuit:
        """
        Convert the probabilistic circuit to a rustworkx graph.

        :param progress_bar: Whether to show a progress bar.
        :return: The rustworkx graph.
        """
        if progress_bar:
            number_of_edges = self.root.number_of_components
            progress_bar = tqdm.tqdm(total=number_of_edges, desc="Converting to rx")
        else:
            progress_bar = None
        result = RustworkxProbabilisticCircuit()
        self.root.to_rustworkx(self.variables, result, progress_bar)
        return result

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["variables"] = [to_json(variable) for variable in self.circuit_variables]
        result["root"] = self.root.to_json()
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        variables = SortedSet(
            from_json(variable, **kwargs) for variable in data["variables"]
        )
        root = Layer.from_json(data["root"], **kwargs)
        return cls(variables, root)
