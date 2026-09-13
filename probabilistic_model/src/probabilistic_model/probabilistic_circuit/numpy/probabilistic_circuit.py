from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np
from krrood.adapters.json_serializer import (
    SubclassJSONSerializer,
    to_json,
    from_json,
    DataclassJSONSerializer,
)
from random_events.variable import Variable
from sortedcontainers import SortedSet
from typing_extensions import Tuple, Self, List, Optional, Dict, Any
import tqdm

from probabilistic_model.probabilistic_circuit.numpy.layer import (
    Layer,
)
from probabilistic_model.probabilistic_circuit.numpy.conversion import (
    RustworkxLayerConverter,
    from_rustworkx,
    to_rustworkx,
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
class LayeredProbabilisticCircuit(SubclassJSONSerializer, ProbabilisticModel):
    """
    A probabilistic circuit as wrapper for a layered probabilistic model using NumPy.
    """

    _variables: SortedSet
    """
    The variables of the circuit.
    """

    root: Layer
    """
    The root layer of the circuit.
    """

    def __post_init__(self):
        pass

    @property
    def variables(self) -> Tuple[Variable, ...]:
        return tuple(self._variables)

    @property
    def circuit_variables(self) -> SortedSet:
        """
        Alias for variables to maintain compatibility if needed, though 'variables' is preferred.
        """
        return self._variables

    @property
    def support(self) -> Event:
        return self.root.support(tuple(self.variables))[0]

    def log_likelihood(self, x: np.ndarray) -> np.ndarray:
        return self.root.log_likelihood_of_nodes(x)[:, 0]

    def probability_of_simple_event(self, event: SimpleEvent) -> float:
        return self.root.probability(event.as_composite_set(), tuple(self.variables))[0]

    def moment(self, order: OrderType, center: CenterType) -> MomentType:
        variable_to_index_map = {var: i for i, var in enumerate(self.variables)}
        moments = self.root.moment(order, center, variable_to_index_map)
        root_moments = moments[0]
        return MomentType(
            {var: root_moments[i] for var, i in variable_to_index_map.items()}
        )

    def sample(self, amount: int) -> np.ndarray:
        indices = np.zeros(amount, dtype=int)
        return self.root.sample(indices, tuple(self.variables))

    def cumulative_distribution_function(self, x: np.ndarray) -> np.ndarray:
        return self.root.cumulative_distribution_function(x)[:, 0]

    def log_mode(self) -> Tuple[Event, float]:
        return self.root.log_mode(tuple(self.variables))[0]

    def marginal(self, variables: Iterable[Variable]) -> Optional[LayeredProbabilisticCircuit]:
        new_root = self.root.marginal(variables, tuple(self.variables))
        if new_root is None:
            return None
        new_vars = SortedSet([v for v in self.variables if v in variables])
        return LayeredProbabilisticCircuit(new_vars, new_root)

    def log_truncated(
        self, event: Event, singleton_allowed: bool = False
    ) -> Tuple[Optional[LayeredProbabilisticCircuit], float]:
        new_root, log_probs = self.root.log_truncated(event, tuple(self.variables))
        if new_root is None or log_probs[0] == -np.inf:
            return None, -np.inf
        return LayeredProbabilisticCircuit(self.variables, new_root), log_probs[0]

    def log_conditional(
        self, point: Dict[Variable, Any]
    ) -> Tuple[Optional[LayeredProbabilisticCircuit], float]:
        event = SimpleEvent.from_data(point).as_composite_set()
        return self.log_truncated(event, singleton_allowed=True)

    @classmethod
    def from_rustworkx(
        cls, pc: RustworkxProbabilisticCircuit, progress_bar: bool = False
    ) -> LayeredProbabilisticCircuit:
        """
        Convert a probabilistic circuit to a layered circuit.

        The result expresses the same distribution as `pc`.

        :param pc: The probabilistic circuit.
        :param progress_bar: Whether to show a progress bar.
        :return: The layered circuit.
        """
        return from_rustworkx(pc, progress_bar)

    def to_rustworkx(self, progress_bar: bool = True) -> RustworkxProbabilisticCircuit:
        """
        Convert the probabilistic circuit to a rustworkx graph.

        :param progress_bar: Whether to show a progress bar.
        :return: The rustworkx graph.
        """
        return to_rustworkx(self, progress_bar)

    def to_json(self, **kwargs) -> Dict[str, Any]:
        return DataclassJSONSerializer.to_json(self, **kwargs)

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        return DataclassJSONSerializer.from_json(data, cls, **kwargs)

