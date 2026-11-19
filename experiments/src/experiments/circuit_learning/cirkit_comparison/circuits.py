"""
Cirkit circuits that compute what a layered circuit computes, one cirkit layer per
layer, so that the benchmark trains and queries RAT-SPNs with cirkit
:cite:`loconte2024what` as a method of its own.
"""

from __future__ import annotations

# %% imports
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
import torch
from cirkit.backend.torch.circuits import TorchCircuit
from cirkit.backend.torch.compiler import TorchCompiler
from cirkit.symbolic.circuit import Circuit
from cirkit.symbolic.initializers import ConstantTensorInitializer
from cirkit.symbolic.layers import CategoricalLayer, HadamardLayer, KroneckerLayer
from cirkit.symbolic.layers import GaussianLayer as CirkitGaussianLayer
from cirkit.symbolic.layers import Layer as CirkitLayer
from cirkit.symbolic.layers import SumLayer as CirkitSumLayer
from cirkit.symbolic.parameters import (
    ExpParameter,
    LogSoftmaxParameter,
    Parameter,
    SoftmaxParameter,
    TensorParameter,
)
from cirkit.utils.scope import Scope
from krrood.patterns.classproperty import classproperty
from scipy.special import log_softmax

from experiments.circuit_learning.cirkit_comparison.exceptions import (
    LayerWithoutCirkitCounterpartError,
)
from experiments.circuit_learning.measurements import QueryMeasurement
from experiments.circuit_learning.rows import (
    ExactQueriesResult,
    LikelihoodSpeedResult,
    Query,
)
from experiments.experiment_definitions import ExperimentResult
from probabilistic_model.learning.training_schedule import PlateauScheduledTraining
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import (
    InnerLayer,
    Layer,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.edge_matrix import (
    SparseEntries,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.product_layer import (
    ProductLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.discrete_layer import (
    SymbolicLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.gaussian_layer import (
    GaussianLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.probability_table import (
    DenseProbabilityTable,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)

# %% converted layers


@dataclass(kw_only=True)
class ConvertedLayer(ABC):
    """
    A layer of a layered circuit and the symbolic cirkit layer that computes the same
    values of its units.
    """

    layer: Layer
    """
    The layer of the layered circuit, with host parameters.
    """

    @property
    @abstractmethod
    def cirkit_layer(self) -> CirkitLayer:
        """
        :return: The symbolic cirkit layer.
        """

    @property
    def tensors(self) -> list[TensorParameter]:
        """
        :return: The learnable cirkit tensors that :attr:`layer`'s parameters are
            computed from, none for a layer without parameters.
        """
        return []

    def layer_parameters(self, *, values: list[np.ndarray]) -> list[np.ndarray]:
        """
        :param values: The values of :attr:`tensors`.
        :return: The parameters of :attr:`layer` that these values stand for, in the
            order of its parameters.
        """
        return self.layer.parameters

    @staticmethod
    def constant_tensor(*, values: np.ndarray) -> TensorParameter:
        """
        :param values: The initial values.
        :return: A learnable cirkit tensor of their shape, initialized with them.
        """
        return TensorParameter(
            *values.shape, initializer=ConstantTensorInitializer(np.asarray(values))
        )


@dataclass(kw_only=True)
class ConvertedGaussianLayer(ConvertedLayer):
    """
    Gaussian input units, whose standard deviation cirkit learns as its logarithm.
    """

    layer: GaussianLayer

    mean: TensorParameter = field(init=False)
    """
    The means of the units.
    """

    log_standard_deviation: TensorParameter = field(init=False)
    """
    The logarithms of the standard deviations of the units.
    """

    gaussian_layer: CirkitGaussianLayer = field(init=False)
    """
    The cirkit Gaussian layer.
    """

    def __post_init__(self) -> None:
        """
        Create the tensors and the cirkit layer from the location and scale of
        :attr:`layer`.
        """
        self.mean = self.constant_tensor(values=self.layer.location)
        self.log_standard_deviation = self.constant_tensor(
            values=np.log(self.layer.scale)
        )
        shape = (self.layer.number_of_nodes,)
        self.gaussian_layer = CirkitGaussianLayer(
            Scope([self.layer.variable]),
            self.layer.number_of_nodes,
            mean=Parameter.from_input(self.mean),
            stddev=Parameter.from_unary(
                ExpParameter(shape), self.log_standard_deviation
            ),
        )

    @property
    def cirkit_layer(self) -> CirkitLayer:
        return self.gaussian_layer

    @property
    def tensors(self) -> list[TensorParameter]:
        return [self.mean, self.log_standard_deviation]

    def layer_parameters(self, *, values: list[np.ndarray]) -> list[np.ndarray]:
        mean, log_standard_deviation = values
        return [mean, np.exp(log_standard_deviation)]


@dataclass(kw_only=True)
class ConvertedSymbolicLayer(ConvertedLayer):
    """
    Categorical input units over the positions of a symbolic variable's domain.
    """

    layer: SymbolicLayer

    logits: TensorParameter = field(init=False)
    """
    The unnormalized log-probabilities of the states of every unit.
    """

    categorical_layer: CategoricalLayer = field(init=False)
    """
    The cirkit categorical layer.
    """

    def __post_init__(self) -> None:
        """
        Create the tensor and the cirkit layer from the probability table of
        :attr:`layer`.

        :raises LayerWithoutCirkitCounterpartError: If the table is not dense or the
            states are not every position of the domain in order.
        """
        table = self.layer.table
        number_of_states = len(self.layer.states)
        if not isinstance(table, DenseProbabilityTable) or not np.array_equal(
            self.layer.states, np.arange(number_of_states)
        ):
            raise LayerWithoutCirkitCounterpartError(layer=self.layer)
        self.logits = self.constant_tensor(values=table.log_probabilities)
        self.categorical_layer = CategoricalLayer(
            Scope([self.layer.variable]),
            self.layer.number_of_nodes,
            num_categories=number_of_states,
            logits=Parameter.from_unary(
                LogSoftmaxParameter(self.logits.shape), self.logits
            ),
        )

    @property
    def cirkit_layer(self) -> CirkitLayer:
        return self.categorical_layer

    @property
    def tensors(self) -> list[TensorParameter]:
        return [self.logits]

    def layer_parameters(self, *, values: list[np.ndarray]) -> list[np.ndarray]:
        (logits,) = values
        return [log_softmax(logits, axis=-1)]


@dataclass(kw_only=True)
class ConvertedInnerLayer(ConvertedLayer, ABC):
    """
    A sum or product layer, whose child layers all have the same number of units, as the
    inputs of a cirkit layer have.
    """

    layer: InnerLayer

    @property
    def units_per_child_layer(self) -> int:
        """
        :return: The number of units of every child layer.
        :raises LayerWithoutCirkitCounterpartError: If the child layers have different
            numbers of units.
        """
        numbers = {child.number_of_nodes for child in self.layer.child_layers}
        if len(numbers) != 1:
            raise LayerWithoutCirkitCounterpartError(layer=self.layer)
        return numbers.pop()


@dataclass(kw_only=True)
class ConvertedSumLayer(ConvertedInnerLayer):
    """
    Sum units that each mix every unit of every child layer.
    """

    layer: SumLayer

    logits: TensorParameter = field(init=False)
    """
    The unnormalized log-weights, one row per unit and one column per unit of the child
    layers, the child layers one after the other.
    """

    sum_layer: CirkitSumLayer = field(init=False)
    """
    The cirkit sum layer.
    """

    def __post_init__(self) -> None:
        """
        Create the tensor and the cirkit layer from the weights of :attr:`layer`.

        :raises LayerWithoutCirkitCounterpartError: If a unit does not mix every unit of
            the child layers.
        """
        units = self.units_per_child_layer
        arity = len(self.layer.child_layers)
        shape = (self.layer.number_of_nodes, units * arity)
        if self.layer.edges.number_of_edges != shape[0] * shape[1]:
            raise LayerWithoutCirkitCounterpartError(layer=self.layer)
        logits = np.full(shape, -np.inf)
        logits[self.layer.edges.rows, self.layer.edges.columns] = np.asarray(
            self.layer.edge_log_weights
        )
        self.logits = self.constant_tensor(values=logits)
        self.sum_layer = CirkitSumLayer(
            units,
            self.layer.number_of_nodes,
            arity,
            weight=Parameter.from_unary(SoftmaxParameter(shape), self.logits),
        )

    @property
    def cirkit_layer(self) -> CirkitLayer:
        return self.sum_layer

    @property
    def tensors(self) -> list[TensorParameter]:
        return [self.logits]

    def layer_parameters(self, *, values: list[np.ndarray]) -> list[np.ndarray]:
        (logits,) = values
        rows, columns = self.layer.edges.rows, self.layer.edges.columns
        log_weights = log_softmax(logits, axis=-1)[rows, columns]
        return [
            self.layer.edges.stored_values(
                SparseEntries(log_weights, rows, columns), -np.inf
            )
        ]


@dataclass(kw_only=True)
class ConvertedProductLayer(ConvertedInnerLayer):
    """
    Product units of the units at the same position in every child layer, a Hadamard
    product, or of every combination of units, a Kronecker product.
    """

    layer: ProductLayer

    product_layer: HadamardLayer | KroneckerLayer = field(init=False)
    """
    The cirkit product layer.
    """

    def __post_init__(self) -> None:
        """
        Create the cirkit layer of the product :attr:`layer` computes.

        :raises LayerWithoutCirkitCounterpartError: If it is neither product.
        """
        units = self.units_per_child_layer
        arity = len(self.layer.child_layers)
        children = self.layer.child_layers
        if self.has_edges_of(product=ProductLayer.node_wise_product_of(children)):
            self.product_layer = HadamardLayer(units, arity)
            return
        if self.has_edges_of(product=ProductLayer.cross_product(child_layers=children)):
            self.product_layer = KroneckerLayer(units, arity)
            return
        raise LayerWithoutCirkitCounterpartError(layer=self.layer)

    def has_edges_of(self, *, product: ProductLayer) -> bool:
        """
        :param product: A product layer over the child layers of :attr:`layer`.
        :return: Whether :attr:`layer` has exactly its units and edges.
        """
        if product.number_of_nodes != self.layer.number_of_nodes:
            return False
        return np.array_equal(
            self.sorted_edges(layer=product), self.sorted_edges(layer=self.layer)
        )

    @staticmethod
    def sorted_edges(*, layer: ProductLayer) -> np.ndarray:
        """
        :param layer: A product layer.
        :return: Its edges as pairs of unit and column, sorted.
        """
        edges = np.stack([layer.edges.rows, layer.edges.columns], axis=1)
        return edges[np.lexsort((edges[:, 1], edges[:, 0]))]

    @property
    def cirkit_layer(self) -> CirkitLayer:
        return self.product_layer


# %% circuits


@dataclass(kw_only=True)
class CirkitCircuit:
    """
    A compiled cirkit circuit that computes the log-likelihood a layered circuit
    computes, folded and optimized as cirkit compiles by default.
    """

    layered: LayeredProbabilisticCircuit
    """
    The layered circuit, with host parameters.
    """

    device: torch.device
    """
    The device the compiled circuit computes on.
    """

    dtype: torch.dtype
    """
    The floating point type of the compiled circuit.
    """

    converted_layers: dict[int, ConvertedLayer] = field(init=False, repr=False)
    """
    The converted layer of every layer of :attr:`layered`, by the identity of the layer.
    """

    compiler: TorchCompiler = field(init=False, repr=False)
    """
    The compiler that compiled the circuit and knows where each tensor ended up.
    """

    compiled: TorchCircuit = field(init=False, repr=False)
    """
    The compiled circuit.
    """

    def __post_init__(self) -> None:
        """
        Convert every layer, children first, and compile the cirkit circuit.
        """
        self.converted_layers = {}
        for layer in reversed(self.layered.layers):
            self.converted_layers[id(layer)] = self.converted_layer(layer=layer)
        cirkit_layers = [
            converted.cirkit_layer for converted in self.converted_layers.values()
        ]
        in_layers = {
            converted.cirkit_layer: [
                self.converted_layers[id(child)].cirkit_layer
                for child in converted.layer.child_layers
            ]
            for converted in self.converted_layers.values()
            if isinstance(converted, ConvertedInnerLayer)
        }
        symbolic = Circuit(
            cirkit_layers,
            in_layers,
            [self.converted_layers[id(self.layered.root)].cirkit_layer],
        )
        self.compiler = TorchCompiler(semiring="lse-sum", fold=True, optimize=True)
        self.compiled = self.compiler.compile(symbolic).to(
            device=self.device, dtype=self.dtype
        )

    @staticmethod
    def converted_layer(*, layer: Layer) -> ConvertedLayer:
        """
        :param layer: A layer of a layered circuit.
        :return: The layer and the cirkit layer that computes the same.
        :raises LayerWithoutCirkitCounterpartError: If no cirkit layer does.
        """
        if isinstance(layer, GaussianLayer):
            return ConvertedGaussianLayer(layer=layer)
        if isinstance(layer, SymbolicLayer):
            return ConvertedSymbolicLayer(layer=layer)
        if isinstance(layer, SumLayer):
            return ConvertedSumLayer(layer=layer)
        if isinstance(layer, ProductLayer):
            return ConvertedProductLayer(layer=layer)
        raise LayerWithoutCirkitCounterpartError(layer=layer)

    def log_likelihood(self, *, rows: torch.Tensor) -> torch.Tensor:
        """
        :param rows: Rows whose symbolic values are positions in the domain, the
            columns in the order of the variables of :attr:`layered`.
        :return: The log-likelihood of every row.
        """
        return self.compiled(rows.to(device=self.device, dtype=self.dtype))[:, 0, 0]

    def tensor_values(self, *, tensor: TensorParameter) -> np.ndarray:
        """
        :param tensor: A tensor of a converted layer.
        :return: Its current values in the compiled circuit.
        """
        compiled, fold = self.compiler.state.retrieve_compiled_parameter(tensor)
        return compiled().detach()[fold].cpu().numpy().astype(np.float64)

    def trained_layered(self) -> LayeredProbabilisticCircuit:
        """
        :return: A copy of :attr:`layered` with the current parameters of the compiled
            circuit, on the host.
        """
        parameters = []
        for layer in self.layered.layers:
            converted = self.converted_layers[id(layer)]
            parameters.extend(
                converted.layer_parameters(
                    values=[
                        self.tensor_values(tensor=tensor)
                        for tensor in converted.tensors
                    ]
                )
            )
        return self.layered.copy_with_parameters(parameters)


# %% training


@dataclass(kw_only=True)
class CirkitTraining(PlateauScheduledTraining):
    """
    Mini-batch training of a cirkit circuit as cirkit trains its circuits: Adam on the
    average negative log-likelihood, one step per batch.
    """

    circuit: CirkitCircuit
    """
    The circuit whose parameters are trained.
    """

    learning_rate: float
    """
    The initial learning rate of Adam.
    """

    optimizer: torch.optim.Optimizer = field(init=False, repr=False)
    """
    The Adam optimizer of the parameters of the compiled circuit.
    """

    def __post_init__(self) -> None:
        """
        Create :attr:`optimizer`.
        """
        self.optimizer = torch.optim.Adam(
            self.circuit.compiled.parameters(), lr=self.learning_rate
        )

    def step_on_batch(self, *, batch: tuple[torch.Tensor, ...]) -> float:
        (rows,) = batch
        self.optimizer.zero_grad()
        loss = -self.circuit.log_likelihood(rows=rows).mean()
        loss.backward()
        self.optimizer.step()
        return float(loss.detach())

    def scale_learning_rate(self, *, scale: float) -> None:
        for group in self.optimizer.param_groups:
            group["lr"] = self.learning_rate * scale


# %% queries


@dataclass(kw_only=True)
class CirkitQueryMeasurement(QueryMeasurement):
    """
    Measures the queries of a stored circuit in cirkit: the likelihood.

    cirkit 0.3.1 samples no circuit with Kronecker products, such as a RAT-SPN, and
    conditions and truncates none. Overrides :meth:`QueryMeasurement.run`.
    """

    reports_exact_queries: bool = False
    """
    Whether the run lists the queries cirkit answers exactly, which one run per circuit
    does.
    """

    @classproperty
    def queries_answered_exactly(cls) -> list[Query]:
        """
        :return: The queries cirkit answers exactly: marginals by integrating
            variables out, and conditionals on observed values as their ratio.
        """
        return [Query.MARGINAL, Query.CONDITIONAL]

    def run(self) -> list[ExperimentResult]:
        circuit = CirkitCircuit(
            layered=self.circuit,
            device=self.setting.device_object,
            dtype=self.setting.dtype,
        )
        rows: list[ExperimentResult] = [self.cirkit_likelihood_speed(circuit=circuit)]
        if self.reports_exact_queries:
            rows.append(self.exact_queries())
        return rows

    def exact_queries(self) -> ExactQueriesResult:
        return ExactQueriesResult(
            configuration=self.configuration,
            exact_queries=self.queries_answered_exactly,
        )

    def cirkit_likelihood_speed(
        self, *, circuit: CirkitCircuit
    ) -> LikelihoodSpeedResult:
        """
        :param circuit: The compiled circuit.
        :return: How fast it computes the likelihood of test rows.
        """
        chunk_rows = min(self.likelihood_rows, len(self.dataset.test))
        rows = torch.asarray(
            self.circuit.encoded(self.dataset.encoded(frame=self.dataset.test))[
                :chunk_rows
            ],
            dtype=circuit.dtype,
            device=circuit.device,
        )

        def likelihoods() -> torch.Tensor:
            """
            :return: The log-likelihoods of the rows, on the host.
            """
            with torch.no_grad():
                return circuit.log_likelihood(rows=rows).cpu()

        timing = self.timed_calls(query=likelihoods)
        return LikelihoodSpeedResult(
            configuration=self.configuration,
            first_call_duration=timing.first_call,
            rows_per_second=chunk_rows / timing.later_call.total_seconds(),
        )
