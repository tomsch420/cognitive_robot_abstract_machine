"""
Strudel structure learning of :cite:`dang2020strudel`: compile a Chow-Liu tree into a
smooth, deterministic and structured-decomposable circuit over symbolic variables, then
split the sum edge most rows flow through on the variable that shares most mutual
information with the others below it, one split at a time.
"""

from __future__ import annotations

# %% imports
import copy
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from random_events.variable import Symbolic
from scipy.sparse.csgraph import shortest_path
from sortedcontainers import SortedSet

from probabilistic_model.adapters.circuit_representations import CircuitRepresentations
from probabilistic_model.distributions.distributions import SymbolicDistribution
from probabilistic_model.learning.layered.exceptions import (
    UnsupportedStructureVariableError,
)
from probabilistic_model.learning.layered.hidden_chow_liu_tree import ChowLiuTree
from probabilistic_model.learning.layered.mutual_information import (
    DiscretizedData,
    PairwiseMutualInformation,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
    ProductUnit,
    SumUnit,
    Unit,
    leaf,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.tensorized.symbolic_encoding import (
    SymbolicEncoding,
)
from probabilistic_model.utils import MissingDict


# %% nodes


@dataclass(eq=False, kw_only=True)
class StrudelNode(ABC):
    """
    A node of a deterministic circuit over symbolic variables, compared by identity.
    """

    @property
    @abstractmethod
    def scope(self) -> frozenset[int]:
        """
        :return: The indices of the variables the node is over.
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def child_nodes(self) -> list[StrudelNode]:
        """
        :return: The children of the node, none for an indicator.
        """
        raise NotImplementedError

    @abstractmethod
    def support_of_rows(self, *, states: np.ndarray, cache: dict) -> np.ndarray:
        """
        :param states: The state index of every value, shape (#rows, #variables).
        :param cache: The supports computed so far in this pass, by node.
        :return: Whether every row lies in the support of the node.
        """
        raise NotImplementedError


@dataclass(eq=False, kw_only=True)
class Indicator(StrudelNode):
    """
    The indicator of one state of one variable.
    """

    variable: int
    """
    The index of the variable.
    """

    state: int
    """
    The index of the state.
    """

    @property
    def scope(self) -> frozenset[int]:
        return frozenset([self.variable])

    @property
    def child_nodes(self) -> list[StrudelNode]:
        return []

    def support_of_rows(self, *, states: np.ndarray, cache: dict) -> np.ndarray:
        if self not in cache:
            cache[self] = states[:, self.variable] == self.state
        return cache[self]


@dataclass(eq=False, kw_only=True)
class StrudelProduct(StrudelNode):
    """
    A product over children with disjoint scopes.
    """

    children: list[StrudelNode]
    """
    The factors.
    """

    @property
    def scope(self) -> frozenset[int]:
        return frozenset().union(*(child.scope for child in self.children))

    @property
    def child_nodes(self) -> list[StrudelNode]:
        return self.children

    def support_of_rows(self, *, states: np.ndarray, cache: dict) -> np.ndarray:
        if self not in cache:
            result = np.ones(len(states), dtype=bool)
            for child in self.children:
                result &= child.support_of_rows(states=states, cache=cache)
            cache[self] = result
        return cache[self]


@dataclass(eq=False, kw_only=True)
class StrudelSum(StrudelNode):
    """
    A mixture of children with the same scope and disjoint supports, whose weights are
    the smoothed fractions of the rows that reach it and go on to each child.
    """

    children: list[StrudelNode]
    """
    The components.
    """

    @property
    def scope(self) -> frozenset[int]:
        return self.children[0].scope

    @property
    def child_nodes(self) -> list[StrudelNode]:
        return self.children

    def support_of_rows(self, *, states: np.ndarray, cache: dict) -> np.ndarray:
        if self not in cache:
            result = np.zeros(len(states), dtype=bool)
            for child in self.children:
                result |= child.support_of_rows(states=states, cache=cache)
            cache[self] = result
        return cache[self]


# %% flows


@dataclass(kw_only=True)
class DeterministicFlows:
    """
    The rows that reach every node and every sum edge of a deterministic circuit, which
    passes every row along at most one child of every sum.
    """

    root: StrudelNode
    """
    The root of the circuit.
    """

    states: np.ndarray
    """
    The state index of every value, shape (#rows, #variables).
    """

    rows_of_nodes: dict[StrudelNode, np.ndarray] = field(init=False)
    """
    Whether every row reaches a node.
    """

    rows_of_edges: dict[tuple[StrudelSum, int], np.ndarray] = field(init=False)
    """
    Whether every row passes along the edge from a sum to its child at a position.
    """

    def __post_init__(self) -> None:
        supports: dict = {}
        self.rows_of_nodes = {}
        self.rows_of_edges = {}
        for node in self.nodes_parents_first():
            reached = self.rows_of_nodes.get(node)
            if reached is None:
                reached = np.ones(len(self.states), dtype=bool)
                self.rows_of_nodes[node] = reached
            if isinstance(node, StrudelSum):
                for position, child in enumerate(node.children):
                    passed = reached & child.support_of_rows(
                        states=self.states, cache=supports
                    )
                    self.rows_of_edges[(node, position)] = passed
                    self.add_rows(node=child, rows=passed)
            elif isinstance(node, StrudelProduct):
                for child in node.children:
                    self.add_rows(node=child, rows=reached)

    def add_rows(self, *, node: StrudelNode, rows: np.ndarray) -> None:
        """
        :param node: A node.
        :param rows: Rows that reach the node from one of its parents.
        """
        if node in self.rows_of_nodes:
            self.rows_of_nodes[node] = self.rows_of_nodes[node] | rows
        else:
            self.rows_of_nodes[node] = rows

    def nodes_parents_first(self) -> list[StrudelNode]:
        """
        :return: Every node of the circuit, each after all of its parents.
        """
        postorder: list[StrudelNode] = []
        visited: set[int] = set()
        stack: list[tuple[StrudelNode, bool]] = [(self.root, False)]
        while stack:
            node, expanded = stack.pop()
            if expanded:
                postorder.append(node)
                continue
            if id(node) in visited:
                continue
            visited.add(id(node))
            stack.append((node, True))
            for child in node.child_nodes:
                if id(child) not in visited:
                    stack.append((child, False))
        return postorder[::-1]


# %% Strudel


@dataclass(kw_only=True)
class Strudel:
    """
    Learns a deterministic, structured-decomposable circuit over symbolic variables by
    greedy splitting, algorithm 2 of the paper with the eFLOW and vMI heuristics.

    A split copies the sub-circuit below the edge once per state of the variable,
    conditioned on it, sharing the nodes whose scope does not hold the variable.
    """

    variables: SortedSet
    """
    The symbolic variables, one per column of the data.
    """

    maximum_splits: int = 1000
    """
    The largest number of splits, the bound of the paper's experiments.
    """

    pseudocount: float = 0.1
    """
    The count added to every edge of a sum when its weights are estimated.
    """

    validation_interval: int = 50
    """
    The number of splits between two checks of the validation log-likelihood.
    """

    def build_circuit(
        self, *, data: np.ndarray, validation_data: np.ndarray | None = None
    ) -> LayeredProbabilisticCircuit:
        """
        :param data: The training rows, whose symbolic values are hashes.
        :param validation_data: Rows whose falling log-likelihood stops the splitting.
        :return: The learned circuit, as a layered circuit with host parameters.
        :raises UnsupportedStructureVariableError: If a variable is not symbolic.
        """
        for variable in self.variables:
            if not isinstance(variable, Symbolic):
                raise UnsupportedStructureVariableError(variable=variable)
        states = self.states_of(data=data)
        root = self.compiled_chow_liu_tree(states=states)
        # splits change nodes in place, so the best circuit so far is kept as a copy
        best_root, best_validation = copy.deepcopy(root), -np.inf
        for split in range(1, self.maximum_splits + 1):
            flows = DeterministicFlows(root=root, states=states)
            candidate = self.best_split(flows=flows, states=states)
            if candidate is None:
                break
            root = self.split(root=root, candidate=candidate)
            if validation_data is None or split % self.validation_interval:
                continue
            validation = float(
                self.to_layered_circuit(root=root, states=states)
                .log_likelihood(events=validation_data)
                .mean()
            )
            if validation < best_validation:
                return self.to_layered_circuit(root=best_root, states=states)
            best_root, best_validation = copy.deepcopy(root), validation
        return self.to_layered_circuit(root=root, states=states)

    def states_of(self, *, data: np.ndarray) -> np.ndarray:
        """
        :param data: Rows whose symbolic values are hashes.
        :return: The state index of every value.
        """
        return np.stack(
            [
                SymbolicEncoding(variable).indices_of_hashes(data[:, index]).astype(int)
                for index, variable in enumerate(self.variables)
            ],
            axis=1,
        )

    def number_of_states(self, *, variable: int) -> int:
        """
        :param variable: The index of a variable.
        :return: The number of states of the variable.
        """
        return len(self.variables[variable].domain.simple_sets)

    # %% initial circuit

    def compiled_chow_liu_tree(self, *, states: np.ndarray) -> StrudelNode:
        """
        :param states: The state index of every value of the training rows.
        :return: The Chow-Liu tree rooted at its Jordan center, compiled into a circuit
            whose sums select the state of a variable given the state of its parent.
        """
        tree = ChowLiuTree.from_mutual_information(
            mutual_information=PairwiseMutualInformation(
                discretized=DiscretizedData(
                    variables=self.variables, data=states.astype(float)
                )
            ).compute()
        )
        number_of_variables = len(self.variables)
        adjacency = np.zeros((number_of_variables, number_of_variables))
        for child, parent in enumerate(tree.parents):
            if parent >= 0:
                adjacency[child, parent] = adjacency[parent, child] = 1
        distances = shortest_path(adjacency, unweighted=True, directed=False)
        center = int(np.argmin(distances.max(axis=1)))
        parents = np.full(number_of_variables, -1)
        order = [center]
        for variable in order:
            for neighbour in np.flatnonzero(adjacency[variable]):
                if (
                    neighbour != center
                    and parents[neighbour] < 0
                    and neighbour not in order
                ):
                    parents[neighbour] = variable
                    order.append(int(neighbour))
        products: dict[int, list[StrudelNode]] = {}
        for variable in reversed(order):
            children = np.flatnonzero(parents == variable)
            products[variable] = [
                StrudelProduct(
                    children=[Indicator(variable=variable, state=state)]
                    + [
                        StrudelSum(children=list(products[int(child)]))
                        for child in children
                    ]
                )
                for state in range(self.number_of_states(variable=variable))
            ]
        return StrudelSum(children=list(products[center]))

    # %% splitting

    def best_split(
        self, *, flows: DeterministicFlows, states: np.ndarray
    ) -> tuple[StrudelSum, int, int] | None:
        """
        :param flows: The rows of every node and edge.
        :param states: The state index of every value of the training rows.
        :return: The sum, the position of its child and the variable of the split, or
            ``None`` if no edge leads to a child over a variable that varies on its rows.
        """
        ranked = sorted(
            flows.rows_of_edges.items(), key=lambda item: -int(item[1].sum())
        )
        for (node, position), rows in ranked:
            child = node.children[position]
            if len(child.scope) < 2 or rows.sum() < 2:
                continue
            scope = np.array(sorted(child.scope))
            child_states = states[np.ix_(rows, scope)]
            varying = np.array(
                [
                    len(np.unique(child_states[:, index])) > 1
                    for index in range(len(scope))
                ]
            )
            if not varying.any():
                continue
            mutual_information = PairwiseMutualInformation(
                discretized=DiscretizedData(
                    variables=SortedSet(self.variables[index] for index in scope),
                    data=child_states.astype(float),
                )
            ).compute()
            scores = np.where(varying, mutual_information.sum(axis=1), -np.inf)
            return node, position, int(scope[int(np.argmax(scores))])
        return None

    def split(
        self, *, root: StrudelNode, candidate: tuple[StrudelSum, int, int]
    ) -> StrudelNode:
        """
        :param root: The root of the circuit.
        :param candidate: The sum, the position of its child and the variable to split.
        :return: The root of the circuit after the split, in which the sum's child is
            replaced by one copy per state of the variable, conditioned on that state.
        """
        node, position, variable = candidate
        child = node.children[position]
        copies = [
            self.conditioned(node=child, variable=variable, state=state, cache={})
            for state in range(self.number_of_states(variable=variable))
        ]
        node.children[position : position + 1] = [
            copy for copy in copies if copy is not None
        ]
        return root

    def conditioned(
        self, *, node: StrudelNode, variable: int, state: int, cache: dict
    ) -> StrudelNode | None:
        """
        :param node: A node.
        :param variable: The variable to condition on.
        :param state: Its state.
        :param cache: The copies made so far in this conditioning, by node.
        :return: The node restricted to the rows with the variable in the state, the
            node itself if its scope does not hold the variable, or ``None`` if no row
            can reach it.
        """
        if variable not in node.scope:
            return node
        if node in cache:
            return cache[node]
        if isinstance(node, Indicator):
            result = node if node.state == state else None
        else:
            children = [
                self.conditioned(
                    node=child, variable=variable, state=state, cache=cache
                )
                for child in node.children
            ]
            if isinstance(node, StrudelProduct):
                result = (
                    None
                    if any(child is None for child in children)
                    else StrudelProduct(children=children)
                )
            else:
                kept = [child for child in children if child is not None]
                result = StrudelSum(children=kept) if kept else None
        cache[node] = result
        return result

    # %% conversion

    def to_layered_circuit(
        self, *, root: StrudelNode, states: np.ndarray
    ) -> LayeredProbabilisticCircuit:
        """
        :param root: The root of the circuit.
        :param states: The training rows, whose flows give the weights.
        :return: The circuit as a layered circuit with host parameters.
        """
        flows = DeterministicFlows(root=root, states=states)
        circuit = ProbabilisticCircuit()
        units: dict[StrudelNode, Unit] = {}
        for node in reversed(flows.nodes_parents_first()):
            if isinstance(node, Indicator):
                variable = self.variables[node.variable]
                units[node] = leaf(
                    SymbolicDistribution(
                        variable=variable,
                        probabilities=MissingDict(
                            float,
                            {int(SymbolicEncoding(variable).hashes[node.state]): 1.0},
                        ),
                    ),
                    circuit,
                )
            elif isinstance(node, StrudelProduct):
                units[node] = ProductUnit(probabilistic_circuit=circuit)
                for child in node.children:
                    units[node].add_subcircuit(units[child])
            else:
                units[node] = SumUnit(probabilistic_circuit=circuit)
                counts = (
                    np.array(
                        [
                            flows.rows_of_edges[(node, position)].sum()
                            for position in range(len(node.children))
                        ],
                        dtype=float,
                    )
                    + self.pseudocount
                )
                for child, count in zip(node.children, counts):
                    units[node].add_subcircuit(
                        units[child], np.log(count / counts.sum())
                    )
        return CircuitRepresentations().convert(circuit, LayeredProbabilisticCircuit)
