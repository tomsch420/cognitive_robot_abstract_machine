from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import rustworkx as rx
from matplotlib import pyplot as plt
from random_events.variable import Symbolic, Variable
from typing_extensions import Optional, Any, Self, List, Tuple, Set, Iterable, Dict

from probabilistic_model.distributions.distributions import SymbolicDistribution
from probabilistic_model.distributions.helper import make_dirac
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
    ProductUnit,
    SumUnit,
    LeafUnit,
    leaf,
)


@dataclass
class Node:
    """
    A node in the bayesian network These distributions do not inherit from probabilistic
    models, since inference in Bayesian Networks is intractable.

    For inference, convert the bayesian network to a probabilistic circuit.
    """

    bayesian_network: Optional[BayesianNetwork] = field(
        kw_only=True, repr=False, default=None
    )
    """
    The bayesian network this node is part of.
    """

    index: Optional[int] = field(kw_only=True, default=None, repr=False)
    """
    The index of the node in the graph of its circuit.
    """

    product_units: Dict[Any, ProductUnit] = field(
        init=False, default_factory=dict, repr=False
    )
    """
    A dictionary from states of the variable to product units.

    Only needed during conversion to probabilistic circuits.
    """

    def __post_init__(self):
        if self.bayesian_network is not None:
            self.bayesian_network.add_node(self)

    def __hash__(self):
        if self.bayesian_network is not None and self.index is not None:
            return hash((self.index, id(self.bayesian_network)))
        else:
            return id(self)

    @property
    def parent(self) -> Node:
        return self.bayesian_network.predecessors(self)[0]

    @property
    def variables(self) -> Tuple[Variable, ...]:
        raise NotImplementedError

    def as_probabilistic_circuit(self, result: ProbabilisticCircuit):
        """
        Add this node to the probabilistic circuit.

        This also creates all the edges implied by this node.

        :param result: The probabilistic circuit to add the nodes to.
        """
        raise NotImplementedError


from probabilistic_model.utils import get_subscript


@dataclass
class BayesianNetwork:
    """
    Class for Bayesian Networks that are rooted, tree shaped and have univariate inner
    nodes.

    This class does not inherit from ProbabilisticModel since it cannot perform
    inference. Bayesian Networks can be converted to a probabilistic circuit which can
    perform inference.
    """

    graph: rx.PyDAG[Node] = field(default_factory=lambda: rx.PyDAG(multigraph=False))
    """
    The graph to check connectivity from.
    """

    def __len__(self):
        """
        Return the number of nodes in the graph.

        :return: The number of nodes in the graph.
        """
        return len(self.graph)

    def __iter__(self):
        """
        Return an iterator over the nodes in the graph.

        :return: An iterator over the nodes in the graph.
        """
        return iter(self.graph.nodes())

    @property
    def leaves(self) -> List[Node]:
        return [node for node in self.nodes() if len(self.successors(node)) == 0]

    def is_valid(self) -> bool:
        """
        Check if this graph is:

        - acyclic
        - connected

        :return: True if the graph is valid, False otherwise.
        """
        return (
            rx.is_connected(self.graph)
            and (len(self.edges()) == (len(self.nodes()) - 1))
            and self.root
        )

    def add_node(self, node: Node):

        if node.bayesian_network is self and node.index is not None:
            return
        elif node.bayesian_network is not None and node.bayesian_network is not self:
            raise NotImplementedError(
                "Cannot add a node that already belongs to another bayesian network."
            )

        node.index = self.graph.add_node(node)

        # write self as the nodes bn
        node.bayesian_network = self

    def add_nodes_from(self, nodes: Iterable[Node]):
        [self.add_node(node) for node in nodes]

    def add_edge(self, parent: Node, child: Node):
        self.add_node(parent)
        self.add_node(child)
        self.graph.add_edge(parent.index, child.index, None)

    def add_edges_from(self, edges: Iterable[Tuple[Node, Node]]):
        [self.add_edge(*edge) for edge in edges]

    def successors(self, node: Node) -> List[Node]:
        return self.graph.successors(node.index)

    def descendants(self, unit: Node) -> Set[Node]:
        return {self.graph[unit] for unit in rx.descendants(self.graph, unit.index)}

    def predecessors(self, unit: Node) -> List[Node]:
        return self.graph.predecessors(unit.index)

    def in_edges(self, node: Node) -> List[Tuple[Node, Node, Optional[float]]]:
        return [
            (
                self.graph.get_node_data(parent_index),
                node,
                edge_data,
            )
            for parent_index, _, edge_data in self.graph.in_edges(node.index)
        ]

    def nodes(self) -> List[Node]:
        """
        Return an iterator over the nodes.

        :return: An iterator over the nodes.
        """
        return self.graph.nodes()

    def edges(self) -> List[Tuple[Node, Node]]:
        return [
            (self.graph[parent], self.graph[child])
            for parent, child in self.graph.edge_list()
        ]

    def in_degree(self, node: Node):
        return self.graph.in_degree(node.index)

    def has_edge(self, parent: Node, child: Node) -> bool:
        return self.graph.has_edge(parent.index, child.index)

    @property
    def root(self) -> Root:
        """
        The root of the circuit is the node with in-degree 0.

        This is the output node, that will perform the final computation.

        :return: The root of the circuit.
        """
        possible_roots = [node for node in self.nodes() if self.in_degree(node) == 0]
        if len(possible_roots) == 1:
            if not isinstance(possible_roots[0], Root):
                raise ValueError("The root is not an instance of Root.")
            return possible_roots[0]
        elif len(possible_roots) > 1:
            raise ValueError(
                f"More than one root found. Possible roots are {possible_roots}"
            )
        else:
            raise ValueError(f"No root found.")

    def __eq__(self, other: Self):
        raise NotImplementedError

    def __repr__(self):
        return f"{self.__class__.__name__} with {len(self.nodes())} nodes and {len(self.edges())} edges"

    def as_probabilistic_circuit(self) -> ProbabilisticCircuit:
        """
        Convert the bayesian network to a probabilistic circuit.

        :return: The probabilistic circuit.
        """
        result = ProbabilisticCircuit()

        for node in rx.topological_sort(self.graph):
            node = self.graph[node]
            node.as_probabilistic_circuit(result)

        result.remove_unreachable_nodes(self.root.root)
        result.simplify()

        return result

    def plot(self, filename: Optional[str] = None):
        """
        Plot the Bayesian Network using Graphviz.

        :param filename: The filename to save the plot to.
        """
        import graphviz

        dot = graphviz.Digraph(
            format="png",
            graph_attr={
                "rankdir": "LR",
                "nodesep": "0.8",
                "ranksep": "0.8",
                "fontname": "Helvetica",
                "ratio": "compress",
                "compound": "true",
                "newrank": "true",
            },
            node_attr={
                "style": "filled,rounded",
                "fontname": "Helvetica",
                "fontsize": "10",
                "penwidth": "2.0",
                "margin": "0.05",
            },
            edge_attr={
                "fontname": "Helvetica",
                "fontsize": "8",
                "color": "#455A64",
                "arrowsize": "0.6",
                "penwidth": "1.2",
            },
        )

        latent_counter = 0
        node_to_id = {}

        # Sort nodes to have deterministic λ numbering if they are latents
        nodes = self.nodes()

        # Helper to identify and style nodes
        def get_node_attributes(node: Node) -> dict:
            nonlocal latent_counter
            var = node.variables[0]
            name = var.name
            label = name
            shape = "box"
            fillcolor = "#E1F5FE"  # Light Blue for normal variables
            type_label = "Observable"

            # Check if it's a latent variable from a SumUnit
            if ".latent" in name:
                latent_counter += 1
                label = f"λ{get_subscript(latent_counter)}"
                fillcolor = "#FFF9C4"  # Light Yellow
                type_label = "Latent"
            # Check if it's an aggregation statistic
            elif "Aggregation" in name or "Aggregations" in name:
                shape = "hexagon"
                fillcolor = "#B3E5FC"  # Muted Blue for aggregations
                type_label = "Aggregation"
                # Strip owner class from label if present
                if "." in label:
                    label = ".".join(label.split(".")[1:])
            # Check if it's an aggregation statistic (MappedVariable check)
            elif hasattr(var, "_chain_root_"):
                try:
                    root = var._chain_root_
                    if hasattr(root, "_type_") and any(
                        base.__name__ == "AggregationStatistic"
                        for base in root._type_.mro()
                    ):
                        shape = "hexagon"
                        fillcolor = "#B3E5FC"  # Muted Blue for aggregations
                        type_label = "Aggregation"
                        if "." in label:
                            label = ".".join(label.split(".")[1:])
                except:
                    pass
            else:
                # For normal variables, strip the owner class name (first component)
                if "." in label:
                    label = ".".join(label.split(".")[1:])

            html_label = f'<<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="4"><TR><TD><B>{label}</B></TD></TR><TR><TD><FONT POINT-SIZE="8">{type_label}</FONT></TD></TR></TABLE>>'

            return {
                "label": html_label,
                "shape": shape,
                "fillcolor": fillcolor,
            }

        for node in nodes:
            node_id = f"node_{id(node)}"
            dot.node(node_id, **get_node_attributes(node))
            node_to_id[node] = node_id

        for parent, child in self.edges():
            dot.edge(node_to_id[parent], node_to_id[child])

        if filename is not None:
            # strip .png if present as dot appends it
            if filename.endswith(".png"):
                filename = filename[:-4]
            dot.render(filename, cleanup=True)
        return dot

    @classmethod
    def from_probabilistic_circuit(
        cls, circuit: ProbabilisticCircuit
    ) -> BayesianNetwork:
        """
        Create a Bayesian Network from a Probabilistic Circuit's structure.

        Each SumUnit in the circuit is represented as a latent variable, and all real
        variables from the circuit's leaves are also included as nodes. Edges represent
        the dependency structure of the circuit.

        :param circuit: The probabilistic circuit to convert.
        :return: A Bayesian Network representing the circuit's structure.
        """
        bn = cls()
        sum_unit_to_node = {}
        variable_to_node = {}

        # 1. Create nodes for SumUnits (latent variables)
        for node in circuit.graph.nodes():
            if isinstance(node, SumUnit):
                latent_var = node.latent_variable
                bn_node = StructureOnlyNode(variable=latent_var)
                bn.add_node(bn_node)
                sum_unit_to_node[node] = bn_node

        # 2. Create nodes for real Variables
        for variable in circuit.variables:
            bn_node = StructureOnlyNode(variable=variable)
            bn.add_node(bn_node)
            variable_to_node[variable] = bn_node

        # 3. Extract edges
        for sum_unit, bn_node in sum_unit_to_node.items():
            # Find direct descendants that are SumUnits or Variables
            visited = set()
            to_visit = list(circuit.graph.successors(sum_unit.index))
            while to_visit:
                current = to_visit.pop()
                if current.index in visited:
                    continue
                visited.add(current.index)

                if isinstance(current, SumUnit):
                    bn.add_edge(bn_node, sum_unit_to_node[current])
                elif isinstance(current, LeafUnit):
                    for var in current.variables:
                        bn.add_edge(bn_node, variable_to_node[var])
                elif isinstance(current, ProductUnit):
                    to_visit.extend(circuit.graph.successors(current.index))

        return bn


@dataclass
class StructureOnlyNode(Node):
    """
    A node that only represents the structure of a variable, without any distribution or
    conversion logic.

    Used for visualization.
    """

    variable: Variable

    __hash__ = Node.__hash__

    @property
    def variables(self) -> Tuple[Variable, ...]:
        return (self.variable,)

    def as_probabilistic_circuit(self, result: ProbabilisticCircuit):
        raise NotImplementedError("This node is only for visualization.")


@dataclass
class Root(Node):
    distribution: SymbolicDistribution

    root: Optional[SumUnit] = field(init=False, repr=False, default=None)
    """
    The root of the circuit that is generated by the as_probabilistic_circuit method.
    """

    __hash__ = Node.__hash__

    @property
    def variable(self) -> Symbolic:
        return self.distribution.variable

    @property
    def variables(self) -> Tuple[Variable, ...]:
        return self.distribution.variables

    def as_probabilistic_circuit(self, result: ProbabilisticCircuit):
        self.root = SumUnit(probabilistic_circuit=result)
        for value, probability in self.distribution.probabilities.items():
            prod = ProductUnit(probabilistic_circuit=result)
            distribution = leaf(
                make_dirac(
                    self.variable,
                    value,
                ),
                result,
            )
            self.root.add_subcircuit(
                prod,
                np.log(probability),
            )
            prod.add_subcircuit(distribution)
            self.product_units[value] = prod


@dataclass
class ConditionalProbabilityTable(Node):
    """
    Conditional probability distribution for Bayesian Network nodes given their parents.

    The parent in this case must be exactly one node.
    """

    conditional_probability_distributions: Dict[Any, SymbolicDistribution] = field(
        default_factory=dict
    )
    __hash__ = Node.__hash__

    @property
    def variable(self) -> Symbolic:
        return list(self.conditional_probability_distributions.values())[0].variable

    @property
    def variables(self) -> Tuple[Variable, ...]:
        return (self.variable,)

    def __repr__(self):
        return f"P({self.variable.name}|{self.parent.variable.name})"

    def to_tabulate(self) -> List[List[str]]:
        """
        Tabulate the truncated probability table.

        :return: A table with the truncated probability table that can be printed using
            tabulate.
        """
        table = [[self.parent.variable.name, self.variable.name, repr(self)]]

        parent_domain_hash_map = self.parent.variable.domain.hash_map
        own_domain_hash_map = self.variable.domain.hash_map

        for (
            parent_hash,
            distribution,
        ) in self.conditional_probability_distributions.items():
            for own_hash, probability in distribution.probabilities.items():
                table.append(
                    [
                        str(parent_domain_hash_map[parent_hash]),
                        str(own_domain_hash_map[own_hash]),
                        str(probability),
                    ]
                )
        return table

    def as_probabilistic_circuit(self, result: ProbabilisticCircuit):
        for value in self.variable.domain:
            prod = ProductUnit(probabilistic_circuit=result)
            distribution = leaf(
                make_dirac(
                    self.variable,
                    value,
                ),
                result,
            )
            prod.add_subcircuit(distribution)
            self.product_units[value.element] = prod

        parent = self.parent

        for (
            key,
            conditional_distribution,
        ) in self.conditional_probability_distributions.items():
            sum_unit = SumUnit(probabilistic_circuit=result)
            parent.product_units[key].add_subcircuit(sum_unit)

            for value, probability in conditional_distribution.probabilities.items():
                sum_unit.add_subcircuit(self.product_units[value], np.log(probability))


@dataclass
class ConditionalProbabilisticCircuit(Node):
    """
    Conditional probability distribution represented as Circuit for Bayesian Network
    nodes given their parents.
    """

    conditional_probability_distributions: Dict[int, ProbabilisticCircuit] = field(
        default_factory=dict
    )
    __hash__ = Node.__hash__

    @property
    def parent(self) -> ConditionalProbabilityTable:
        return super().parent

    @property
    def variables(self) -> Tuple[Variable, ...]:
        return tuple(
            list(self.conditional_probability_distributions.values())[0].variables
        )

    def __repr__(self):
        return f"P({', '.join([v.name for v in self.variables])} | {self.parent.variable.name})"

    def as_probabilistic_circuit(self, result: ProbabilisticCircuit):
        parent = self.parent

        for (
            key,
            conditional_distribution,
        ) in self.conditional_probability_distributions.items():
            old_root = conditional_distribution.root
            node_remap = result.mount(old_root)
            root_in_result = node_remap[old_root.index]
            parent.product_units[key].add_subcircuit(root_in_result)
