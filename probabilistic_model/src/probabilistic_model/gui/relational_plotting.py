from __future__ import annotations
import graphviz
from collections import defaultdict
from typing import TYPE_CHECKING, Optional, Dict, List, Any, Tuple
import rustworkx as rx

from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    SumUnit,
    LeafUnit,
    ProductUnit,
)
from probabilistic_model.bayesian_network.bayesian_network import (
    BayesianNetwork,
    StructureOnlyNode,
)
from krrood.entity_query_language.core.mapped_variable import MappedVariable, Attribute
from krrood.parametrization.feature_extraction.aggregations import AggregationStatistic
from krrood.ormatic.data_access_objects.dao import get_dao_schema

if TYPE_CHECKING:
    from probabilistic_model.probabilistic_circuit.relational.rspn import (
        RelationalProbabilisticCircuit,
        ExchangeableDistributionTemplate,
    )


def is_aggregation_variable(
    variable: Any, rspn: Optional[RelationalProbabilisticCircuit] = None
) -> bool:
    """
    Check if a variable is an aggregation statistic.
    """
    # If it is a MappedVariable, we can check its root type
    if isinstance(variable, MappedVariable):
        root = variable._chain_root_
        if (
            hasattr(root, "_type_")
            and isinstance(root._type_, type)
            and issubclass(root._type_, AggregationStatistic)
        ):
            return True
        return False

    # If it is a random_events Variable, check its name
    name = getattr(variable, "name", str(variable))
    if "Aggregation" in name or "Aggregations" in name:
        return True

    if rspn is not None and rspn.feature_extractor is not None:
        all_aggregations = set()
        for features in rspn.feature_extractor.exchangeable_features.values():
            for f in features:
                if hasattr(f, "_name_"):
                    all_aggregations.add(f._name_)
                if hasattr(f, "get_clean_name_from_mapped_variable"):
                    all_aggregations.add(f.get_clean_name_from_mapped_variable())
        if name in all_aggregations:
            return True

    return False


def get_unique_part_path(variable: Any, root_class: Type) -> List[str]:
    """
    Get the path of unique parts for a variable.

    Example: person.arm.hand.finger_count -> ['arm', 'hand']
    """
    if isinstance(variable, MappedVariable):
        path = []
        for step in variable._access_path_:
            if not isinstance(step, Attribute):
                continue

            owner = step._owner_class_
            if owner is None:
                continue

            try:
                from krrood.ormatic.data_access_objects.helper import get_dao_class

                dao_class = get_dao_class(owner)
                if dao_class is None:
                    break
                schema = get_dao_schema(dao_class)
                if any(
                    r.key == step._attribute_name_ for r in schema.single_relationships
                ):
                    path.append(step._attribute_name_)
                else:
                    # Once we hit a non-relationship attribute, we are at the end of the part path
                    break
            except:
                break
        return path

    # If it is a random_events Variable, we use its name to deduce the path
    name = getattr(variable, "name", str(variable))
    parts = name.split(".")
    # The first part is usually the root class name, skip it
    current_class = root_class
    path = []
    for part in parts[1:]:
        try:
            from krrood.symbol_graph.helpers import get_field_type_endpoint
            from krrood.ormatic.data_access_objects.helper import get_dao_class

            dao_class = get_dao_class(current_class)
            if dao_class is None:
                break
            schema = get_dao_schema(dao_class)
            if any(r.key == part for r in schema.single_relationships):
                path.append(part)
                current_class = get_field_type_endpoint(current_class, part)
            else:
                break
        except:
            break
    return path


from probabilistic_model.utils import get_subscript


class RSPNUMLPlotter:
    """
    Plotter for Relational Probabilistic Circuits in UML style.
    """

    def __init__(self, rspn: RelationalProbabilisticCircuit):
        self.rspn = rspn
        self.dot = graphviz.Digraph(
            format="png",
            graph_attr={
                "compound": "true",
                "rankdir": "LR",
                "nodesep": "0.8",
                "ranksep": "0.8",
                "fontname": "Helvetica",
                "fontsize": "12",
                "ratio": "compress",
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
        self.node_to_id = {}
        self.cluster_id_by_path = {}
        self.cluster_counter = 0

    def _get_cluster_id(self) -> str:
        self.cluster_counter += 1
        return f"cluster_{self.cluster_counter}"

    def plot(self, filename: str):
        self._add_rspn(self.rspn, self.dot)
        if filename.endswith(".png"):
            filename = filename[:-4]
        self.dot.render(filename, cleanup=True)

    def _add_rspn(
        self,
        rspn: RelationalProbabilisticCircuit,
        parent_graph: graphviz.Digraph,
        prefix: str = "",
    ) -> str:
        class_name = rspn.class_.__name__
        cluster_id = self._get_cluster_id()
        self.cluster_id_by_path[(prefix, ())] = cluster_id

        with parent_graph.subgraph(name=cluster_id) as c:
            c.attr(
                label=class_name,
                style="filled,rounded",
                fillcolor="#F5F5F5",
                color="#BDBDBD",
                penwidth="2.0",
                fontname="Helvetica",
            )

            # Anchor node for connections to/from this cluster
            anchor_id = f"anchor_{cluster_id}"
            c.node(anchor_id, "", shape="none", width="0", height="0")

            if rspn.class_probabilistic_circuit is not None:
                self._add_bn(rspn, c, cluster_id, prefix=prefix)
            else:
                c.node(f"not_fitted_{cluster_id}", "Not Fitted", shape="none")

            # Exchangeable parts
            for (
                field_name,
                template,
            ) in rspn.exchangeable_distribution_templates.items():
                child_rspn = template.template_distribution
                child_cluster_id = self._add_rspn(
                    child_rspn, self.dot, prefix=f"{prefix}_{field_name}"
                )

                # Connect aggregation nodes to this cluster
                # We need to find nodes that represent aggregation statistics for this field
                drawn_edge = False
                for var in rspn.feature_extractor.exchangeable_features.get(
                    field_name, []
                ):
                    if drawn_edge:
                        break

                    # Find the node for this variable in the BN
                    node_id = None
                    if hasattr(var, "_name_"):
                        node_id = self.node_to_id.get(var._name_)
                    if not node_id and hasattr(
                        var, "get_clean_name_from_mapped_variable"
                    ):
                        node_id = self.node_to_id.get(
                            var.get_clean_name_from_mapped_variable()
                        )
                    if not node_id:
                        node_id = self.node_to_id.get(getattr(var, "name", str(var)))

                    if node_id:
                        child_prefix = f"{prefix}_{field_name}"
                        child_agg_cluster_id = self.cluster_id_by_path.get(
                            (child_prefix, ("Aggregations",))
                        )
                        parent_agg_cluster_id = self.cluster_id_by_path.get(
                            (prefix, ("Aggregations",))
                        )

                        # Determine source and target
                        if child_agg_cluster_id:
                            source_node_id = f"anchor_{child_agg_cluster_id}"
                            ltail = child_agg_cluster_id
                        else:
                            source_node_id = f"anchor_{child_cluster_id}"
                            ltail = child_cluster_id

                        if parent_agg_cluster_id:
                            target_node_id = f"anchor_{parent_agg_cluster_id}"
                            lhead = parent_agg_cluster_id
                        else:
                            target_node_id = node_id
                            lhead = None

                        self.dot.edge(
                            source_node_id,
                            target_node_id,
                            style="dashed",
                            ltail=ltail,
                            lhead=lhead,
                        )
                        drawn_edge = True

        return cluster_id

    def _add_bn(
        self,
        rspn: RelationalProbabilisticCircuit,
        parent_cluster: graphviz.Digraph,
        cluster_id: str,
        prefix: str = "",
    ):
        circuit = rspn.class_probabilistic_circuit
        bn = BayesianNetwork.from_probabilistic_circuit(circuit)

        # Group nodes by their unique part path
        nodes_by_path = defaultdict(list)

        for node in bn.nodes():
            var = node.variables[0]
            if is_aggregation_variable(var, rspn):
                nodes_by_path[("Aggregations",)].append(node)
            elif ".latent" in var.name:
                nodes_by_path[()].append(node)
            else:
                path = tuple(get_unique_part_path(var, rspn.class_))
                nodes_by_path[path].append(node)

        latent_counter = 0

        # Function to recursively add clusters and nodes
        def add_nodes_to_clusters(
            current_path: Tuple[str, ...], current_graph: graphviz.Digraph
        ):
            nonlocal latent_counter

            # Add anchor for this cluster if it's the Aggregations cluster
            if current_path == ("Aggregations",):
                current_graph.node(
                    f"anchor_{current_graph.name}",
                    "",
                    shape="none",
                    width="0",
                    height="0",
                )

            # Add nodes for this path
            for node in nodes_by_path[current_path]:
                var = node.variables[0]
                name = var.name
                shape = "box"
                color = "#E1F5FE"  # Light Blue
                type_label = "Observable"

                if is_aggregation_variable(var, rspn):
                    shape = "hexagon"
                    color = "#B3E5FC"  # Muted Blue
                    label = name.split(".")[-1]
                    # strip "aggregation" from label if present
                    clean_label = label
                    for word in [
                        "_aggregation",
                        "aggregation",
                        "_Aggregation",
                        "Aggregation",
                    ]:
                        clean_label = clean_label.replace(word, "")
                    if clean_label:
                        label = clean_label
                    type_label = None
                elif ".latent" in name:
                    shape = "box"
                    color = "#FFF9C4"  # Light Yellow
                    latent_counter += 1
                    label = f"λ{get_subscript(latent_counter)}"
                    type_label = "Latent"
                else:
                    label = name.split(".")[-1]

                type_row = (
                    f'<TR><TD><FONT POINT-SIZE="8">{type_label}</FONT></TD></TR>'
                    if type_label
                    else ""
                )
                html_label = f'<<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="4"><TR><TD><B>{label}</B></TD></TR>{type_row}</TABLE>>'

                node_id = f"node_{id(node)}"
                current_graph.node(node_id, html_label, shape=shape, fillcolor=color)
                self.node_to_id[node] = node_id
                self.node_to_id[var] = node_id
                self.node_to_id[name] = node_id

            # Find sub-paths
            sub_paths = {
                path
                for path in nodes_by_path
                if len(path) > len(current_path)
                and path[: len(current_path)] == current_path
            }
            # Only immediate children
            immediate_children = {path[: len(current_path) + 1] for path in sub_paths}

            for child_path in sorted(immediate_children):
                child_cluster_id = self._get_cluster_id()
                self.cluster_id_by_path[(prefix, child_path)] = child_cluster_id
                with current_graph.subgraph(name=child_cluster_id) as sub:
                    if child_path == ("Aggregations",):
                        sub.attr(
                            label=child_path[-1],
                            style="filled,rounded",
                            fillcolor="#E1F5FE",
                            color="#90A4AE",
                            fontname="Helvetica",
                        )
                    else:
                        sub.attr(
                            label=child_path[-1],
                            style="dashed,rounded",
                            color="#90A4AE",
                            fontname="Helvetica",
                        )
                    add_nodes_to_clusters(child_path, sub)

        add_nodes_to_clusters((), parent_cluster)

        # Add edges (BN edges are within the BN)
        for parent, child in bn.edges():
            parent_id = self.node_to_id.get(parent)
            child_id = self.node_to_id.get(child)
            if parent_id and child_id:
                # Add edges to the top-level RSPN cluster to ensure they can cross sub-clusters
                parent_cluster.edge(parent_id, child_id)
