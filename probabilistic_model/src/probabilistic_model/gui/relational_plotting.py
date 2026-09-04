from __future__ import annotations
import graphviz
import html
from collections import defaultdict
from typing import TYPE_CHECKING, Optional, Dict, List, Any, Tuple, Type
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
    return get_aggregation_collection_name(variable, rspn) is not None


def get_aggregation_collection_name(
    variable: Any, rspn: Optional[RelationalProbabilisticCircuit] = None
) -> Optional[str]:
    """
    Get the name of the collection being aggregated by this variable.
    """
    if rspn is None:
        return None

    # If it is a MappedVariable, we can check its root type
    if isinstance(variable, MappedVariable):
        root = variable._chain_root_
        if (
            hasattr(root, "_type_")
            and isinstance(root._type_, type)
            and issubclass(root._type_, AggregationStatistic)
        ):
            # Try to find which field this AggregationStatistic is for
            if rspn.feature_extractor:
                name = variable.get_clean_name_from_mapped_variable()
                for (
                    field_name,
                    features,
                ) in rspn.feature_extractor.exchangeable_features.items():
                    for f in features:
                        if hasattr(f, "get_clean_name_from_mapped_variable"):
                            if f.get_clean_name_from_mapped_variable() == name:
                                return field_name
            return "unknown"
        return None

    # If it is a random_events Variable, check its name
    name = getattr(variable, "name", str(variable))
    if name.endswith("()"):
        name = name[:-2]

    if rspn is not None and rspn.feature_extractor is not None:
        for (
            field_name,
            features,
        ) in rspn.feature_extractor.exchangeable_features.items():
            for f in features:
                f_name = None
                if hasattr(f, "_name_"):
                    f_name = f._name_
                elif hasattr(f, "get_clean_name_from_mapped_variable"):
                    f_name = f.get_clean_name_from_mapped_variable()

                if f_name and (name == f_name or name.endswith("." + f_name)):
                    return field_name

    # Try looking at AggregationStatistic classes directly
    from krrood.parametrization.feature_extraction.aggregations import (
        get_aggregation_class,
    )

    if rspn is not None:
        agg_cls = get_aggregation_class(rspn.class_)
        if agg_cls is not None:
            # Check the registry for each field
            # The registry might be on ancestors, so we check the field names
            # available on the DAO schema if possible, or just all fields in registry
            for field_name in agg_cls.aggregation_registry:
                for func in agg_cls.aggregation_registry[field_name]:
                    if name == func.__name__ or name.endswith("." + func.__name__):
                        return field_name
            # Also check inherited ones
            for field_name in getattr(
                rspn.schema_information, "collection_relationships", []
            ):
                if isinstance(field_name, str):
                    fname = field_name
                else:
                    fname = getattr(field_name, "key", None)
                if fname:
                    for func in agg_cls.aggregation_features_of_field(fname):
                        if name == func.__name__ or name.endswith("." + func.__name__):
                            return fname

    if "Aggregation" in name or "Aggregations" in name:
        return "unknown"

    return None


def get_unique_part_path(
    variable: Any, root_class: Type
) -> List[Tuple[str, Optional[str]]]:
    """
    Get the path of unique parts for a variable.

    Example: person.arm.hand.finger_count -> [('arm', 'Arm'), ('hand', 'Hand')]
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
                rel = next(
                    (
                        r
                        for r in schema.single_relationships
                        if r.key == step._attribute_name_
                    ),
                    None,
                )
                if rel:
                    target_class = rel.target_dao_class or rel.domain_type
                    type_name = getattr(target_class, "__name__", str(target_class))
                    if type_name.endswith("DAO"):
                        type_name = type_name[:-3]
                    path.append((step._attribute_name_, type_name))
                else:
                    # Once we hit a non-relationship attribute, we are at the end of the part path
                    break
            except:
                break
        return path

    # If it is a random_events Variable, we use its name to deduce the path
    name = getattr(variable, "name", str(variable))
    parts = name.split(".")

    current_class = root_class
    path = []

    # The first part is usually the root class name, skip it if so
    start_index = 0
    if parts:
        first_part = parts[0]
        class_name = getattr(root_class, "__name__", str(root_class))
        if first_part == class_name or first_part == class_name.replace("DAO", ""):
            start_index = 1

    for part in parts[start_index:]:
        try:
            from krrood.ormatic.data_access_objects.helper import get_dao_class

            dao_class = get_dao_class(current_class)
            if dao_class is None:
                break
            schema = get_dao_schema(dao_class)

            rel = next((r for r in schema.single_relationships if r.key == part), None)
            if rel:
                target_class = rel.target_dao_class or rel.domain_type
                type_name = getattr(target_class, "__name__", str(target_class))
                if type_name.endswith("DAO"):
                    type_name = type_name[:-3]
                path.append((part, type_name))
                current_class = target_class
            else:
                break
        except:
            break
    return path


from probabilistic_model.utils import get_subscript, wrap_text, clean_type_name


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
                "rankdir": "TB",
                "nodesep": "0.6",
                "ranksep": "0.8",
                "fontname": "Helvetica-Bold",
                "fontsize": "12",
                "newrank": "true",
                "ordering": "out",
                "splines": "curved",
                "concentrate": "true",
                "bgcolor": "white",
                "pad": "0.3",
            },
            node_attr={
                "style": "filled,rounded",
                "fontname": "Helvetica",
                "fontsize": "10",
                "penwidth": "1.2",
                "margin": "0.1",
                "width": "1.4",
                "height": "0.3",
            },
            edge_attr={
                "fontname": "Helvetica",
                "fontsize": "8",
                "color": "#78909C",
                "arrowsize": "0.7",
                "penwidth": "1.0",
            },
        )
        self.node_to_id = {}
        self.node_to_rank = {}
        self.nodes_by_global_rank = defaultdict(list)
        self.cluster_id_by_path = {}
        self.cluster_counter = 0

        self.class_colors = [
            ("#DCFCE7", "#16A34A"),  # Green
            ("#FEF9C3", "#CA8A04"),  # Yellow
            ("#CFFAFE", "#0891B2"),  # Cyan
            ("#F3E8FF", "#9333EA"),  # Purple
            ("#FFEDD5", "#EA580C"),  # Orange
            ("#FCE7F3", "#DB2777"),  # Pink
            ("#CCFBF1", "#0D9488"),  # Teal
            ("#E0E7FF", "#4F46E5"),  # Indigo
            ("#F5F5F4", "#57534E"),  # Stone
            ("#F1F5F9", "#475569"),  # Slate
        ]
        self.assigned_colors = {
            "Position": self.class_colors[0],
            "KRROODPosition": self.class_colors[0],
            "Orientation": self.class_colors[1],
            "KRROODOrientation": self.class_colors[1],
        }
        self.color_index = 2

    def _get_colors_for_type(self, type_name: str) -> Tuple[str, str]:
        if type_name not in self.assigned_colors:
            self.assigned_colors[type_name] = self.class_colors[
                self.color_index % len(self.class_colors)
            ]
            self.color_index += 1
        return self.assigned_colors[type_name]

    def _get_cluster_id(self) -> str:
        self.cluster_counter += 1
        return f"cluster_{self.cluster_counter}"

    def _get_variable_type_from_features(
        self, var_name: str, var: Variable, rspn: RelationalProbabilisticCircuit
    ) -> Optional[str]:
        """
        Deduce the type of a variable by looking it up in the feature extractor or on
        the variable itself.
        """
        t = None
        if hasattr(var, "_type_") and var._type_ is not None:
            t = var._type_

        if t is None and rspn.feature_extractor:
            for feature in rspn.feature_extractor.features:
                if hasattr(feature, "get_clean_name_from_mapped_variable"):
                    clean_name = feature.get_clean_name_from_mapped_variable()
                    # The var_name from BN might have class prefix.
                    if var_name.endswith(clean_name):
                        t = feature._type_
                        break

        if t is None:
            return None

        name = getattr(t, "__name__", str(t))
        if name.endswith("DAO"):
            name = name[:-3]
        # Handle typing.Optional[float] etc.
        if "Optional[" in name:
            name = name.replace("Optional[", "").replace("]", "")
        if "'" in name:
            name = name.split("'")[1]
        return name

    def plot(self, filename: str) -> graphviz.Digraph:
        self._add_rspn(self.rspn, self.dot)

        # Add global rank constraints
        for rank, nodes in self.nodes_by_global_rank.items():
            if len(nodes) > 1:
                with self.dot.subgraph() as s:
                    s.attr(rank="same")
                    for node_id in nodes:
                        s.node(node_id)

        if filename.endswith(".png"):
            filename = filename[:-4]
        self.dot.render(filename, cleanup=True)
        return self.dot

    def _add_rspn(
        self,
        rspn: RelationalProbabilisticCircuit,
        parent_graph: graphviz.Digraph,
        prefix: str = "",
        rank_offset: int = 0,
    ) -> str:
        class_name = rspn.class_.__name__
        if class_name.endswith("DAO"):
            class_name = class_name[:-3]
        cluster_id = self._get_cluster_id()
        self.cluster_id_by_path[(prefix, ())] = cluster_id

        fillcolor, bordercolor = self._get_colors_for_type(class_name)

        with self.dot.subgraph(name=cluster_id) as c:
            c.attr(
                label=class_name,
                style="filled,rounded",
                fillcolor=fillcolor,
                color=bordercolor,
                penwidth="2.0",
                fontname="Helvetica-Bold",
                fontsize="11",
                labeljust="c",
            )

            # Anchor node for connections to/from this cluster
            anchor_id = f"anchor_{cluster_id}"
            c.node(anchor_id, "", shape="none", width="0", height="0")

            if rspn.class_probabilistic_circuit is not None:
                bn_depth = self._add_bn(
                    rspn, c, cluster_id, prefix=prefix, rank_offset=rank_offset
                )
            else:
                c.node(f"not_fitted_{cluster_id}", "Not Fitted", shape="none")
                bn_depth = 1

            # Exchangeable parts
            for (
                field_name,
                template,
            ) in rspn.exchangeable_distribution_templates.items():
                child_rspn = template.template_distribution
                # print(f"DEBUG: Processing field '{field_name}' in class '{class_name}' (prefix='{prefix}')")
                # print(f"DEBUG: exchangeable_features has: {list(rspn.feature_extractor.exchangeable_features.keys())}")

                # Determine rank offset for child based on aggregation node rank in parent
                child_rank_offset = rank_offset + bn_depth

                for var in rspn.feature_extractor.exchangeable_features.get(
                    field_name, []
                ):
                    # Try to find the rank of the aggregation node
                    # This is a bit tricky as variables are mapped to nodes
                    node = None
                    # Search for node in current BN that models this variable
                    if rspn.class_probabilistic_circuit is not None:
                        # We need to find which node in the BN corresponds to 'var'
                        # Actually, we can just look up in self.node_to_rank if we had the node
                        pass

                child_cluster_id = self._add_rspn(
                    child_rspn,
                    c,
                    prefix=f"{prefix}_{field_name}",
                    rank_offset=child_rank_offset,
                )

                # Connect aggregation nodes to this cluster
                # We need to find nodes that represent aggregation statistics for this field
                child_prefix = f"{prefix}_{field_name}"
                for var in rspn.feature_extractor.exchangeable_features.get(
                    field_name, []
                ):
                    # Find the node for this variable in the BN
                    node_id = None
                    var_name = None
                    if hasattr(var, "_name_"):
                        var_name = var._name_
                    elif hasattr(var, "get_clean_name_from_mapped_variable"):
                        var_name = var.get_clean_name_from_mapped_variable()
                    else:
                        var_name = getattr(var, "name", str(var))

                    if var_name:
                        node_id = self.node_to_id.get((prefix, var_name))
                        # print(f"DEBUG: Lookup node_id for prefix='{prefix}', var='{var_name}': {node_id}")

                    if node_id:
                        # Look for specific aggregation cluster in parent
                        parent_agg_cluster_id = self.cluster_id_by_path.get(
                            (prefix, (("Aggregations", field_name),))
                        )
                        if not parent_agg_cluster_id:
                            parent_agg_cluster_id = self.cluster_id_by_path.get(
                                (prefix, (("Aggregations", "unknown"),))
                            )

                        # Determine source and target (reversed: Child -> Parent)
                        # Try to find a specific node in the child cluster to connect from
                        source_node_id = None
                        ltail = None

                        # Heuristic: Find a child variable whose name is part of the parent aggregation name
                        clean_parent_name = var_name.split(".")[-1].replace("()", "")
                        for word in [
                            "sum_",
                            "avg_",
                            "max_",
                            "min_",
                            "count_",
                            "total_",
                        ]:
                            if clean_parent_name.startswith(word):
                                clean_parent_name = clean_parent_name[len(word) :]

                        for child_var in child_rspn.variables:
                            if ".latent" in child_var.name:
                                continue
                            clean_child_name = child_var.name.split(".")[-1].replace(
                                "()", ""
                            )
                            if (
                                clean_child_name == clean_parent_name
                                or clean_child_name in clean_parent_name
                                or clean_parent_name in clean_child_name
                            ):
                                source_node_id = self.node_to_id.get(
                                    (child_prefix, child_var.name)
                                )
                                if source_node_id:
                                    break

                        if not source_node_id:
                            # Fallback: Find root latent node of child
                            for cv in child_rspn.class_probabilistic_circuit.variables:
                                if ".latent" in cv.name:
                                    source_node_id = self.node_to_id.get(
                                        (child_prefix, cv.name)
                                    )
                                    if source_node_id:
                                        break

                        if not source_node_id:
                            source_node_id = f"anchor_{child_cluster_id}"
                            ltail = child_cluster_id

                        # Parent is the target
                        target_node_id = node_id
                        lhead = None

                        # print(f"DEBUG: Edge from {source_node_id} to {target_node_id} (prefix='{prefix}', var='{var_name}')")
                        self.dot.edge(
                            source_node_id,
                            target_node_id,
                            style="dashed",
                            arrowhead="odiamond",
                            ltail=ltail,
                            lhead=lhead,
                            color="#546E7A",
                        )

        return cluster_id

    def _add_bn(
        self,
        rspn: RelationalProbabilisticCircuit,
        parent_cluster: graphviz.Digraph,
        cluster_id: str,
        prefix: str = "",
        rank_offset: int = 0,
    ) -> int:
        circuit = rspn.class_probabilistic_circuit
        bn = BayesianNetwork.from_probabilistic_circuit(circuit)

        # Record node ranks
        try:
            layers = bn.layers
            for i, layer in enumerate(layers):
                for node in layer:
                    self.node_to_rank[node] = rank_offset + i
            depth = len(layers)
        except:
            depth = 1

        # Group nodes by their unique part path
        nodes_by_path = defaultdict(list)

        for node in bn.nodes():
            var = node.variables[0]
            agg_collection = get_aggregation_collection_name(var, rspn)
            if agg_collection:
                nodes_by_path[(("Aggregations", agg_collection),)].append(node)
            elif ".latent" in var.name:
                nodes_by_path[()].append(node)
            else:
                path = tuple(get_unique_part_path(var, rspn.class_))
                nodes_by_path[path].append(node)

        latent_counter = 0

        # Function to recursively add clusters and nodes
        def add_nodes_to_clusters(
            current_path: Tuple[Tuple[str, Optional[str]], ...],
            current_graph: graphviz.Digraph,
        ):
            nonlocal latent_counter

            # Add anchor for this cluster if it's an Aggregations cluster
            if current_path and current_path[0][0] == "Aggregations":
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
                cardinality_row = ""

                if is_aggregation_variable(var, rspn):
                    shape = "hexagon"
                    color = "#DBEAFE"  # Blue
                    field_name = name.split(".")[-1]
                    # strip "aggregation" from label if present
                    clean_label = field_name
                    for word in [
                        "_aggregation",
                        "aggregation",
                        "_Aggregation",
                        "Aggregation",
                    ]:
                        clean_label = clean_label.replace(word, "")

                    var_type = self._get_variable_type_from_features(name, var, rspn)
                    label = wrap_text(clean_label)
                    type_label = clean_type_name(var_type)
                elif ".latent" in name:
                    shape = "box"
                    color = "#FFF9C4"  # Light Yellow
                    latent_counter += 1
                    label = f"λ{get_subscript(latent_counter)}"
                    type_label = None
                    cardinality_row = ""
                    if (
                        isinstance(node, StructureOnlyNode)
                        and node.cardinality is not None
                    ):
                        cardinality_row = f'<TR><TD ALIGN="CENTER"><FONT POINT-SIZE="7">|{label}| = {node.cardinality}</FONT></TD></TR>'
                else:
                    label = wrap_text(name.split(".")[-1])
                    type_label = clean_type_name(
                        self._get_variable_type_from_features(name, var, rspn)
                    )

                if type_label:
                    type_label = clean_type_name(type_label)
                    label = f'{label}: <FONT POINT-SIZE="7" COLOR="#555555">{html.escape(type_label)}</FONT>'
                else:
                    label = f'{label}'

                html_label = f'<<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="2"><TR><TD ALIGN="CENTER"><B>{label}</B></TD></TR>{cardinality_row}</TABLE>>'

                node_id = f"node_{id(node)}"
                current_graph.node(node_id, html_label, shape=shape, fillcolor=color)
                self.node_to_id[node] = node_id
                self.node_to_id[(prefix, name)] = node_id

                # Record global rank
                rank = self.node_to_rank.get(node)
                if rank is not None:
                    self.nodes_by_global_rank[rank].append(node_id)

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
                with self.dot.subgraph(name=child_cluster_id) as sub:
                    name, type_name = child_path[-1]
                    if name == "Aggregations":
                        label = (
                            f"Aggregations: {type_name}"
                            if type_name and type_name != "unknown"
                            else "Aggregations"
                        )
                        sub.attr(
                            label=label,
                            style="filled,rounded",
                            fillcolor="#DBEAFE",
                            color="#2563EB",
                            fontname="Helvetica-Bold",
                            fontsize="10",
                            labeljust="c",
                            penwidth="1.2",
                        )
                    else:
                        label = f"{name}: {type_name}" if type_name else name
                        fillcolor, bordercolor = self._get_colors_for_type(
                            type_name or name
                        )
                        sub.attr(
                            label=label,
                            style="filled,rounded",
                            fillcolor=fillcolor,
                            color=bordercolor,
                            fontname="Helvetica-Bold",
                            fontsize="10",
                            labeljust="c",
                            penwidth="1.2",
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

        return depth
