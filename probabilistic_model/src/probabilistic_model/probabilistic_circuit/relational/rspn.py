"""
Relational probabilistic circuits ("RSPNs").

.. note::
    This module deliberately bridges ``probabilistic_model`` and ``krrood``: it
    imports krrood feature extraction here, while ``krrood.parametrization.model_registries``
    imports :class:`RelationalProbabilisticCircuit` back. This bidirectional coupling
    predates the relational refactor and is kept intentionally; it is the seam where
    krrood's symbolic feature extraction meets probabilistic_model's circuits.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

import numpy as np
import pandas as pd
from sortedcontainers import SortedSet
from typing_extensions import TYPE_CHECKING, Any, Optional, Type

from krrood.entity_query_language.query.match import AbstractMatchExpression
from krrood.ormatic.data_access_objects.dao import (
    DataAccessObject,
    DataAccessObjectSchema,
    get_dao_schema,
)
from krrood.ormatic.data_access_objects.from_dao import FromDataAccessObjectState
from krrood.ormatic.data_access_objects.helper import to_dao
from krrood.parametrization.feature_extraction.aggregations import (
    compute_aggregation_statistics,
)
from krrood.parametrization.feature_extraction.feature_extractor import FeatureExtractor

if TYPE_CHECKING:
    from krrood.entity_query_language.query.match import Match
from probabilistic_model.learning.jpt.jpt import JointProbabilityTree
from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
from probabilistic_model.probabilistic_circuit.relational.exceptions import (
    CircuitNotFittedError,
)
from probabilistic_model.probabilistic_circuit.relational.helper import (
    find_lowest_product_nodes_that_model_variables,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
    ProductUnit,
    SumUnit,
)
from random_events.variable import Variable


def _rename_variables_with_part_prefix(
    circuit: ProbabilisticCircuit,
    prefix: str,
    excluded_variables: list[Variable],
) -> None:
    """
    Rename each variable in the circuit to include ``prefix`` as a namespace.

    Produces names of the form ``"{prefix}.{variable.name}"``.
    Variables listed in ``excluded_variables`` are left unchanged.

    .. note::
        ``ProbabilisticCircuit.leaves`` returns an empty list when the root is
        itself a leaf (single-node circuit), because ``LeafUnit.leaves`` is defined
        as ``[]``.  The rename is applied to those root leaves directly to handle
        that case.

    :param circuit: The circuit whose variables are renamed in-place.
    :param prefix: String prefix to prepend to every variable name.
    :param excluded_variables: Variables that should keep their current names.
    """
    variable_renames = {
        variable: type(variable)(f"{prefix}.{variable.name}", domain=variable.domain)
        for variable in circuit.variables
        if variable not in excluded_variables
    }
    circuit.update_variables(variable_renames)
    # When the root is a leaf, update_variables misses it (LeafUnit.leaves == []).
    if circuit.root.is_leaf and circuit.root.variable in variable_renames:
        circuit.root.distribution.variable = variable_renames[circuit.root.variable]


@dataclass
class ExchangeableDistributionTemplate:
    """
    A fitted distribution template for one exchangeable (many-to-many) relation.

    Wraps a ``RelationalProbabilisticCircuit`` that was trained on the child
    objects of the relation together with the parent's aggregation statistics as
    latent context variables.
    """

    template_distribution: RelationalProbabilisticCircuit
    """
    The fitted ``RelationalProbabilisticCircuit`` representing the child distribution.
    """

    latent_variables: list[Variable] = field(default_factory=list)
    """
    Variables shared between the parent and child circuits that are used for
    conditioning but are not part of the final grounded distribution.
    """

    def _ground_part_circuit(
        self, part, aggregation_statistics: dict[Variable, Any], index: int = 0
    ) -> ProbabilisticCircuit:
        """
        Ground and prepare the circuit for a single exchangeable part.

        Conditions the template circuit on ``aggregation_statistics``, marginalizes
        away the latent variables, renames surviving variables with the part's
        prefix, and reindexes the graph for safe mounting.

        :param part: The query part (a ``Match`` or a concrete domain object).
        :param aggregation_statistics: Observed aggregation values to condition on.
        :param index: Position of this part in its parent list; used as fallback prefix
            when ``part`` does not carry a symbolic variable.
        :return: A self-contained circuit ready to be mounted into the parent.
        """
        part_circuit = self.template_distribution.ground(part)
        conditioning_result, _ = part_circuit.log_conditional_in_place(
            aggregation_statistics
        )
        if conditioning_result is None:
            part_circuit = self.template_distribution.ground(part)
        non_latent_variables = [
            variable
            for variable in part_circuit.variables
            if variable not in self.latent_variables
        ]
        part_circuit.marginal_in_place(non_latent_variables)
        prefix = (
            str(part.variable)
            if isinstance(part, AbstractMatchExpression)
            else str(index)
        )
        _rename_variables_with_part_prefix(part_circuit, prefix, self.latent_variables)
        if len(part_circuit.nodes()) == 0:
            raise ValueError("The grounding of the part failed.")
        return part_circuit

    def ground(
        self, parts_to_ground: list, aggregation_statistics: dict[Variable, Any]
    ) -> ProbabilisticCircuit:
        """
        Build a product circuit by grounding each exchangeable part independently.

        :param parts_to_ground: The query parts, one per child object in the relation.
        :param aggregation_statistics: Observed aggregation values shared across all parts.
        :return: A product circuit over the grounded distributions of all parts.
        """
        result = ProbabilisticCircuit()
        root = ProductUnit(probabilistic_circuit=result)
        for index, part in enumerate(parts_to_ground):
            part_circuit = self._ground_part_circuit(
                part, aggregation_statistics, index
            )
            part_root_index = part_circuit.root.index
            node_index_map = result.mount(part_circuit.root)
            root.add_subcircuit(node_index_map[part_root_index])
        return result


@dataclass
class RelationalProbabilisticCircuit:
    """
    A probabilistic circuit that jointly models a class and its relational structure.

    When the training instances have mixed concrete types (joined-table inheritance),
    this circuit automatically becomes polymorphic: it fits one sub-circuit per
    concrete type and combines them as a mixture at grounding time.
    """

    class_: Type
    """
    The domain class whose instances this distribution models.
    """

    class_probabilistic_circuit: Optional[ProbabilisticCircuit] = None
    """
    The fitted joint distribution over the class's scalar attributes and aggregation
    statistics, populated by ``fit``.  ``None`` for polymorphic circuits (those with
    non-empty :attr:`sub_type_circuits`).
    """

    exchangeable_distribution_templates: dict[str, ExchangeableDistributionTemplate] = (
        field(default_factory=dict)
    )
    """
    Mapping from each exchangeable-part field name to its fitted
    ``ExchangeableDistributionTemplate``.
    """

    n_samples: int = field(default=100)
    """
    Number of Monte Carlo samples drawn from the aggregation statistics posterior when
    grounding an exchangeable distribution template.

    A larger value gives a more accurate integral approximation at the cost of a larger
    grounded circuit.  Set to ``1`` to recover near-deterministic behaviour when all
    aggregation statistics are fully observed.
    """

    sub_type_circuits: dict[Type, RelationalProbabilisticCircuit] = field(
        default_factory=dict
    )
    """
    Per-concrete-type sub-circuits populated when training instances have mixed
    concrete types (polymorphism via joined-table inheritance).  Empty for
    monomorphic circuits.
    """

    log_type_weights: dict[Type, float] = field(default_factory=dict)
    """
    Log prior probability of each concrete type, estimated from training-set
    frequencies.  Non-empty exactly when :attr:`sub_type_circuits` is non-empty.
    """

    schema_information: Optional[DataAccessObjectSchema] = field(
        init=False, default=None
    )
    """
    The :class:`~krrood.ormatic.data_access_objects.dao.DataAccessObjectSchema` describing
    the DAO class's columns and relationships.
    """

    feature_extractor: Optional[FeatureExtractor] = field(init=False, default=None)
    """
    Feature extractor built from the training instances.
    """

    @staticmethod
    def _build_class_dataframe(
        feature_extractor: FeatureExtractor,
        instances: list[DataAccessObject],
        dataframe_from_parent: Optional[pd.DataFrame],
    ) -> pd.DataFrame:
        """
        Build the preprocessed dataframe used to fit the class-level JPT.

        :param feature_extractor: The extractor used to create and preprocess the dataframe.
        :param instances: Training instances to extract features from.
        :param dataframe_from_parent: Pre-built dataframe from a parent fit call, or ``None``.
        :return: A preprocessed, column-sorted dataframe ready for JPT training.
        """
        if dataframe_from_parent is not None:
            return dataframe_from_parent
        dataframe = feature_extractor.create_dataframe(instances)
        dataframe = feature_extractor.preprocess_dataframe(dataframe)
        return dataframe.sort_index(axis=1)

    def _build_child_joint_dataframe(
        self,
        exchangeable_part: str,
        instances: list[DataAccessObject],
        aggregation_indices: list[int],
        aggregation_names: list[str],
        child_feature_extractor: FeatureExtractor,
        child_class_prefix: str,
    ) -> pd.DataFrame:
        """
        Build a dataframe combining aggregation statistics with per-child-object attributes.

        Each row corresponds to one child object and contains the parent instance's
        aggregation values followed by all child features (including nested unique-part
        attributes). Column names strip the child class prefix so that, after variable
        renaming, they align with the krrood access-path convention.

        :param exchangeable_part: Field name of the one-to-many relation on each instance.
        :param instances: Training instances from which rows are generated.
        :param aggregation_indices: Positions of aggregation features in the feature vector.
        :param aggregation_names: Column names for the aggregation portion of each row.
        :param child_feature_extractor: Feature extractor built from the child instances.
        :param child_class_prefix: Class name prefix to strip from child feature names
        :return: A dataframe with one row per child object across all instances.
        """
        rows = []
        for instance in instances:
            feature_vector = self.feature_extractor.apply_mapping(instance)
            aggregation_row = [feature_vector[index] for index in aggregation_indices]
            for association in getattr(instance, exchangeable_part):
                child_features = child_feature_extractor.apply_mapping(
                    association.target
                )
                rows.append(aggregation_row + child_features)
        full_names = [f._name_ for f in child_feature_extractor.features]
        short_names = [
            (
                name[len(child_class_prefix) :]
                if name.startswith(child_class_prefix)
                else name
            )
            for name in full_names
        ]
        return pd.DataFrame(columns=aggregation_names + short_names, data=rows)

    def _build_polymorphic_child_joint_dataframe(
        self,
        exchangeable_part: str,
        instances: list[DataAccessObject],
        aggregation_indices: list[int],
        aggregation_names: list[str],
        child_partition: dict[Type, list[DataAccessObject]],
    ) -> pd.DataFrame:
        """
        Build a combined child dataframe over polymorphic child objects.

        Constructs one dataframe per concrete child type (using a per-type
        ``FeatureExtractor``), prepends the parent's aggregation statistics to
        each row, strips the concrete-class prefix from feature column names, and
        concatenates all per-type dataframes with an outer join so that columns
        belonging to one type receive ``NaN`` for instances of other types.

        :param exchangeable_part: Field name of the one-to-many relation on each instance.
        :param instances: Training instances whose children are used to build the dataframe.
        :param aggregation_indices: Positions of aggregation features in the feature vector.
        :param aggregation_names: Column names for the aggregation portion of each row.
        :param child_partition: Concrete-type partition of all child instances.
        :return: A combined dataframe with one row per child object across all instances.
        """
        per_type_fes: dict[Type, FeatureExtractor] = {}
        per_type_prefixes: dict[Type, str] = {}
        for concrete_type, typed_children in child_partition.items():
            per_type_fes[concrete_type] = FeatureExtractor.from_instances(typed_children)
            domain_name = type(
                typed_children[0].from_dao(FromDataAccessObjectState())
            ).__name__
            per_type_prefixes[concrete_type] = f"{domain_name}."

        per_type_rows: dict[Type, list] = {ct: [] for ct in child_partition}
        for parent_instance in instances:
            feature_vector = self.feature_extractor.apply_mapping(parent_instance)
            aggregation_row = [feature_vector[index] for index in aggregation_indices]
            for association in getattr(parent_instance, exchangeable_part):
                child = association.target
                concrete_type = type(child).original_class()
                child_features = per_type_fes[concrete_type].apply_mapping(child)
                per_type_rows[concrete_type].append(aggregation_row + child_features)

        per_type_dfs = []
        for concrete_type, rows in per_type_rows.items():
            child_fe = per_type_fes[concrete_type]
            prefix = per_type_prefixes[concrete_type]
            full_names = [f._name_ for f in child_fe.features]
            short_names = [
                name[len(prefix):] if name.startswith(prefix) else name
                for name in full_names
            ]
            per_type_dfs.append(
                pd.DataFrame(columns=aggregation_names + short_names, data=rows)
            )
        return pd.concat(per_type_dfs, join="outer", ignore_index=True)

    def _fit_exchangeable_part(
        self,
        exchangeable_part: str,
        instances: list[DataAccessObject],
    ) -> ExchangeableDistributionTemplate:
        """
        Fit an ``ExchangeableDistributionTemplate`` for one exchangeable part.

        Builds a joint dataframe that pairs each child object's attributes with the
        parent's aggregation statistics, infers which variables are latent (the
        aggregation columns), and recursively fits a ``RelationalProbabilisticCircuit``
        on the child instances using that dataframe.

        When the children span multiple concrete types (joined-table inheritance),
        the dataframe is built per type and concatenated with an outer join so that
        type-specific columns receive ``NaN`` for instances of other types.

        :param exchangeable_part: Field name of the one-to-many relation on each instance.
        :param instances: Training instances whose children are used to fit the template.
        :return: A fitted ``ExchangeableDistributionTemplate`` for the given part.
        """
        aggregation_functions = self.feature_extractor.exchangeable_features[
            exchangeable_part
        ]
        aggregation_indices = [
            next(
                index
                for index, feature in enumerate(self.feature_extractor.features)
                if feature is aggregation_function
            )
            for aggregation_function in aggregation_functions
        ]
        aggregation_names = [function._name_ for function in aggregation_functions]

        child_instances = [
            association.target
            for association in itertools.chain.from_iterable(
                getattr(instance, exchangeable_part) for instance in instances
            )
        ]
        child_partition = self._partition_by_concrete_type(child_instances)
        if len(child_partition) > 1:
            child_dataframe = self._build_polymorphic_child_joint_dataframe(
                exchangeable_part,
                instances,
                aggregation_indices,
                aggregation_names,
                child_partition,
            )
        else:
            child_feature_extractor = FeatureExtractor.from_instances(child_instances)
            child_class_name = type(
                child_instances[0].from_dao(FromDataAccessObjectState())
            ).__name__
            child_class_prefix = f"{child_class_name}."
            child_dataframe = self._build_child_joint_dataframe(
                exchangeable_part,
                instances,
                aggregation_indices,
                aggregation_names,
                child_feature_extractor,
                child_class_prefix,
            )
        child_type = type(child_instances[0])
        latent_variables = [
            inferred.variable
            for inferred in infer_variables_from_dataframe(child_dataframe)
            if inferred.variable.name in aggregation_names
        ]
        template = ExchangeableDistributionTemplate(
            RelationalProbabilisticCircuit(child_type),
            latent_variables,
        )
        template.template_distribution.fit(
            child_instances, dataframe_from_parent=child_dataframe
        )
        return template

    def _domain_class(self) -> Type:
        """
        Return the domain class this circuit models.

        :attr:`class_` may be a DAO class (when the circuit is created from an
        exchangeable-part fit) or a domain class directly (when constructed
        explicitly).  This method normalises both cases to the domain class.

        :return: The domain class of this circuit.
        """
        if hasattr(self.class_, "original_class"):
            return self.class_.original_class()
        return self.class_

    @staticmethod
    def _partition_by_concrete_type(
        instances: list[DataAccessObject],
    ) -> dict[Type, list[DataAccessObject]]:
        """
        Partition DAO instances by the domain type of each concrete instance.

        Uses the DAO class's :meth:`original_class` to recover the domain type;
        this correctly resolves joined-table inheritance subtypes.

        :param instances: The DAO instances to partition.
        :return: A mapping from each concrete domain type to its instances.
        """
        partition: dict[Type, list[DataAccessObject]] = {}
        for instance in instances:
            domain_type = type(instance).original_class()
            if domain_type not in partition:
                partition[domain_type] = []
            partition[domain_type].append(instance)
        return partition

    def _fit_polymorphic(
        self,
        partition: dict[Type, list[DataAccessObject]],
        combined_dataframe: Optional[pd.DataFrame] = None,
    ) -> None:
        """
        Fit one sub-circuit per concrete type, then fit a unified class circuit.

        Sub-circuits are stored in :attr:`sub_type_circuits` for metadata (type
        weights, sub-circuit access).  A single ``JointProbabilityTree`` is also
        fitted over the union of all concrete types' variable spaces and stored in
        :attr:`class_probabilistic_circuit`.  Type-specific variables receive
        ``NaN`` for instances of other types; JPT handles these gracefully.

        :param partition: Mapping from each concrete domain type to its DAO instances.
        :param combined_dataframe: Pre-built combined dataframe (aggregation columns
            included) supplied by a parent ``_fit_exchangeable_part`` call.  When
            ``None``, the combined dataframe is constructed from per-type feature
            extractors (no aggregation columns).
        """
        total = sum(len(v) for v in partition.values())
        log_total = math.log(total)
        per_type_dfs: list[pd.DataFrame] = []
        for concrete_type, sub_instances in partition.items():
            sub_rpc = RelationalProbabilisticCircuit(
                concrete_type, n_samples=self.n_samples
            )
            sub_rpc.fit(sub_instances)
            self.sub_type_circuits[concrete_type] = sub_rpc
            self.log_type_weights[concrete_type] = (
                math.log(len(sub_instances)) - log_total
            )
            if combined_dataframe is None and sub_rpc.feature_extractor is not None:
                df = sub_rpc.feature_extractor.create_dataframe(sub_instances)
                df = sub_rpc.feature_extractor.preprocess_dataframe(df)
                df = df.sort_index(axis=1)
                class_prefix = f"{concrete_type.__name__}."
                df.columns = [
                    col[len(class_prefix):] if col.startswith(class_prefix) else col
                    for col in df.columns
                ]
                per_type_dfs.append(df)
        training_df: Optional[pd.DataFrame]
        if combined_dataframe is not None:
            training_df = combined_dataframe
        elif per_type_dfs:
            training_df = pd.concat(per_type_dfs, join="outer", ignore_index=True)
        else:
            training_df = None
        if training_df is not None:
            variables = infer_variables_from_dataframe(training_df)
            self.class_probabilistic_circuit = JointProbabilityTree(
                annotated_variables=variables
            ).fit(training_df)

    def fit(
        self,
        instances: list[DataAccessObject],
        dataframe_from_parent: Optional[pd.DataFrame] = None,
    ):
        """
        Fit the relational probabilistic circuit from a list of DAO instances.

        Builds a ``FeatureExtractor``, trains a ``JointProbabilityTree`` on the
        class-level features, and then recursively fits one
        ``ExchangeableDistributionTemplate`` per exchangeable part discovered in
        the schema.

        :param instances: Training instances; all must share the same DAO class.
        :param dataframe_from_parent: Pre-built dataframe supplied by a parent
            ``_fit_exchangeable_part`` call.  When provided, feature extraction
            and preprocessing are skipped.
        :return: ``self``, to allow chaining.
        """
        partition = self._partition_by_concrete_type(instances)
        domain_class = self._domain_class()
        if len(partition) > 1 or (
            len(partition) == 1 and next(iter(partition)) != domain_class
        ):
            self._fit_polymorphic(partition, combined_dataframe=dataframe_from_parent)
            return self

        self.feature_extractor = FeatureExtractor.from_instances(instances)
        class_dataframe = self._build_class_dataframe(
            self.feature_extractor, instances, dataframe_from_parent
        )
        variables = infer_variables_from_dataframe(class_dataframe)
        self.class_probabilistic_circuit = JointProbabilityTree(
            annotated_variables=variables
        ).fit(class_dataframe)
        self.schema_information = get_dao_schema(type(instances[0]))
        for collection_relationship in self.schema_information.collection_relationships:
            exchangeable_part = collection_relationship.key
            if exchangeable_part not in self.feature_extractor.exchangeable_features:
                continue
            self.exchangeable_distribution_templates[exchangeable_part] = (
                self._fit_exchangeable_part(exchangeable_part, instances)
            )
        return self

    def _condition_class_circuit(
        self,
        circuit: ProbabilisticCircuit,
        aggregation_statistics: dict[Variable, Any],
        latent_variables: list[Variable],
    ) -> tuple[ProbabilisticCircuit, list[ProductUnit]]:
        """
        Condition the class circuit on aggregation statistics.

        :param circuit: The current working copy of the class circuit.
        :param aggregation_statistics: Observed aggregation values to condition on.
        :param latent_variables: Variables that link the class circuit to the
            exchangeable distribution template.
        :return: The conditioned circuit and the surviving product
            nodes that will be extended with the grounded exchangeable distribution.
        """
        product_nodes_to_extend = find_lowest_product_nodes_that_model_variables(
            circuit, SortedSet(latent_variables)
        )
        conditioning_result, _ = circuit.log_conditional_in_place(
            aggregation_statistics
        )
        if conditioning_result is None:
            circuit = self.class_probabilistic_circuit.__deepcopy__()
            product_nodes_to_extend = find_lowest_product_nodes_that_model_variables(
                circuit, SortedSet(latent_variables)
            )
        if len(circuit.nodes()) == 0:
            raise ValueError("The grounding of the class failed.")
        surviving_product_nodes = [
            node for node in product_nodes_to_extend if node.index is not None
        ]
        return circuit, surviving_product_nodes

    def _sample_aggregation_statistics(
        self,
        circuit: ProbabilisticCircuit,
        available_statistics: dict[Variable, Any],
        latent_variables: list[Variable],
    ) -> list[dict[Variable, Any]]:
        """
        Draw samples of aggregation statistics from the class circuit prior.

        Marginalises the circuit (before any conditioning) to the aggregation (latent)
        variables to obtain ``P(agg_stats)``, conditions on the subset of statistics
        that are already known (``available_statistics``), then draws :attr:`n_samples`
        samples of the remaining unknown statistics.  Each returned dict contains the
        full set of latent variables: known ones take their observed value in every
        sample; unknown ones are drawn from the posterior
        ``P(unknown | known, class context)``.

        This method must be called on the **unconditioned** circuit because
        :meth:`~probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit.log_conditional_in_place`
        marginalises out conditioned variables, making them unavailable for sampling
        afterwards.

        :param circuit: The unconditioned deep-copy of the class circuit.
        :param available_statistics: Aggregation statistics that are computable from the
            query; may be empty or partial.
        :param latent_variables: All aggregation statistic variables for this part.
        :return: One dict per sample mapping every latent variable to a value.
        """
        agg_marginal = circuit.marginal(latent_variables)
        if agg_marginal is None:
            return [dict(available_statistics)] * self.n_samples

        available_in_marginal = {
            variable: value
            for variable, value in available_statistics.items()
            if variable in agg_marginal.variables
        }
        if available_in_marginal:
            conditioned, _ = agg_marginal.log_conditional_in_place(available_in_marginal)
            if conditioned is None:
                agg_marginal = circuit.marginal(latent_variables)

        samples_array = agg_marginal.sample(self.n_samples)
        variable_to_index = agg_marginal.variable_to_index_map
        result = []
        for i in range(self.n_samples):
            sample = dict(available_statistics)
            for variable, index in variable_to_index.items():
                sample[variable] = samples_array[i, index]
            result.append(sample)
        return result

    def _build_grounded_mixture(
        self,
        template: ExchangeableDistributionTemplate,
        parts_to_ground: list,
        aggregation_statistic_samples: list[dict[Variable, Any]],
    ) -> ProbabilisticCircuit:
        """
        Ground the template once per sample and combine the results into a uniform mixture.

        Each sample produces one grounded circuit ``P(x | sᵢ)``.  All circuits are
        combined in a :class:`~probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.SumUnit`
        with equal log-weight ``-log(n_samples)``, approximating the integral
        ``∫ P(x | s) P(s | class context) ds``.

        :param template: The exchangeable distribution template to ground.
        :param parts_to_ground: The query parts, one per child object in the relation.
        :param aggregation_statistic_samples: Sampled aggregation statistics, one dict per sample.
        :return: A mixture circuit over the grounded distributions.
        """
        mixture = ProbabilisticCircuit()
        mixture_root = SumUnit(probabilistic_circuit=mixture)
        log_weight = -math.log(len(aggregation_statistic_samples))
        for aggregation_statistics in aggregation_statistic_samples:
            grounded = template.ground(parts_to_ground, aggregation_statistics)
            grounded_root_index = grounded.root.index
            node_index_map = mixture.mount(grounded.root)
            mixture_root.add_subcircuit(
                node_index_map[grounded_root_index], log_weight=log_weight
            )
        return mixture

    def _ground_polymorphic(self, query: Match) -> ProbabilisticCircuit:
        """
        Ground by mixing per-concrete-type sub-circuits with learned type weights.

        Produces a :class:`~probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.SumUnit`
        whose components are the grounded circuits of all concrete subtypes, weighted
        by their empirical log prior ``P(type = Cᵢ)``.

        Each sub-circuit is grounded with the same query as the parent; concrete
        types are assumed to share all exchangeable parts of the base type and
        may only add scalar attributes.

        :param query: The underspecified, resolved query instance.
        :return: A mixture circuit over the grounded concrete-type distributions.
        :raises CircuitNotFittedError: If no sub-type circuits have been fitted yet.
        """
        if not self.sub_type_circuits:
            raise CircuitNotFittedError(self.class_)
        mixture = ProbabilisticCircuit()
        mixture_root = SumUnit(probabilistic_circuit=mixture)
        for concrete_type, sub_rpc in self.sub_type_circuits.items():
            sub_circuit = sub_rpc.ground(query)
            sub_root_index = sub_circuit.root.index
            node_index_map = mixture.mount(sub_circuit.root)
            mixture_root.add_subcircuit(
                node_index_map[sub_root_index],
                log_weight=self.log_type_weights[concrete_type],
            )
        return mixture

    def ground(self, query: Match) -> ProbabilisticCircuit:
        """Ground the relational circuit for a specific query.

        Starting from a deep copy of ``class_probabilistic_circuit``, each
        exchangeable part's template is grounded for the objects specified in the
        query and attached to the conditioning product nodes of the class circuit.

        :param query: An underspecified, resolved query instance whose structure
            determines which parts are grounded and how many child objects each
            exchangeable relation contains.
        :return: A concrete ``ProbabilisticCircuit`` over all variables implied
            by the query.
        :raises CircuitNotFittedError: If ``ground`` is called before ``fit``.
        """
        if self.class_probabilistic_circuit is None:
            raise CircuitNotFittedError(self.class_)
        circuit = self.class_probabilistic_circuit.__deepcopy__()
        instance = query.construct_instance()
        for (
            exchangeable_part_name,
            template,
        ) in self.exchangeable_distribution_templates.items():
            aggregation_statistics = compute_aggregation_statistics(
                instance,
                self.feature_extractor.exchangeable_features[exchangeable_part_name],
                template.latent_variables,
            )
            samples = self._sample_aggregation_statistics(
                circuit, aggregation_statistics, template.latent_variables
            )
            circuit, product_nodes_to_extend = self._condition_class_circuit(
                circuit, aggregation_statistics, template.latent_variables
            )
            for product_node in product_nodes_to_extend:
                mixture = self._build_grounded_mixture(
                    template, query.kwargs[exchangeable_part_name], samples
                )
                mixture_root_index = mixture.root.index
                node_index_map = circuit.mount(mixture.root)
                product_node.add_subcircuit(node_index_map[mixture_root_index])
        return circuit
