from __future__ import annotations

import enum
import itertools
from collections import defaultdict, deque
from dataclasses import dataclass, field

import pandas as pd
import sqlalchemy
from sqlalchemy.orm import MANYTOONE, ONETOMANY
from typing_extensions import TYPE_CHECKING, Any, Type
from krrood.entity_query_language.core.mapped_variable import MappedVariable
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.factories import variable
from krrood.ormatic.data_access_objects.from_dao import FromDataAccessObjectState
from krrood.ormatic.utils import get_python_type_from_sqlalchemy_column, is_data_column
from krrood.parametrization.feature_extraction.aggregations import get_aggregation_class
from random_events.variable import compatible_types

if TYPE_CHECKING:
    from krrood.ormatic.data_access_objects.dao import DataAccessObject


@dataclass
class ExtractedFeatures:
    """
    The result of traversing a DAO object graph for features.
    """

    features: list[MappedVariable]
    """
    Symbolic variables for every extractable feature, in traversal order.
    """

    exchangeable_features: dict[str, list[MappedVariable]]
    """
    Mapping from each exchangeable-part field name to its aggregation variables.
    """


@dataclass
class FeatureExtractor:
    """
    Extracts symbolic features from DAO instances, including scalar attributes,
    unique-part sub-trees, and aggregation statistics over exchangeable parts.

    Prefer ``FeatureExtractor.from_instances`` for construction; the direct
    constructor is for cases where the feature list is already known.
    """

    features: list[MappedVariable]
    """
    Symbolic variables representing every extractable feature, in traversal order.
    """

    exchangeable_features: dict[str, list[MappedVariable]] = field(
        default_factory=lambda: defaultdict(list), init=False
    )
    """
    Mapping from each exchangeable-part field name to its discovered aggregation variables.
    """

    @classmethod
    def from_instances(cls, instances: list[DataAccessObject]) -> FeatureExtractor:
        """
        Create a new feature extractor from the given instances.

        Exchangeable parts whose domain class has no entry in
        :class:`~krrood.parametrization.feature_extraction.aggregations.AggregationRegistry`
        are silently skipped; the remaining scalar and unique-part features are still extracted.

        :param instances: The instances to create the feature extractor from.
        :return: A new feature extractor.
        """
        if not instances:
            raise ValueError("No instances provided")

        dao_state = FromDataAccessObjectState()
        first_instance = instances[0]
        domain_object = first_instance.from_dao(dao_state)

        root = variable(type(domain_object), [])
        extracted = cls._extract_features(first_instance, root)
        extractor = cls(features=extracted.features)
        extractor.exchangeable_features = extracted.exchangeable_features
        return extractor

    @staticmethod
    def _extract_features(
        example_instance: DataAccessObject, symbolic_root: Variable
    ) -> ExtractedFeatures:
        """
        Traverses the DAO object graph breadth-first and collects all features.

        :param example_instance: A representative DAO instance that defines the schema.
        :param symbolic_root: The root symbolic variable for the traversal.
        :return: The discovered scalar features and per-relation aggregation features.
        """
        result = []
        seen = set()
        exchangeable_features = defaultdict(list)
        queue = deque()
        queue.append((example_instance, symbolic_root))

        while queue:
            current_instance, current_symbolic = queue.popleft()

            if id(current_instance) in seen:
                continue
            seen.add(id(current_instance))

            instance_composition = EntityCompositionDescriptor(type(current_instance))

            result.extend(
                FeatureExtractor._process_attributes(
                    current_instance, current_symbolic, instance_composition.attributes
                )
            )

            exchangeable_features.update(
                FeatureExtractor._process_many_to_many(
                    current_instance, instance_composition.many_to_many_relations
                )
            )
            queue.extend(
                FeatureExtractor._process_many_to_one(
                    current_instance,
                    current_symbolic,
                    instance_composition.many_to_one_relations,
                )
            )

        result.extend(itertools.chain.from_iterable(exchangeable_features.values()))
        return ExtractedFeatures(result, exchangeable_features)

    @staticmethod
    def _process_attributes(
        instance: DataAccessObject,
        symbolic_root: Variable,
        attributes: list[sqlalchemy.Column],
    ) -> list[MappedVariable]:
        """
        Collects symbolic variables for all scalar data columns of ``instance``.

        Columns whose value is not a compatible primitive type are skipped.
        :param instance: The DAO instance to inspect.
        :param symbolic_root: The symbolic variable rooted at ``instance``.
        :param attributes: The scalar data columns of the instance's schema.
        :return: One typed ``MappedVariable`` per compatible scalar attribute.
        """
        result = []
        for attribute in attributes:
            value = getattr(instance, attribute.key)

            if not isinstance(value, compatible_types):
                continue

            symbolic_attribute = getattr(symbolic_root, attribute.name)
            symbolic_attribute._type_ = get_python_type_from_sqlalchemy_column(
                attribute
            )
            result.append(symbolic_attribute)
        return result

    @staticmethod
    def _process_many_to_one(
        instance: DataAccessObject,
        symbolic_root: Variable,
        many_to_one_relations: list[str],
    ) -> deque[Any]:
        """
        Enqueues non-null many-to-one relations for further BFS traversal.

        :param instance: The DAO instance to inspect.
        :param symbolic_root: The symbolic variable rooted at ``instance``.
        :param many_to_one_relations: Field names of the instance's many-to-one relations.
        :return: ``(child_instance, child_symbolic)`` pairs ready for BFS expansion.
        """
        queue = deque()
        for part in many_to_one_relations:
            value = getattr(instance, part)

            if value is None:
                continue

            queue.append((value, getattr(symbolic_root, part)))
        return queue

    @staticmethod
    def _process_many_to_many(
        current_instance: DataAccessObject,
        many_to_many_relations: list[str],
    ) -> dict[str, list[MappedVariable]]:
        """
        Collects aggregation statistic variables for all one-to-many relations of ``current_instance``.

        :param current_instance: The DAO instance to inspect.
        :param many_to_many_relations: Field names of the instance's one-to-many relations.
        :return: A mapping from each one-to-many field name to its aggregation variables.
        """
        result = defaultdict(list)
        dao_state = FromDataAccessObjectState()
        domain_object = current_instance.from_dao(dao_state)

        aggregation_cls = get_aggregation_class(type(domain_object))
        if aggregation_cls is None:
            return result

        aggregation_instance = aggregation_cls(instance=domain_object)
        for field_name in many_to_many_relations:
            if not getattr(domain_object, field_name):
                continue
            for feature in aggregation_instance.symbolic_aggregation_features_for(
                field_name
            ):
                result[field_name].append(feature)

        return result

    def apply_mapping(self, instance: DataAccessObject) -> list:
        """
        Extracts the mapped values for each feature from the given instance.
        :param instance: The instance to extract features from.
        :return: A list of mapped values.
        """
        aggregation_features = {
            aggregation
            for aggregations in self.exchangeable_features.values()
            for aggregation in aggregations
        }
        result = []
        dao_state = FromDataAccessObjectState()
        domain_object = instance.from_dao(dao_state)
        aggregation_cls = get_aggregation_class(type(domain_object))
        aggregation_instance = (
            aggregation_cls(instance=domain_object)
            if aggregation_cls is not None
            else None
        )
        for feature in self.features:
            if feature in aggregation_features:
                result.append(
                    feature.apply_mapping_on_external_root(aggregation_instance)
                )
            else:
                result.append(feature.apply_mapping_on_external_root(instance))
        return result

    def create_dataframe(self, instances: list[DataAccessObject]) -> pd.DataFrame:
        """
        Create a dataframe from the given instances.
        :param instances: The instances to create the dataframe from.
        :return: A dataframe containing the mapped values for each feature.
        """
        result = [self.apply_mapping(instance) for instance in instances]
        features_names = [feature._name_ for feature in self.features]
        return pd.DataFrame(columns=features_names, data=result)

    def preprocess_dataframe(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Preprocess the dataframe for JointProbabilityTrees by converting boolean columns to integers and enum columns to hashes.
        :param df: The dataframe to preprocess.
        :return: The dataframe in a JPT compatible format.
        """
        feature_map = dict(zip(df.columns, self.features))
        for column in df.columns:
            feature = feature_map[column]
            if feature._type_ is bool:
                df[column] = df[column].astype(int)
            elif isinstance(feature._type_, enum.EnumType):
                df[column] = df[column].apply(lambda x: hash(x))
            elif feature._type_ not in compatible_types and feature._type_ is not None:
                raise TypeError(
                    f"Unsupported type {feature._type_} for column {column}"
                )
        return df


@dataclass
class EntityCompositionDescriptor:
    """
    Describes the scalar attributes and ORM relationships of a DAO class.

    Constructed from the class's SQLAlchemy mapper; used by :class:`FeatureExtractor`
    to direct BFS traversal and aggregation discovery.
    """

    dao_class: Type[DataAccessObject] = field(init=True)
    """
    The DAO class whose SQLAlchemy mapper is inspected.
    """

    attributes: list[sqlalchemy.Column] = field(init=False, default_factory=list)
    """
    Scalar data columns of the DAO class.
    """

    many_to_one_relations: list[str] = field(init=False, default_factory=list)
    """
    Field names of many-to-one relations (FK on this table).
    """

    many_to_many_relations: list[str] = field(init=False, default_factory=list)
    """
    Field names of many-to-many relations.
    """

    def __post_init__(self):
        mapper = sqlalchemy.inspection.inspect(self.dao_class)

        for relationship in mapper.relationships:
            if relationship.direction == MANYTOONE:
                self.many_to_one_relations.append(relationship.key)
            elif relationship.direction == ONETOMANY:
                self.many_to_many_relations.append(relationship.key)
        for column in mapper.columns:
            if is_data_column(column) and column not in mapper.relationships:
                self.attributes.append(column)
