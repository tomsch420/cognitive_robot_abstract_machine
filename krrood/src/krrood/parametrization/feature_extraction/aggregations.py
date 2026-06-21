from __future__ import annotations

import inspect
import warnings
from dataclasses import dataclass
from functools import lru_cache
from typing_extensions import Callable, Optional, Type, TypeVar, Any

from krrood.entity_query_language.core.mapped_variable import MappedVariable
from random_events.variable import Symbolic, Variable
from krrood.entity_query_language.factories import variable
from krrood.patterns.subclass_safe_generic import SubClassSafeGeneric
from krrood.utils import T, recursive_subclasses


def aggregation_statistic(field_ref: list[Any]) -> Callable[[Callable], Callable]:
    """
    Marks a method as an aggregation statistic for the named exchangeable-part field.

    The field reference must be created via
    :func:`~krrood.entity_query_language.factories.variable`:
    ``variable(OwnerType, None).field_name``.  Its ``_attribute_name_`` property
    identifies which exchangeable-part field this statistic aggregates over.

    :param field_ref: A typed :class:`~krrood.entity_query_language.core.mapped_variable.Attribute`
        produced by attribute access on a symbolic variable.
    """

    def decorator(func: Callable) -> Callable:
        func._field_reference = field_ref
        func.is_aggregation_statistic = True
        return func

    return decorator


@lru_cache(maxsize=None)
def get_aggregation_class(owner: Type) -> Optional[Type[AggregationStatistic]]:
    """
    Returns the :class:`AggregationStatistic` subclass whose generic ``T`` matches ``owner``.

    Modelled on :func:`~krrood.ormatic.data_access_objects.helper.get_dao_class`: discovery
    is implicit — any concrete subclass of :class:`AggregationStatistic` that binds ``T``
    to a domain class is automatically found here without explicit registration.

    :param owner: The domain class to look up.
    :return: The matching subclass, or ``None`` if none has been defined.
    """
    for subclass in recursive_subclasses(AggregationStatistic):
        if subclass.get_generic_type() == owner:
            return subclass
    return None


@dataclass
class AggregationStatistic(SubClassSafeGeneric[T]):
    """
    Base class for aggregation statistics over a domain object's exchangeable-part fields.

    Subclasses bind ``T`` to a concrete owner type and declare one or more methods, each
    annotated with :func:`aggregation_statistic`.  Discovery happens automatically via
    :func:`get_aggregation_class` — no explicit registration is required.

    .. note::
        Each owner class may have at most one ``AggregationStatistic`` subclass, which must
        handle all of its exchangeable-part fields.  Shared logic across owner types should
        be extracted to an intermediate abstract subclass whose concrete children each bind
        their own ``T``.
    """

    instance: T
    """
    The owner domain object whose exchangeable-part fields are aggregated.
    """

    @property
    def aggregation_features(self) -> list[Callable]:
        """
        All methods on this class marked with :func:`aggregation_statistic`.

        :return: The marked callable methods, sorted alphabetically by name.
        """
        return [
            func
            for _, func in inspect.getmembers(
                self.__class__, predicate=inspect.isfunction
            )
            if hasattr(func, "is_aggregation_statistic")
        ]

    def symbolic_aggregation_features_for(
        self, field_name: str
    ) -> list[MappedVariable]:
        """
        Symbolic variables for statistic methods that aggregate the named exchangeable-part field.

        :param field_name: The field name on the owner to filter by.
        :return: One :class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable`
            per matching statistic method, in alphabetical order.
        """
        aggregation_variable = variable(type(self), [])
        return [
            getattr(aggregation_variable, func.__name__)()
            for func in self.aggregation_features
            if func._field_reference._attribute_name_ == field_name
        ]

    def apply_mapping_for(self, field_name: str) -> list:
        """
        Evaluates every statistic for the named field against this instance.

        :param field_name: The exchangeable-part field name to evaluate.
        :return: One concrete value per matching statistic method, in alphabetical order.
        """
        return [
            feature.apply_mapping_on_external_root(self)
            for feature in self.symbolic_aggregation_features_for(field_name)
        ]


def compute_aggregation_statistics(
    domain_object,
    feature_functions: list[MappedVariable],
    latent_variables: list[Variable],
) -> dict[Variable, Any]:
    """
    Evaluate aggregation feature functions against a domain object and map results to latent variables.

    Each feature function is evaluated only if its name matches a latent variable; values outside
    the training domain of their variable are silently skipped to avoid impossible conditioning events.

    :param domain_object: The domain object whose aggregation statistics are computed.
    :param feature_functions: Symbolic feature functions for one exchangeable-part field.
    :param latent_variables: Latent variables that define which statistics are relevant.
    :return: A mapping from matched latent variables to their observed values.
    """
    latent_variable_by_name = {latent_variable.name: latent_variable for latent_variable in latent_variables}
    aggregation_cls = get_aggregation_class(type(domain_object))
    if aggregation_cls is None:
        return {}
    aggregation_instance = aggregation_cls(instance=domain_object)
    statistics = {}
    for feature_function in feature_functions:
        feature_name = feature_function._name_
        if feature_name not in latent_variable_by_name:
            continue
        value = feature_function.apply_mapping_on_external_root(aggregation_instance)
        latent_variable = latent_variable_by_name[feature_name]
        if isinstance(latent_variable, Symbolic):
            if not any(elem.element == value for elem in latent_variable.domain.simple_sets):
                warnings.warn(
                    f"Value {value!r} for aggregation feature '{feature_name}' is outside the "
                    f"training domain of variable '{latent_variable.name}'. Skipping.",
                    stacklevel=2,
                )
                continue
        statistics[latent_variable] = value
    return statistics
