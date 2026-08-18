"""
Tests for the dangling-atom, instantiated-atom, and closing-atom refinement operators on
``CandidateRuleBody``.
"""

import pytest

from krrood.entity_query_language.factories import variable
from krrood.entity_query_language.rule_mining.candidate_rule import (
    CandidateRuleBody,
    candidate_attribute_names,
)
from krrood.entity_query_language.rule_mining.exceptions import (
    IncompatibleVariableTypesError,
    UnknownAttributeError,
)

from ...dataset.rule_mining_fixture import Container, Handle

# %% fixture data


def containers_and_handles():
    """
    :return: A small world of containers and handles keyed by name: two non-empty
        containers (one with two handles, one with one), an empty container, and an
        orphan handle with no container.
    """
    container_1 = Container(name="Container1")
    container_2 = Container(name="Container2")
    empty_container = Container(name="EmptyContainer")
    handle_1 = Handle(name="Handle1", container=container_1)
    handle_2 = Handle(name="Handle2", container=container_1)
    handle_3 = Handle(name="Handle3", container=container_2)
    orphan_handle = Handle(name="OrphanHandle", container=None)
    container_1.handles = [handle_1, handle_2]
    container_2.handles = [handle_3]
    return {
        "container_1": container_1,
        "container_2": container_2,
        "empty_container": empty_container,
        "handle_1": handle_1,
        "handle_2": handle_2,
        "handle_3": handle_3,
        "orphan_handle": orphan_handle,
    }


# %% extend_with_related_variable


def test_extend_with_related_variable_over_scalar_field_joins_expected_pairs():
    world = containers_and_handles()
    head = variable(
        Handle,
        domain=[
            world["handle_1"],
            world["handle_2"],
            world["handle_3"],
            world["orphan_handle"],
        ],
    )
    body = CandidateRuleBody(head_variable=head)

    extended = body.extend_with_related_variable(head, "container")
    results = list(extended.to_query().evaluate())

    assert [(handle, handle.container) for handle in results] == [
        (world["handle_1"], world["container_1"]),
        (world["handle_2"], world["container_1"]),
        (world["handle_3"], world["container_2"]),
    ]


def test_extend_with_related_variable_over_collection_field_flattens_per_element():
    world = containers_and_handles()
    head = variable(
        Container,
        domain=[
            world["container_1"],
            world["container_2"],
            world["empty_container"],
        ],
    )
    body = CandidateRuleBody(head_variable=head)

    extended = body.extend_with_related_variable(head, "handles")
    results = list(extended.to_query().evaluate())

    assert results == [
        world["container_1"],
        world["container_1"],
        world["container_2"],
    ]


def test_extend_with_related_variable_raises_on_unknown_attribute():
    world = containers_and_handles()
    head = variable(Handle, domain=[world["handle_1"]])
    body = CandidateRuleBody(head_variable=head)

    with pytest.raises(UnknownAttributeError):
        body.extend_with_related_variable(head, "nonexistent_attribute")


# %% constrain_variable_to_value


def test_constrain_variable_to_value_restricts_to_matching_value():
    world = containers_and_handles()
    head = variable(
        Handle, domain=[world["handle_1"], world["handle_2"], world["handle_3"]]
    )
    body = CandidateRuleBody(head_variable=head)

    constrained = body.constrain_variable_to_value(head, world["handle_2"])
    results = list(constrained.to_query().evaluate())

    assert results == [world["handle_2"]]


# %% close_by_equating_variables


def test_close_by_equating_variables_restricts_to_matching_bindings_and_closes_both():
    world = containers_and_handles()
    variable_a = variable(Handle, domain=[world["handle_1"], world["handle_2"]])
    variable_b = variable(Handle, domain=[world["handle_1"], world["handle_3"]])
    body = CandidateRuleBody(
        head_variable=variable_a, open_variables=(variable_a, variable_b)
    )

    closed = body.close_by_equating_variables(variable_a, variable_b)
    results = list(closed.to_query().evaluate())

    assert results == [world["handle_1"]]
    assert closed.open_variables == ()


def test_close_by_equating_variables_raises_on_incompatible_types():
    world = containers_and_handles()
    container_variable = variable(Container, domain=[world["container_1"]])
    handle_variable = variable(Handle, domain=[world["handle_1"]])
    body = CandidateRuleBody(
        head_variable=container_variable,
        open_variables=(container_variable, handle_variable),
    )

    with pytest.raises(IncompatibleVariableTypesError):
        body.close_by_equating_variables(container_variable, handle_variable)


# %% candidate_attribute_names


def test_candidate_attribute_names_lists_declared_fields():
    assert candidate_attribute_names(Container) == ["name", "handles"]


# %% immutability


def test_extension_methods_return_new_body_leaving_original_unaffected():
    world = containers_and_handles()
    head = variable(Handle, domain=[world["handle_1"], world["handle_2"]])
    original = CandidateRuleBody(head_variable=head, open_variables=(head,))

    original.extend_with_related_variable(head, "container")
    original.constrain_variable_to_value(head, world["handle_1"])

    assert original.open_variables == (head,)
    assert original.conditions == ()
