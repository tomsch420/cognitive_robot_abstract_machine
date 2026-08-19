"""
Tests for support/confidence scoring of ``CandidateRuleBody``.
"""

import pytest

from krrood.entity_query_language.factories import variable
from krrood.entity_query_language.rule_mining.candidate_rule import CandidateRuleBody
from krrood.entity_query_language.rule_mining.exceptions import EmptyRuleBodyError
from krrood.entity_query_language.rule_mining.scoring import RuleScore, ScoreThresholds

from ...dataset.rule_mining_fixture import Handle
from .test_candidate_rule import containers_and_handles

# %% score


def test_score_after_single_dangling_atom():
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
    body.extend_with_related_variable(head, "container")

    score = body.score()

    assert score == RuleScore(support=3, confidence=0.75)


def test_score_after_closing_atom_on_top_of_two_prior_atoms():
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
    other_head = variable(Handle, domain=[world["handle_1"]])
    body = CandidateRuleBody(head_variable=head, open_variables=[head, other_head])
    body.extend_with_related_variable(head, "container")
    container_variable = body.open_variables[-1]
    body.constrain_variable_to_value(container_variable, world["container_1"])
    body.close_by_equating_variables(head, other_head)

    score = body.score()

    assert score == RuleScore(support=1, confidence=0.5)


def test_score_with_prior_support_zero_has_zero_confidence():
    world = containers_and_handles()
    head = variable(
        Handle, domain=[world["handle_1"], world["handle_2"], world["handle_3"]]
    )
    body = CandidateRuleBody(head_variable=head)
    body.constrain_variable_to_value(head, world["empty_container"])

    score = body.score()

    assert score == RuleScore(support=0, confidence=0.0)


def test_score_raises_on_empty_rule_body():
    world = containers_and_handles()
    head = variable(Handle, domain=[world["handle_1"]])
    body = CandidateRuleBody(head_variable=head)

    with pytest.raises(EmptyRuleBodyError):
        body.score()


# %% RuleScore.meets


def test_rule_score_meets_thresholds_when_both_are_cleared():
    score = RuleScore(support=3, confidence=0.75)
    thresholds = ScoreThresholds(minimum_support=3, minimum_confidence=0.5)

    assert score.meets(thresholds) is True


def test_rule_score_does_not_meet_thresholds_when_confidence_is_too_low():
    score = RuleScore(support=3, confidence=0.4)
    thresholds = ScoreThresholds(minimum_support=3, minimum_confidence=0.5)

    assert score.meets(thresholds) is False
