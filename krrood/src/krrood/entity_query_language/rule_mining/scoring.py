"""
Support and confidence scoring for candidate rule bodies.
"""

from __future__ import annotations

from dataclasses import dataclass

# %% score thresholds


@dataclass
class ScoreThresholds:
    """
    The minimum support and confidence a candidate rule body must clear to survive
    pruning during mining.
    """

    minimum_support: int
    """
    The lowest acceptable :attr:`RuleScore.support`.
    """

    minimum_confidence: float
    """
    The lowest acceptable :attr:`RuleScore.confidence`.
    """


# %% rule score


@dataclass
class RuleScore:
    """
    The support and confidence of a candidate rule body, as scored by
    :meth:`~krrood.entity_query_language.rule_mining.candidate_rule.CandidateRuleBody.score`.

    Reference: :cite:t:`galarraga2013amie` — support and confidence over a relational
    rule body.
    """

    support: int
    """
    The number of bindings satisfying the full rule body.
    """

    confidence: float
    """
    The fraction of bindings satisfying the rule body without its most-recently-added
    atom that also satisfy that atom.
    """

    def meets(self, thresholds: ScoreThresholds) -> bool:
        """
        Check whether this score clears both of ``thresholds``.

        :param thresholds: The minimum support and confidence to clear.
        :return: Whether both :attr:`support` and :attr:`confidence` meet or exceed
            ``thresholds``.
        """
        return (
            self.support >= thresholds.minimum_support
            and self.confidence >= thresholds.minimum_confidence
        )
