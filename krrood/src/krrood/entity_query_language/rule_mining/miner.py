"""
Breadth-first search over candidate rule bodies, recovering closed relational rules that
meet given support/confidence thresholds.
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Dict, List, Sequence, Tuple, Union

from krrood.class_diagrams.attribute_introspector import DataclassOnlyIntrospector
from krrood.entity_query_language.core.mapped_variable import CanBehaveLikeAVariable
from krrood.entity_query_language.factories import variable
from krrood.entity_query_language.rule_mining.candidate_rule import CandidateRuleBody
from krrood.entity_query_language.rule_mining.scoring import ScoreThresholds

# %% refinement steps


@dataclass(frozen=True)
class SeedDomain:
    """
    A pool of entities the search may introduce a fresh variable over.
    """

    entity_type: type
    """
    The static type of the entities in the pool.
    """

    instances: Sequence
    """
    The entities the seeded variable ranges over.
    """


@dataclass(frozen=True)
class SeedStep:
    """
    Introduces a second, independently-domained variable, so a pattern relating two
    separate entities can be searched for.

    Seeding the head's own pool searches for a self-join; seeding another pool searches
    for a rule relating the head to a different type.
    """

    seed_index: int
    """
    The index, within the search's seed domains, of the pool the variable is drawn from.
    """


@dataclass(frozen=True)
class ExtendStep:
    """
    The dangling-atom operator, addressed by the plan-step index of its source rather
    than the source variable itself, so a step is a plain, replayable value.
    """

    source_step: int
    """
    The index, within the same plan, of the step that introduced the source variable;
    ``-1`` means the head variable.
    """

    attribute_name: str
    """
    The attribute traversed from the source variable.
    """


@dataclass(frozen=True)
class CloseStep:
    """
    The closing-atom operator, addressed by the plan-step indices of the two variables
    it equates.
    """

    variable_a_step: int
    """
    The index, within the same plan, of the step that introduced the first variable.
    """

    variable_b_step: int
    """
    The index, within the same plan, of the step that introduced the second variable.
    """


RefinementStep = Union[SeedStep, ExtendStep, CloseStep]
"""
One step of a candidate rule body's construction plan.
"""

# %% rule miner


@dataclass
class RuleMiner:
    """
    Breadth-first, AMIE-style search over :class:`CandidateRuleBody`, built entirely on
    its three refinement operators plus depth-0 seeding, pruning by support.

    A candidate under search is represented as a plan (a list of :data:`RefinementStep`)
    rather than a live :class:`CandidateRuleBody`: the operators mutate a body in place,
    so branching the search by copying a partially-built body would require deep-copying
    live EQL variable objects, which recurses indefinitely over their dynamic attribute
    machinery. A plan is replayed into a fresh body only when a variable's declared
    attributes need inspecting or a candidate needs scoring.

    Deliberately minimal, sufficient to recover a hand-planted rule from a small
    synthetic domain, not the production miner a later item will run against real data:
    a dangling atom is only ever extended once from any given source (never chained a
    second hop deeper) and seeding happens at depth 0 only. Rules reached via different
    atom orders are not deduplicated.

    Reference: :cite:t:`galarraga2013amie`.
    """

    thresholds: ScoreThresholds
    """
    The minimum support and confidence a closed rule must clear to be returned.
    """

    maximum_atoms: int
    """
    The greatest number of atoms (conditions) a candidate rule body may accumulate
    before its branch stops being refined further.
    """

    def mine(
        self,
        head_type: type,
        domain: Sequence,
        auxiliary_domains: Sequence[SeedDomain] = (),
    ) -> List[CandidateRuleBody]:
        """
        Search for closed rule bodies over ``head_type`` that meet :attr:`thresholds`.

        :param head_type: The static type of the entities the mined rules are about.
        :param domain: The instances ``head_type``'s variable ranges over.
        :param auxiliary_domains: Further pools the search may seed a variable over, so
            rules relating the head to another type are reachable. The head's own pool
            is always seedable, which is what makes a self-join reachable.
        :return: Every closed candidate rule body found that meets :attr:`thresholds`.
        """
        head_variable = variable(head_type, domain=domain)
        seed_domains = [
            SeedDomain(entity_type=head_type, instances=domain),
            *auxiliary_domains,
        ]
        results: List[CandidateRuleBody] = []
        frontier: List[List[RefinementStep]] = [[]] + [
            [SeedStep(index)] for index in range(len(seed_domains))
        ]

        for _ in range(self.maximum_atoms):
            next_frontier: List[List[RefinementStep]] = []
            for plan in frontier:
                for refined_plan in self._refine(head_variable, seed_domains, plan):
                    body, _ = self._materialize(
                        head_variable, seed_domains, refined_plan
                    )
                    score = body.score()
                    if score.support < self.thresholds.minimum_support:
                        continue
                    if self._is_closed(refined_plan) and score.meets(self.thresholds):
                        results.append(body)
                    next_frontier.append(refined_plan)
            frontier = next_frontier

        return results

    def _refine(
        self,
        head_variable: CanBehaveLikeAVariable,
        seed_domains: Sequence[SeedDomain],
        plan: List[RefinementStep],
    ) -> List[List[RefinementStep]]:
        """
        Generate every single-atom extension of ``plan``.

        :return: One dangling-atom extend per declared attribute of the head variable
            and of any seeded variable, plus one closing atom per pair of currently-
            open, type-compatible variables.
        """
        body, introduced = self._materialize(head_variable, seed_domains, plan)
        refinements: List[List[RefinementStep]] = []

        extendable_steps = [-1] + [
            index for index, step in enumerate(plan) if isinstance(step, SeedStep)
        ]
        for source_step in extendable_steps:
            source_variable = (
                head_variable if source_step == -1 else introduced[source_step]
            )
            owner_type = source_variable._type_
            for attribute in DataclassOnlyIntrospector().discover(owner_type):
                refinements.append(
                    [*plan, ExtendStep(source_step, attribute.public_name)]
                )

        live_steps = [
            index
            for index, step in enumerate(plan)
            if isinstance(step, (ExtendStep, SeedStep)) and self._is_live(index, plan)
        ]
        for position, step_a in enumerate(live_steps):
            for step_b in live_steps[position + 1 :]:
                if self._types_are_compatible(introduced[step_a], introduced[step_b]):
                    refinements.append([*plan, CloseStep(step_a, step_b)])

        return refinements

    def _materialize(
        self,
        head_variable: CanBehaveLikeAVariable,
        seed_domains: Sequence[SeedDomain],
        plan: List[RefinementStep],
    ) -> Tuple[CandidateRuleBody, Dict[int, CanBehaveLikeAVariable]]:
        """
        Replay ``plan`` into a fresh :class:`CandidateRuleBody`.

        :return: The resulting body, and a mapping from each plan-step index that
            introduces a variable to the variable it introduced.
        """
        body = CandidateRuleBody(head_variable=head_variable)
        introduced: Dict[int, CanBehaveLikeAVariable] = {}
        for index, step in enumerate(plan):
            if isinstance(step, SeedStep):
                seed_domain = seed_domains[step.seed_index]
                seeded_variable = variable(
                    seed_domain.entity_type, domain=seed_domain.instances
                )
                body.open_variables.append(seeded_variable)
                introduced[index] = seeded_variable
            elif isinstance(step, ExtendStep):
                source = (
                    head_variable
                    if step.source_step == -1
                    else introduced[step.source_step]
                )
                body.extend_with_related_variable(source, step.attribute_name)
                introduced[index] = body.open_variables[-1]
            else:
                body.close_by_equating_variables(
                    introduced[step.variable_a_step], introduced[step.variable_b_step]
                )
        return body, introduced

    @staticmethod
    def _is_live(index: int, plan: List[RefinementStep]) -> bool:
        """
        :return: Whether the variable introduced at ``index`` has not yet been closed.
        """
        return not any(
            isinstance(step, CloseStep)
            and index in (step.variable_a_step, step.variable_b_step)
            for step in plan
        )

    @staticmethod
    def _is_closed(plan: List[RefinementStep]) -> bool:
        """
        :return: Whether every variable the plan introduced is connected to the rest of
            the rule — either closed against another variable, or used as the source of
            a further dangling-atom extension. A variable that is merely introduced and
            never used again is the one shape genuinely still "dangling".
        """
        introduced_steps = {
            index
            for index, step in enumerate(plan)
            if isinstance(step, (ExtendStep, SeedStep))
        }
        connected_steps = {
            step_index
            for step in plan
            if isinstance(step, CloseStep)
            for step_index in (step.variable_a_step, step.variable_b_step)
        } | {
            step.source_step
            for step in plan
            if isinstance(step, ExtendStep) and step.source_step != -1
        }
        return introduced_steps <= connected_steps

    @staticmethod
    def _types_are_compatible(
        variable_a: CanBehaveLikeAVariable, variable_b: CanBehaveLikeAVariable
    ) -> bool:
        """
        :return: Whether the two variables' static types share a common subtype, so an
            equality between them could hold — the same check
            :meth:`~CandidateRuleBody.close_by_equating_variables` makes itself.
        """
        type_a, type_b = variable_a._type_, variable_b._type_
        return issubclass(type_a, type_b) or issubclass(type_b, type_a)
