"""
Tests validating ``RuleMiner`` recovers a deliberately planted relational rule.
"""

from krrood.entity_query_language.rule_mining.miner import RuleMiner, SeedDomain
from krrood.entity_query_language.rule_mining.scoring import RuleScore, ScoreThresholds

from ...dataset.rule_mining_fixture import Container, Handle

# %% siblings-share-a-container


def siblings_sharing_a_container_world():
    """
    :return: Five handles across three containers: ``c1`` holds ``h1``/``h2``, ``c2``
        holds ``h3``/``h4``, ``c3`` holds ``h5`` alone — so "sibling" handles (sharing
        a container) exist for the first two but not the third.
    """
    c1 = Container(name="C1")
    c2 = Container(name="C2")
    c3 = Container(name="C3")
    h1 = Handle(name="H1", container=c1)
    h2 = Handle(name="H2", container=c1)
    h3 = Handle(name="H3", container=c2)
    h4 = Handle(name="H4", container=c2)
    h5 = Handle(name="H5", container=c3)
    c1.handles = [h1, h2]
    c2.handles = [h3, h4]
    c3.handles = [h5]
    return [h1, h2, h3, h4, h5]


def test_mine_recovers_the_planted_sibling_handle_rule():
    handles = siblings_sharing_a_container_world()

    results = RuleMiner(
        thresholds=ScoreThresholds(minimum_support=2, minimum_confidence=0.1),
        maximum_atoms=3,
    ).mine(Handle, handles)

    matches = [
        body
        for body in results
        if sorted(handle.name for handle in body.to_query().evaluate())
        == ["H1", "H1", "H2", "H2", "H3", "H3", "H4", "H4", "H5"]
    ]
    assert len(matches) >= 1
    assert matches[0].score() == RuleScore(support=9, confidence=0.36)


# %% mining across two entity types


def test_mine_recovers_a_rule_joining_the_head_to_an_auxiliary_type():
    """
    A rule relating two different types is only reachable if the search may seed a
    variable of a type other than the head's own.
    """
    handles = siblings_sharing_a_container_world()
    containers = []
    for handle in handles:
        if handle.container not in containers:
            containers.append(handle.container)

    results = RuleMiner(
        thresholds=ScoreThresholds(minimum_support=2, minimum_confidence=0.1),
        maximum_atoms=3,
    ).mine(
        Handle,
        handles,
        auxiliary_domains=[SeedDomain(entity_type=Container, instances=containers)],
    )

    matches = [
        body
        for body in results
        if sorted(handle.name for handle in body.to_query().evaluate())
        == ["H1", "H2", "H3", "H4", "H5"]
    ]
    assert len(matches) >= 1
    assert matches[0].score() == RuleScore(support=5, confidence=1.0)


def test_mine_without_auxiliary_domains_seeds_only_the_head_type():
    handles = siblings_sharing_a_container_world()

    results = RuleMiner(
        thresholds=ScoreThresholds(minimum_support=2, minimum_confidence=0.1),
        maximum_atoms=2,
    ).mine(Handle, handles)

    seeded_types = {
        variable._type_ for body in results for variable in body.open_variables
    }
    assert Container not in seeded_types
