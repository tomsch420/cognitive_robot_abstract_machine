"""
Tests validating ``RuleMiner`` recovers a deliberately planted relational rule.
"""

from krrood.entity_query_language.rule_mining.miner import RuleMiner
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
