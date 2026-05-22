---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.16.4
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Verbalization

EQL queries are Python objects — they can be inspected, composed, stored, and now: **read aloud**.

Verbalization turns any EQL expression into a plain-English sentence. This is useful for:

- **Debugging** — instantly understand what a complex query actually asks.
- **Explainability** — surface query intent in logs, UIs, or reports.
- **Testing** — assert on what a query means, not just what it returns.

## The Quick API

The simplest way to verbalize any EQL expression is `verbalize_expression`.

```{code-cell} ipython3
from dataclasses import dataclass
from krrood.entity_query_language.factories import variable, entity, an
from krrood.entity_query_language.verbalization.verbalizer import verbalize_expression

@dataclass
class Robot:
    name: str
    battery: int

robots = [Robot("R2D2", 95), Robot("C3PO", 20), Robot("BB8", 80)]
r = variable(Robot, domain=robots)

query = an(entity(r).where(r.battery > 50))
print(verbalize_expression(query))
```

The output reads like a natural sentence describing exactly what the query selects.

## More Conditions, Still Readable

Adding more `.where()` conditions does not break the sentence — EQL connects them naturally.

```{code-cell} ipython3
query = an(entity(r).where(r.battery > 50, r.name != "BB8"))
print(verbalize_expression(query))
```

## Cross-Variable Conditions

Verbalization handles cross-variable comparisons too — the sentence describes the *relationship* between variables.

```{code-cell} ipython3
@dataclass
class Mission:
    assigned_to: Robot
    priority: int

missions = [Mission(robots[0], 1), Mission(robots[1], 3)]
m = variable(Mission, domain=missions)

query = an(entity(r).where(m.assigned_to == r, m.priority > 2))
print(verbalize_expression(query))
```

## Colored Terminal Output

For richer output in a terminal, use `VerbalizationPipeline.ansi()`. Each part of the sentence is
color-coded by its semantic role.

```{code-cell} ipython3
from krrood.entity_query_language.verbalization.pipeline import VerbalizationPipeline

pipeline = VerbalizationPipeline.ansi()
print(pipeline.verbalize(query))
```

Color legend:

| Color | Role | Example |
|---|---|---|
| Cornflower blue | **Variable type** | `Robot`, `Mission` |
| Teal | **Attribute** | `battery`, `assigned_to` |
| Orange | **Operator** | `is greater than`, `is not` |
| Green | **Logical connective** | `and`, `or`, `such that` |
| Gray | **Literal value** | `50`, `"BB8"` |
| Yellow | **Keyword / rule structure** | `If`, `then`, `whose` |

## HTML Output for Notebooks

`VerbalizationPipeline.html()` produces `<span>` tags for direct use in Jupyter or any HTML context.

```{code-cell} ipython3
from IPython.display import HTML
from krrood.entity_query_language.verbalization.pipeline import VerbalizationPipeline

pipeline = VerbalizationPipeline.html()
HTML(pipeline.verbalize(query))
```

### Hierarchical HTML

Pass `hierarchical=True` to get an indented bullet structure — great for rule trees.

```{code-cell} ipython3
HTML(VerbalizationPipeline.html(hierarchical=True).verbalize(query))
```

## Verbalizing Rule Trees

Verbalization really shines on rule trees. The if/then structure is rendered clearly.

```{code-cell} ipython3
from krrood.entity_query_language.factories import (
    variable, entity, an, deduced_variable, add, inference, refinement, Symbol, not_
)
@dataclass
class Bird:
    name: str

@dataclass
class LoveBirds:
    bird_1: Bird
    bird_2: Bird
    strong_love: bool

@dataclass
class BirdView(Symbol):
    bird: Bird

@dataclass
class StrongLoveBird(BirdView): pass

@dataclass
class WeakLoveBird(BirdView): pass

birds = [Bird('tweety'), Bird('snappy'), Bird('sleepy')]
bird = birds[0]
love_birds = [
    LoveBirds(birds[0], birds[1], True),
    LoveBirds(birds[1], birds[2], False),
]

love_birds = variable(LoveBirds, domain=love_birds)
bird_view = deduced_variable(BirdView)
rule_query = an(entity(inference(StrongLoveBird)(bird=love_birds.bird_1)).where(love_birds.strong_love))

HTML(VerbalizationPipeline.html(hierarchical=True).verbalize(rule_query))
```

The hierarchical renderer shows the **If/then** structure with each condition and conclusion
on its own line, indented under the relevant clause.

## Verbalization as an Explanation Tool

Because EQL tracks *how* inferences were made, you can verbalize the query that produced any
inferred result — not just hand-written queries.

```{code-cell} ipython3
from krrood.entity_query_language.explanation.explanation import explain_inference

inferred_views = list(rule_query.evaluate())
inferred_object = inferred_views[0]

explanation = explain_inference(inferred_object)
print(verbalize_expression(explanation.query_root))
```

This outputs the exact query that matched and produced `inferred_object`, described in English.
It is directly useful for displaying *why* a robot perceives something as a Drawer, a Door, etc.

## Hyperlinks to Source Code

Pass `link_resolver=AutoAPIResolver(...)` to any pipeline factory and class and attribute
names become clickable links — opening the corresponding Sphinx AutoAPI documentation page.

`AutoAPIResolver` works in two modes:

| Mode | How to construct | When it works |
|---|---|---|
| **Local** | `AutoAPIResolver.for_package("krrood")` | After `sphinx-build doc doc/_build/html` |
| **GitHub Pages** | `AutoAPIResolver(base_url="https://cram2.github.io/…/krrood")` | Always, no local build needed |

The demo below uses `verbalization_domain.Robot` and `verbalization_domain.Mission` — classes
defined in `doc/eql/user/verbalization_domain.py` whose mock API page is committed alongside
this notebook and deployed with the docs.

```{code-cell} ipython3
:tags: [remove-cell]

import sys
from pathlib import Path
# verbalization_domain.py lives in doc/eql/user/.
# When the notebook kernel runs from test_tmp/, Path("..") resolves there.
for _candidate in [Path("doc/eql/user"), Path("eql/user"), Path(".."), Path(".")]:
    if (_candidate / "verbalization_domain.py").exists():
        sys.path.insert(0, str(_candidate.resolve()))
        break
```

```{code-cell} ipython3
from verbalization_domain import Robot, Mission
from krrood.entity_query_language.factories import variable, entity, an
from krrood.entity_query_language.verbalization.pipeline import VerbalizationPipeline
from krrood.entity_query_language.verbalization.rendering.source_link_resolver import AutoAPIResolver

vd_robots = [Robot("R2D2", 95), Robot("C3PO", 20)]
vd_missions = [Mission(vd_robots[0], 3)]
r = variable(Robot, domain=vd_robots)
m = variable(Mission, domain=vd_missions)
linked_query = an(entity(r).where(m.assigned_to == r, m.priority > 2))

# Local — requires docs to be built first: sphinx-build doc doc/_build/html
resolver = AutoAPIResolver.for_package("krrood")
VerbalizationPipeline.html(link_resolver=resolver).display(linked_query)
```

```{code-cell} ipython3
# GitHub Pages — always available, no local build needed.
resolver = AutoAPIResolver(base_url="https://cram2.github.io/cognitive_robot_abstract_machine/krrood")
VerbalizationPipeline.html(link_resolver=resolver).display(linked_query)
```

## API Reference

- {py:func}`~krrood.entity_query_language.verbalization.verbalizer.verbalize_expression` — plain text, one-liner
- {py:class}`~krrood.entity_query_language.verbalization.pipeline.VerbalizationPipeline` — full control over format and color
  - `.plain()` — plain text, paragraph prose
  - `.ansi()` — ANSI true-color terminal output
  - `.ansi(hierarchical=True)` — indented bullet structure, ANSI
  - `.html()` — HTML `<span>` colors, paragraph prose
  - `.html(hierarchical=True)` — HTML, indented bullet structure
  - `.html(link_resolver=...)` — adds clickable hyperlinks to class and attribute names
  - `.display(expr)` — renders inline in Jupyter or opens a browser tab elsewhere
