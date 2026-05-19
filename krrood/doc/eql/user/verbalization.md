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

# Query Verbalization

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
from krrood.entity_query_language.verbalization import verbalize_expression

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

```python
from krrood.entity_query_language.verbalization import VerbalizationPipeline

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
from krrood.entity_query_language.verbalization import VerbalizationPipeline

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
    variable, entity, an, deduced_variable, add, inference, refinement
)

@dataclass
class Connection:
    parent: Robot
    child: Robot
    is_fixed: bool

@dataclass
class View:
    root: Robot

@dataclass
class FixedView(View): pass

@dataclass
class RevoluteView(View): pass

connections = [
    Connection(robots[0], robots[1], True),
    Connection(robots[1], robots[2], False),
]

conn = variable(Connection, domain=connections)
view = deduced_variable(View)

rule_query = an(entity(view).where(conn.parent == r))

with rule_query:
    add(view, inference(FixedView)(root=r))
    with refinement(conn.is_fixed == False):
        add(view, inference(RevoluteView)(root=r))

HTML(VerbalizationPipeline.html(hierarchical=True).verbalize(rule_query))
```

The hierarchical renderer shows the **If/then** structure with each condition and conclusion
on its own line, indented under the relevant clause.

## Verbalization as an Explanation Tool

Because EQL tracks *how* inferences were made, you can verbalize the query that produced any
inferred result — not just hand-written queries.

```python
from krrood.entity_query_language.explanation import explain_inference
from krrood.entity_query_language.verbalization import verbalize_expression

# inferred_object was produced by a rule tree earlier
explanation = explain_inference(inferred_object)
print(verbalize_expression(explanation.query_root))
```

This outputs the exact query that matched and produced `inferred_object`, described in English.
It is directly useful for displaying *why* a robot perceives something as a Drawer, a Door, etc.

## API Reference

- {py:func}`~krrood.entity_query_language.verbalization.verbalizer.verbalize_expression` — plain text, one-liner
- {py:class}`~krrood.entity_query_language.verbalization.pipeline.VerbalizationPipeline` — full control over format and color
  - `.plain()` — plain text, paragraph prose
  - `.ansi()` — ANSI true-color terminal output
  - `.ansi(hierarchical=True)` — indented bullet structure, ANSI
  - `.html()` — HTML `<span>` colors, paragraph prose
  - `.html(hierarchical=True)` — HTML, indented bullet structure
