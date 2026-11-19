---
jupytext:
  cell_metadata_filter: -all
  formats: md:myst
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.11.5
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Learning Methods for Probabilistic Circuits

This page compares published methods that learn the parameters or the structure of a
probabilistic circuit. It compares them by their reported results on standard benchmarks,
their cost, and how well they fit the tensorized layered circuits of this package
({doc}`layered_circuit`). From that comparison, it chooses the methods this package
implements next. Expressive architectures (continuous latent variables, squared or
negative weights) and Bayesian learning are out of scope.

The package already learns circuits with

- expectation maximization on circuits stored in rustworkx,
- gradient descent on layered circuits, in JAX and in PyTorch,
- the structure of randomized and tensorized sum-product networks (RAT-SPNs)
  {cite}`peharz2020random`, built from random region graphs,
- joint probability trees {cite}`nyga2023joint` and the Nyga induction
  ({doc}`joint_probability_trees`, {doc}`nyga_distribution`).

## Reported Results

### Twenty Binary Density Estimation Datasets

The twenty binary datasets (NLTCS to Ad) are the standard benchmark for circuit structure
learners. {cite}`liu2021tractable` reports the test log-likelihood of seven learners on all
twenty in one table (their table 2). The table below summarizes it: the mean over the
datasets of the average test log-likelihood per example, how many datasets each learner is
best on, and its mean rank among the seven.

| Learner                                       | Learns                        | Mean test LL | Best on | Mean rank |
|-----------------------------------------------|-------------------------------|-------------:|--------:|----------:|
| HCLT, regularized {cite}`liu2021tractable`    | parameters of a fixed structure |     -53.91 |      10 |      2.05 |
| ID-SPN {cite}`rooshenas2014learning`          | structure and parameters      |       -53.89 |       7 |      1.85 |
| EiNet {cite}`peharz2020einsum`                | parameters of a random structure |   -56.61 |       1 |      4.20 |
| LearnPSDD {cite}`liang2017learning`           | structure and parameters      |       -56.90 |       0 |      5.15 |
| LearnSPN {cite}`gens2013learning`             | structure and parameters      |       -56.96 |       0 |      4.95 |
| Strudel {cite}`dang2020strudel`               | structure and parameters      |       -57.13 |       0 |      5.20 |
| RAT-SPN {cite}`peharz2020random`              | parameters of a random structure |   -58.02 |       2 |      4.05 |

A hidden Chow-Liu tree (HCLT) circuit, trained with expectation maximization, data
softening and entropy regularization, ties ID-SPN on average and beats both deterministic
learners (LearnPSDD, Strudel) on every dataset.

### Large Image Datasets

On larger data the same structures are trained with stronger parameter learning. The
table reports test bits per dimension (lower is better). The MNIST row comes from
{cite}`dang2022sparse`, the ImageNet rows from {cite}`liu2023scaling` and
{cite}`liu2023understanding`; each of these papers also measured the baselines in its row.

| Dataset    | RAT-SPN | EiNet | HCLT | Sparse PC {cite}`dang2022sparse` | LVD {cite}`liu2023scaling` | LVD, improved {cite}`liu2023understanding` |
|------------|--------:|------:|-----:|---------------------------------:|---------------------------:|-------------------------------------------:|
| MNIST      |    1.67 |     - | 1.20 |                             1.14 |                          - |                                          - |
| ImageNet32 |    6.90 |  5.63 | 4.82 |                                - |                       4.39 |                                       4.06 |
| ImageNet64 |    6.82 |  5.69 | 4.67 |                                - |                       4.12 |                                       3.80 |

### Optimizers

{cite}`liu2026rethinking` compares optimizers for the parameters of a fixed circuit. On the
UK BioBank chromosome 6 data, with a 1024-state HCLT variant, the negative
log-likelihood is 110.4 with Adam, 47.2 with mini-batch expectation maximization, 45.1
with full-batch expectation maximization and 42.2 with their mini-batch method anemone.
Anemone reaches full-batch expectation maximization's final log-likelihood in 130 epochs,
against 1000.

## Methods

Cost is per method, for $N$ training examples over $d$ variables.

### Parameter Learning

Mini-batch expectation maximization {cite}`peharz2020einsum`
: Takes the expected counts of every sum unit from the gradient of the log-likelihood with
  respect to its weights, and updates the weights in closed form after every batch. Costs
  one forward and one backward pass per batch. It fits every array backend that
  differentiates, and is the optimizer the other parameter methods and pruning build on.

Anemone {cite}`liu2026rethinking`
: Mini-batch expectation maximization with a learning rate per parameter, scaled by how
  much the parameter contributes to the batch's likelihood. Costs the same as mini-batch
  expectation maximization, from the same expected counts.

Data softening and entropy regularization {cite}`liu2021tractable`
: Data softening trains on a weighted dataset in which every observed value may flip with
  a small probability; entropy regularization adds the entropy of the circuit to the
  objective of each maximization step. Both keep the cost of expectation maximization, and
  together they produced the HCLT results above.

Probabilistic dropout {cite}`peharz2020random`
: Drops input distributions and sum children at random during gradient training.
  Costs nothing beyond the mask.

Latent variable distillation {cite}`liu2023scaling`
: Assigns the latent variables of a circuit by clustering embeddings of a deep
  generative model, trains the circuit on the completed data, then fine-tunes it on the
  observed data. Training the deep generative model dominates the cost, and it needs a new
  dependency. It has been shown on images and text only, where it gives the largest gains
  above.

### Structure Learning

Hidden Chow-Liu trees {cite}`liu2021tractable`
: Learns a Chow-Liu tree {cite}`chow1968approximating` over the variables, replaces every
  variable with a latent variable that has $k$ states, and hangs the observed variable
  below it. The pairwise mutual information costs $O(d^2 N)$ once; the spanning tree comes
  from scipy. The tree compiles into alternating sum and product layers, so the circuit is
  a layered circuit and is trained by the parameter methods above.

Pruning and growing {cite}`dang2022sparse`
: Starting from an HCLT circuit, alternately removes the edges with the smallest
  expected flow and grows the circuit by copying and perturbing units, retraining after
  each step. Costs several training runs. The expected flows are the expected counts of
  expectation maximization, and the sum layers already store sparse edge matrices.

LearnSPN {cite}`gens2013learning`
: Recursively clusters the rows into the children of a sum unit and splits the variables
  into independent groups below a product unit. Costs a clustering and a set of
  pairwise independence tests at every level; scikit-learn provides the clustering. It
  overlaps with the joint probability tree induction, which also splits the rows top down
  but builds deterministic sums and factorizes only at its leaves. It is best on none of
  the twenty datasets.

Strudel {cite}`dang2020strudel`
: Greedily grows a smooth, deterministic and structured-decomposable circuit from a
  Chow-Liu based vtree. HCLT beats it in likelihood on every dataset above, but
  determinism makes most likely assignment queries tractable, which HCLT circuits do not
  answer exactly.

ID-SPN {cite}`rooshenas2014learning`
: LearnSPN with Markov networks, compiled into arithmetic circuits, as leaves. It ties
  HCLT on the twenty datasets, but its leaves are not available as layers.

## Decision

| Method                                   | Decision |
|------------------------------------------|----------|
| Mini-batch expectation maximization      | implement first; every other parameter method and pruning needs it |
| Hidden Chow-Liu trees                    | implement next; the strongest structure on the twenty datasets and the start for pruning |
| Data softening and entropy regularization | implement after mini-batch expectation maximization |
| Anemone                                  | add; implement after mini-batch expectation maximization |
| Pruning and growing                      | implement after hidden Chow-Liu trees and mini-batch expectation maximization |
| Strudel                                  | implement after hidden Chow-Liu trees; the deterministic learner, for exact most likely assignments |
| Probabilistic dropout                    | implement for gradient training |
| LearnSPN                                 | implement as the classical baseline, after checking its overlap with the joint probability tree induction |
| Latent variable distillation             | implement last, once its deep generative model is approved as a dependency |
| ID-SPN                                   | leave out; its leaves are not available as layers |

```{note}
Results from different papers come from different hyperparameter searches and
implementations. The shared benchmark of this package measures every implemented method
again, on the twenty binary datasets and on continuous UCI datasets, against RAT-SPN and
joint probability trees. It measures training time, peak memory, and train and test
log-likelihood; the speed of likelihood, conditioning, truncation, sampling and grounding
of relational circuits; and which queries, causal ones included, the circuit answers
exactly.
```
