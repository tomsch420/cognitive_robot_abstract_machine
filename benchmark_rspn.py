"""Synthetic benchmark for RelationalProbabilisticCircuit quality.

Ground-truth generative model
------------------------------
Each sample is a KRROODPosition drawn from three independent Gaussians:

    x ~ N(MU_X, SIGMA_X²)
    y ~ N(MU_Y, SIGMA_Y²)
    z ~ N(MU_Z, SIGMA_Z²)

Quality metric
--------------
Forward KL divergence:  KL(RSPN ∥ oracle)

    KL(q ∥ p) = E_{x ~ q}[log q(x) − log p(x)]

Estimated by drawing N_EVAL samples from the fitted RSPN circuit and
evaluating both q (RSPN log-likelihood, in-support by construction) and
p (oracle analytic log-likelihood) on those samples.

Lower is better; 0 means the RSPN perfectly matches the true distribution.

We also report a "baseline" KL where q is a per-coordinate MLE Gaussian
fitted on the training data, evaluated on the same RSPN samples.

Run with:
    .venv/bin/python benchmark_rspn.py
"""
from __future__ import annotations

import numpy as np
import scipy.stats as stats

import test.krrood_test.dataset.ormatic_interface  # noqa: F401 — registers DAOs

from krrood.entity_query_language.factories import underspecified
from krrood.ormatic.data_access_objects.helper import to_dao
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from test.krrood_test.dataset.example_classes import KRROODPosition

# ── Ground-truth parameters ───────────────────────────────────────────────────

MU_X, SIGMA_X = 2.0, 0.50
MU_Y, SIGMA_Y = 1.0, 0.30
MU_Z, SIGMA_Z = 0.5, 0.15

N_EVAL = 1_000
TRAIN_SIZES = [10, 20, 50, 100, 200, 500]
SEED = 42

# Column order in the grounded circuit (verified at run time below).
COL_X, COL_Y, COL_Z = 0, 1, 2


# ── Helpers ───────────────────────────────────────────────────────────────────


def generate_positions(n: int, rng: np.random.Generator) -> list[KRROODPosition]:
    """Sample ``n`` KRROODPosition instances from the ground-truth Gaussians."""
    xs = rng.normal(MU_X, SIGMA_X, n)
    ys = rng.normal(MU_Y, SIGMA_Y, n)
    zs = rng.normal(MU_Z, SIGMA_Z, n)
    return [KRROODPosition(x=float(x), y=float(y), z=float(z)) for x, y, z in zip(xs, ys, zs)]


def fit_rspn(train_positions: list[KRROODPosition]) -> tuple[RelationalProbabilisticCircuit, object]:
    """Fit an RSPN and return it together with the grounded circuit."""
    model = RelationalProbabilisticCircuit(KRROODPosition)
    model.fit([to_dao(p) for p in train_positions])

    query = underspecified(KRROODPosition)(x=..., y=..., z=...)
    query.resolve()
    grounded = model.ground(query)

    # Verify column order matches our expectations.
    var_index = grounded.variable_to_index_map
    for var, col in var_index.items():
        attr = var.name.split(".")[-1]
        expected = {"x": COL_X, "y": COL_Y, "z": COL_Z}[attr]
        assert col == expected, f"Unexpected column layout: {var.name} at {col}"

    return model, grounded


def oracle_log_density(samples: np.ndarray) -> np.ndarray:
    """Analytic log-likelihood of ``samples`` under the true Gaussian parameters.

    :param samples: Array of shape (n, 3) with columns [x, y, z].
    :returns: Array of shape (n,) with per-sample log-likelihoods.
    """
    return (
        stats.norm.logpdf(samples[:, COL_X], MU_X, SIGMA_X)
        + stats.norm.logpdf(samples[:, COL_Y], MU_Y, SIGMA_Y)
        + stats.norm.logpdf(samples[:, COL_Z], MU_Z, SIGMA_Z)
    )


def baseline_log_density(
    train: list[KRROODPosition],
    samples: np.ndarray,
) -> np.ndarray:
    """Per-coordinate MLE Gaussian log-likelihood evaluated on ``samples``.

    :param train: Training positions used to estimate the MLE parameters.
    :param samples: Array of shape (n, 3).
    :returns: Array of shape (n,) with per-sample log-likelihoods.
    """
    train_arr = np.array([[p.x, p.y, p.z] for p in train])
    mu = train_arr.mean(axis=0)
    sigma = np.maximum(train_arr.std(axis=0, ddof=1), 1e-6)
    return sum(
        stats.norm.logpdf(samples[:, col], mu[col], sigma[col])
        for col in (COL_X, COL_Y, COL_Z)
    )


def forward_kl(log_q: np.ndarray, log_p: np.ndarray) -> float:
    """Estimate KL(q ∥ p) = E_q[log q − log p] from matched log-density arrays."""
    return float(np.mean(log_q - log_p))


# ── Benchmark loop ────────────────────────────────────────────────────────────


def run_benchmark() -> None:
    rng = np.random.default_rng(SEED)

    print("Synthetic RSPN benchmark — KRROODPosition (3-D independent Gaussians)")
    print(f"Ground truth: x~N({MU_X},{SIGMA_X}²)  y~N({MU_Y},{SIGMA_Y}²)  z~N({MU_Z},{SIGMA_Z}²)")
    print(f"Metric: KL(model ∥ oracle)  [lower is better; 0 = perfect match]")
    print(f"Samples per evaluation: {N_EVAL}")
    print()
    print(f"{'N_train':>8}  {'KL(RSPN)':>12}  {'KL(Baseline)':>14}  {'RSPN better?':>13}")
    print("-" * 56)

    for n_train in TRAIN_SIZES:
        train_positions = generate_positions(n_train, rng)
        _, grounded = fit_rspn(train_positions)

        # Draw samples from the RSPN; all are guaranteed in-support.
        rspn_samples = grounded.sample(N_EVAL)

        rspn_log_q = grounded.log_likelihood(rspn_samples)
        rspn_log_p_oracle = oracle_log_density(rspn_samples)
        rspn_log_p_baseline = baseline_log_density(train_positions, rspn_samples)

        kl_rspn = forward_kl(rspn_log_q, rspn_log_p_oracle)
        kl_baseline = forward_kl(rspn_log_p_baseline, rspn_log_p_oracle)
        rspn_wins = kl_rspn < kl_baseline

        print(
            f"{n_train:>8}  {kl_rspn:>+12.4f}  {kl_baseline:>+14.4f}  {'yes' if rspn_wins else 'no':>13}"
        )

    print()
    print("KL(RSPN)     = E_{x~RSPN}[log p_RSPN(x) − log p_oracle(x)]")
    print("KL(Baseline) = E_{x~RSPN}[log p_baseline(x) − log p_oracle(x)]")
    print("Note: baseline uses MLE Gaussians fitted per coordinate on the training set.")


if __name__ == "__main__":
    run_benchmark()
