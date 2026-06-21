"""Relational synthetic benchmark for RelationalProbabilisticCircuit.

Ground-truth generative model
------------------------------
Each SceneRoom is sampled as follows:

    position.x ~ N(MU_X, SIGMA_X²)
    position.y ~ N(MU_Y, SIGMA_Y²)
    position.z = 0  (constant)
    orientation = (0, 0, 0, 1)  (constant)
    n_objects   ~ Categorical({2: P_2_OBJ, 3: P_3_OBJ})
    object_i.type ~ Categorical({TABLE: P_TABLE, CHAIR: P_CHAIR})

This exercises the full relational machinery: scalar features, discrete
aggregation statistics (chair_count, table_count, total_count), and a
variable-length exchangeable object list.

Quality metric
--------------
Forward KL divergence KL(RSPN ∥ oracle) estimated via RSPN self-samples:

    KL(q ∥ p) = E_{x ~ q}[log q(x) − log p(x)]

RSPN samples (always in-support) are evaluated under both the RSPN circuit
and the analytic oracle log-density. Lower is better; 0 = perfect match.

Baseline: a per-variable MLE model that fits each scalar coordinate and
each discrete variable independently from training data, ignoring relational
structure.

Run with:
    .venv/bin/python benchmark_rspn_relational.py
"""
from __future__ import annotations

import math
import numpy as np
import scipy.stats as stats
from collections import Counter

import test.krrood_test.dataset.ormatic_interface  # noqa: F401 — registers DAOs

from krrood.entity_query_language.factories import underspecified
from krrood.ormatic.data_access_objects.helper import to_dao
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from test.krrood_test.dataset.example_classes import (
    KRROODOrientation,
    KRROODPosition,
    SceneObject,
    SceneObjectType,
    SceneRoom,
)

# ── Ground-truth parameters ───────────────────────────────────────────────────

MU_X, SIGMA_X = 3.0, 0.8
MU_Y, SIGMA_Y = 1.5, 0.5
P_2_OBJ = 0.4   # probability that a room has 2 objects
P_3_OBJ = 0.6   # probability that a room has 3 objects
P_TABLE = 0.3   # probability that any object is a TABLE
P_CHAIR = 0.7   # probability that any object is a CHAIR

N_EVAL = 500
TRAIN_SIZES = [10, 20, 50, 100, 200, 500]
SEED = 0


# ── Generative model ──────────────────────────────────────────────────────────


def sample_room(rng: np.random.Generator) -> SceneRoom:
    """Draw one SceneRoom from the ground-truth distribution."""
    x = float(rng.normal(MU_X, SIGMA_X))
    y = float(rng.normal(MU_Y, SIGMA_Y))
    n = int(rng.choice([2, 3], p=[P_2_OBJ, P_3_OBJ]))
    types = rng.choice(
        [SceneObjectType.TABLE, SceneObjectType.CHAIR],
        size=n,
        p=[P_TABLE, P_CHAIR],
    )
    return SceneRoom(
        position=KRROODPosition(x=x, y=y, z=0.0),
        orientation=KRROODOrientation(x=0, y=0, z=0, w=1),
        objects=[SceneObject(type=t) for t in types],
    )


def generate_rooms(n: int, rng: np.random.Generator) -> list[SceneRoom]:
    """Sample ``n`` SceneRoom instances."""
    return [sample_room(rng) for _ in range(n)]


# ── Oracle log-likelihood ─────────────────────────────────────────────────────


def oracle_log_likelihood_room(room: SceneRoom) -> float:
    """Analytic log-likelihood of one SceneRoom under the true parameters."""
    n = len(room.objects)
    p_n = P_2_OBJ if n == 2 else P_3_OBJ
    log_p = (
        stats.norm.logpdf(room.position.x, MU_X, SIGMA_X)
        + stats.norm.logpdf(room.position.y, MU_Y, SIGMA_Y)
        + math.log(p_n)
        + sum(
            math.log(P_TABLE if obj.type == SceneObjectType.TABLE else P_CHAIR)
            for obj in room.objects
        )
    )
    return log_p


def oracle_log_density_from_circuit_samples(
    samples: np.ndarray,
    var_index: dict,
    n_objects: int,
) -> np.ndarray:
    """Evaluate oracle log-density on samples drawn from a grounded RSPN circuit.

    Reconstructs room log-probabilities from the variable columns in ``samples``.

    :param samples: Array of shape (n, n_vars) from ``grounded.sample()``.
    :param var_index: The ``variable_to_index_map`` of the grounded circuit.
    :param n_objects: Number of objects in the query (fixed per circuit).
    :returns: Array of shape (n,) with per-sample log-likelihoods.
    """
    n = len(samples)
    result = np.zeros(n)

    # Position scalars
    result += stats.norm.logpdf(samples[:, var_index["SceneRoom.position.x"]], MU_X, SIGMA_X)
    result += stats.norm.logpdf(samples[:, var_index["SceneRoom.position.y"]], MU_Y, SIGMA_Y)

    # Count probability
    p_n = P_2_OBJ if n_objects == 2 else P_3_OBJ
    result += math.log(p_n)

    # Object type probabilities — each column contains the hash of the enum value;
    # we can recover the probability by checking which hash matches TABLE / CHAIR.
    hash_table = np.float64(hash(SceneObjectType.TABLE))
    hash_chair = np.float64(hash(SceneObjectType.CHAIR))
    for i in range(n_objects):
        col = samples[:, var_index[f"SceneRoom.objects[{i}].type"]]
        log_p_type = np.where(col == hash_table, math.log(P_TABLE), math.log(P_CHAIR))
        result += log_p_type

    return result


# ── Baseline log-density ──────────────────────────────────────────────────────


def baseline_log_density_from_circuit_samples(
    train_rooms: list[SceneRoom],
    samples: np.ndarray,
    var_index: dict,
    n_objects: int,
) -> np.ndarray:
    """MLE per-variable Gaussian/Categorical, evaluated on circuit samples.

    Fits each scalar coordinate and each discrete variable independently,
    ignoring the relational structure (no joint model over object counts).
    """
    n = len(samples)
    result = np.zeros(n)

    # Scalar positions — MLE Gaussian
    train_x = np.array([r.position.x for r in train_rooms])
    train_y = np.array([r.position.y for r in train_rooms])
    mu_x, sig_x = train_x.mean(), max(train_x.std(ddof=1), 1e-6)
    mu_y, sig_y = train_y.mean(), max(train_y.std(ddof=1), 1e-6)
    result += stats.norm.logpdf(samples[:, var_index["SceneRoom.position.x"]], mu_x, sig_x)
    result += stats.norm.logpdf(samples[:, var_index["SceneRoom.position.y"]], mu_y, sig_y)

    # Count distribution — MLE Categorical
    count_freq: Counter[int] = Counter(len(r.objects) for r in train_rooms)
    total = len(train_rooms)
    p_n = count_freq.get(n_objects, 0) / total if total > 0 else 1e-10
    result += math.log(max(p_n, 1e-10))

    # Object types — MLE Categorical (pooled across all objects and positions)
    all_types = [obj.type for r in train_rooms for obj in r.objects]
    type_freq: Counter[SceneObjectType] = Counter(all_types)
    total_types = len(all_types) or 1
    p_table = type_freq.get(SceneObjectType.TABLE, 0) / total_types
    p_chair = type_freq.get(SceneObjectType.CHAIR, 0) / total_types

    hash_table = np.float64(hash(SceneObjectType.TABLE))
    for i in range(n_objects):
        col = samples[:, var_index[f"SceneRoom.objects[{i}].type"]]
        log_p_type = np.where(
            col == hash_table,
            math.log(max(p_table, 1e-10)),
            math.log(max(p_chair, 1e-10)),
        )
        result += log_p_type

    return result


# ── KL helpers ────────────────────────────────────────────────────────────────


def forward_kl(log_q: np.ndarray, log_p: np.ndarray) -> float:
    """Estimate KL(q ∥ p) = E_q[log q − log p] from matched log-density arrays."""
    finite_mask = np.isfinite(log_q) & np.isfinite(log_p)
    if not finite_mask.any():
        return float("inf")
    return float(np.mean(log_q[finite_mask] - log_p[finite_mask]))


# ── Benchmark loop ────────────────────────────────────────────────────────────


def run_benchmark_for_count(n_objects: int, rng_seed: int) -> None:
    rng = np.random.default_rng(rng_seed)
    print(f"\n── Rooms with exactly {n_objects} objects ─────────────────────────────")
    print(f"{'N_train':>8}  {'KL(RSPN)':>12}  {'KL(Baseline)':>14}  {'RSPN better?':>13}")
    print("-" * 56)

    for n_train in TRAIN_SIZES:
        all_train = generate_rooms(n_train * 5, rng)  # oversample to get enough of this count
        train_rooms = [r for r in all_train if len(r.objects) == n_objects]
        if len(train_rooms) < 2:
            # Not enough rooms of this count; use mixed training data
            train_rooms = generate_rooms(n_train, rng)

        model = RelationalProbabilisticCircuit(SceneRoom)
        model.fit([to_dao(r) for r in train_rooms])

        query = underspecified(SceneRoom)(
            position=underspecified(KRROODPosition)(x=..., y=..., z=...),
            orientation=underspecified(KRROODOrientation)(x=..., y=..., z=..., w=...),
            objects=[underspecified(SceneObject)(type=...) for _ in range(n_objects)],
        )
        query.resolve()
        grounded = model.ground(query)

        samples = grounded.sample(N_EVAL)
        var_index = {v.name: i for v, i in grounded.variable_to_index_map.items()}

        rspn_log_q = grounded.log_likelihood(samples)
        oracle_log_p = oracle_log_density_from_circuit_samples(samples, var_index, n_objects)
        baseline_log_p = baseline_log_density_from_circuit_samples(
            train_rooms, samples, var_index, n_objects
        )

        kl_rspn = forward_kl(rspn_log_q, oracle_log_p)
        kl_base = forward_kl(baseline_log_p, oracle_log_p)

        print(
            f"{n_train:>8}  {kl_rspn:>+12.4f}  {kl_base:>+14.4f}  "
            f"{'yes' if kl_rspn < kl_base else 'no':>13}"
        )


def run_benchmark() -> None:
    print("Relational RSPN benchmark — SceneRoom with SceneObject lists")
    print(f"Ground truth: x~N({MU_X},{SIGMA_X}²)  y~N({MU_Y},{SIGMA_Y}²)")
    print(f"             n_objects ~ Cat({{2: {P_2_OBJ}, 3: {P_3_OBJ}}})")
    print(f"             obj.type  ~ Cat({{TABLE: {P_TABLE}, CHAIR: {P_CHAIR}}})")
    print(f"Metric: KL(model ∥ oracle) via {N_EVAL} RSPN self-samples [lower = better]")

    run_benchmark_for_count(n_objects=2, rng_seed=SEED)
    run_benchmark_for_count(n_objects=3, rng_seed=SEED + 1)

    print()
    print("KL(RSPN)     = E_{x~RSPN}[log p_RSPN(x) − log p_oracle(x)]")
    print("KL(Baseline) = E_{x~RSPN}[log p_baseline(x) − log p_oracle(x)]")
    print("Baseline: independent MLE Gaussian (position) + MLE Categorical (type) per variable.")


if __name__ == "__main__":
    run_benchmark()
