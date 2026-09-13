import time
import numpy as np
import pandas as pd
from sklearn.datasets import fetch_california_housing
from probabilistic_model.learning.jpt.jpt import JointProbabilityTree
from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
from probabilistic_model.probabilistic_circuit.numpy.probabilistic_circuit import (
    ProbabilisticCircuit as NumPyProbabilisticCircuit,
)
from random_events.variable import Continuous
from random_events.product_algebra import SimpleEvent
from random_events.interval import closed
import random


def benchmark_jpt():
    print("Loading California Housing dataset...")
    data = fetch_california_housing(as_frame=True)
    df = data.frame

    # Use a subset of data for faster training if needed, but benchmark on full
    # Actually, JPT training can be slow, so I'll limit the depth/leaves
    print("Training JPT (this might take a minute)...")
    variables = infer_variables_from_dataframe(df)
    jpt = JointProbabilityTree(
        annotated_variables=variables, min_samples_per_leaf=100, max_depth=10
    )
    t0 = time.time()
    rx_pc = jpt.fit(df)
    train_time = time.time() - t0
    print(
        f"JPT trained in {train_time:.2f}s. Number of nodes in RX circuit: {len(rx_pc)}"
    )

    print("Converting to NumPy circuit...")
    t0 = time.time()
    np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)
    conv_time = time.time() - t0
    print(
        f"Converted in {conv_time:.4f}s. Number of layers: {len(np_pc.root.all_layers())}"
    )

    batch_sizes = [1000, 10000, 20000]

    print(
        f"\n{'Batch Size':<12} | {'Operation':<15} | {'RX (s)':<10} | {'NP (s)':<10} | {'Speedup':<8}"
    )
    print("-" * 65)

    variables = rx_pc.variables

    for batch_size in batch_sizes:
        # Prepare test data
        test_data = df.sample(n=batch_size, replace=True).values

        # Likelihood
        t0 = time.time()
        rx_ll = rx_pc.log_likelihood(test_data)
        t_rx = time.time() - t0

        t0 = time.time()
        np_ll = np_pc.log_likelihood(test_data)
        t_np = time.time() - t0

        # Check correctness
        np.testing.assert_allclose(rx_ll, np_ll, atol=1e-5, rtol=1e-5)

        print(
            f"{batch_size:<12} | {'Likelihood':<15} | {t_rx:10.4f} | {t_np:10.4f} | {t_rx/t_np:7.2f}x"
        )

        # Sampling
        t0 = time.time()
        rx_pc.sample(batch_size)
        t_rx = time.time() - t0

        t0 = time.time()
        np_pc.sample(batch_size)
        t_np = time.time() - t0
        print(
            f"{batch_size:<12} | {'Sampling':<15} | {t_rx:10.4f} | {t_np:10.4f} | {t_rx/t_np:7.2f}x"
        )

    # Truncation (using 2 random events each)
    print("-" * 65)
    trunc_configs = [1, 10, 50]
    for num_simple_sets in trunc_configs:
        events = []
        for _ in range(2):
            composite_event = None
            for _ in range(num_simple_sets):
                event_data = {}
                for var in variables:
                    if isinstance(var, Continuous):
                        v_min = df[var.name].min()
                        v_max = df[var.name].max()
                        l, r = sorted(
                            [random.uniform(v_min, v_max), random.uniform(v_min, v_max)]
                        )
                        event_data[var] = closed(l, r)
                simple_event = SimpleEvent.from_data(event_data).as_composite_set()
                if composite_event is None:
                    composite_event = simple_event
                else:
                    composite_event = composite_event | simple_event
            events.append(composite_event)

        t_rx_total = 0
        t_np_total = 0
        for event in events:
            t0 = time.time()
            rx_pc.log_truncated(event)
            t_rx_total += time.time() - t0

            t0 = time.time()
            np_pc.log_truncated(event)
            t_np_total += time.time() - t0
        print(
            f"{'-':<12} | {f'Trunc ({num_simple_sets} sets)':<15} | {t_rx_total:10.4f} | {t_np_total:10.4f} | {t_rx_total/t_np_total:7.2f}x"
        )

    # Conditioning
    print("-" * 65)
    points = []
    for _ in range(5):
        # Condition on 2 variables
        vars_to_condition = random.sample(variables, 2)
        pt = {v: df[v.name].sample(1).values[0] for v in vars_to_condition}
        points.append(pt)

    t_rx_total = 0
    t_np_total = 0
    for pt in points:
        t0 = time.time()
        rx_pc.log_conditional(pt)
        t_rx_total += time.time() - t0

        t0 = time.time()
        np_pc.log_conditional(pt)
        t_np_total += time.time() - t0
    print(
        f"{'-':<12} | {'Condition (x5)':<15} | {t_rx_total:10.4f} | {t_np_total:10.4f} | {t_rx_total/t_np_total:7.2f}x"
    )


if __name__ == "__main__":
    benchmark_jpt()
