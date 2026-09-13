import time
import numpy as np
import random
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit as RXProbabilisticCircuit,
    SumUnit,
    ProductUnit,
    leaf,
)
from probabilistic_model.probabilistic_circuit.numpy.probabilistic_circuit import (
    ProbabilisticCircuit as NumPyProbabilisticCircuit,
)
from probabilistic_model.distributions.gaussian import GaussianDistribution
from probabilistic_model.distributions.distributions import SymbolicDistribution
from random_events.variable import Continuous, Symbolic
from random_events.product_algebra import SimpleEvent
from random_events.interval import closed
from random_events.set import Set
import typing


def generate_random_circuit(variables, depth, width):
    pc = RXProbabilisticCircuit()

    current_layer = []
    for var in variables:
        for _ in range(width):
            if isinstance(var, Continuous):
                dist = GaussianDistribution(
                    variable=var,
                    location=random.uniform(-5, 5),
                    scale=random.uniform(0.5, 2.0),
                )
            else:
                domain_elements = list(var.domain.simple_sets)
                probs = np.random.dirichlet(np.ones(len(domain_elements)))
                dist = SymbolicDistribution(
                    variable=var,
                    probabilities={
                        hash(val): p for val, p in zip(domain_elements, probs)
                    },
                )
            l = leaf(dist, pc)
            current_layer.append(l)

    for d in range(depth):
        product_layer = []
        if len(variables) > 1:
            nodes_by_var = {
                var: [n for n in current_layer if var in n.variables]
                for var in variables
            }
            unused_nodes = set(current_layer)

            while unused_nodes or len(product_layer) < width:
                p = ProductUnit(probabilistic_circuit=pc)
                for var in variables:
                    var_unused = [n for n in unused_nodes if var in n.variables]
                    if var_unused:
                        node = random.choice(var_unused)
                        unused_nodes.remove(node)
                    else:
                        node = random.choice(nodes_by_var[var])
                    p.add_subcircuit(node)
                product_layer.append(p)
        else:
            product_layer = current_layer

        sum_layer = []
        unused_nodes = set(product_layer)
        while unused_nodes or len(sum_layer) < width:
            s = SumUnit(probabilistic_circuit=pc)
            if unused_nodes:
                node = random.choice(list(unused_nodes))
                unused_nodes.remove(node)
                children = [node]
            else:
                children = [random.choice(product_layer)]

            num_extra = random.randint(1, 4)
            if len(product_layer) > 1:
                extra_children = random.sample(
                    product_layer, min(len(product_layer), num_extra)
                )
                for c in extra_children:
                    if c not in children:
                        children.append(c)

            weights = np.random.dirichlet(np.ones(len(children)))
            for child, weight in zip(children, weights):
                s.add_subcircuit(child, np.log(weight))
            sum_layer.append(s)
        current_layer = sum_layer

    root = SumUnit(probabilistic_circuit=pc)
    weights = np.random.dirichlet(np.ones(len(current_layer)))
    for child, weight in zip(current_layer, weights):
        root.add_subcircuit(child, np.log(weight))

    return pc


def benchmark():
    np.random.seed(42)
    random.seed(42)

    c_vars = [Continuous(f"c{i}") for i in range(2)]
    s_vars = [
        Symbolic(f"s{i}", domain=Set.from_iterable(["a", "b", "c"])) for i in range(1)
    ]
    variables = c_vars + s_vars

    configs = [(1, 5), (2, 10), (3, 20), (4, 40)]
    batch_sizes = [1000, 10000]

    print(
        f"{'Config (D,W)':<15} | {'Batch':<7} | {'Op':<15} | {'RX (s)':<10} | {'NP (s)':<10} | {'Speedup':<8}"
    )
    print("-" * 85)

    for depth, width in configs:
        rx_pc = generate_random_circuit(variables, depth, width)

        start = time.time()
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)
        conv_time = time.time() - start

        config_str = f"({depth}, {width})"

        for batch_size in batch_sizes:
            # Construct data
            data = np.empty((batch_size, len(variables)), dtype=object)
            for i, var in enumerate(variables):
                if isinstance(var, Continuous):
                    data[:, i] = np.random.randn(batch_size)
                else:
                    domain_elements = list(var.domain.simple_sets)
                    data[:, i] = np.random.choice(domain_elements, size=batch_size)

            # Likelihood
            t0 = time.time()
            rx_pc.log_likelihood(data)
            t_rx = time.time() - t0
            t0 = time.time()
            np_pc.log_likelihood(data)
            t_np = time.time() - t0
            print(
                f"{config_str:<15} | {batch_size:<7} | {'Likelihood':<15} | {t_rx:10.4f} | {t_np:10.4f} | {t_rx/t_np:7.2f}x"
            )

            # Sampling
            t0 = time.time()
            rx_pc.sample(batch_size)
            t_rx = time.time() - t0
            t0 = time.time()
            np_pc.sample(batch_size)
            t_np = time.time() - t0
            print(
                f"{config_str:<15} | {batch_size:<7} | {'Sampling':<15} | {t_rx:10.4f} | {t_np:10.4f} | {t_rx/t_np:7.2f}x"
            )

            # Combined
            t0 = time.time()
            samples = rx_pc.sample(batch_size)
            rx_pc.log_likelihood(samples)
            t_rx = time.time() - t0
            t0 = time.time()
            samples = np_pc.sample(batch_size)
            np_pc.log_likelihood(samples)
            t_np = time.time() - t0
            print(
                f"{config_str:<15} | {batch_size:<7} | {'Samp+LL':<15} | {t_rx:10.4f} | {t_np:10.4f} | {t_rx/t_np:7.2f}x"
            )

        # Truncation
        trunc_configs = [1, 10, 100]
        for num_simple_sets in trunc_configs:
            events = []
            for _ in range(3):
                composite_event = None
                for _ in range(num_simple_sets):
                    event_data = {}
                    for v in variables:
                        if isinstance(v, Continuous):
                            l, r = sorted([random.uniform(-2, 2), random.uniform(-2, 2)])
                            event_data[v] = closed(l, r)
                        else:
                            domain_elements = [se.element for se in v.domain.simple_sets]
                            event_data[v] = v.make_value(
                                random.sample(
                                    domain_elements, random.randint(1, len(domain_elements))
                                )
                            )
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
                f"{config_str:<15} | {'-':<7} | {f'Trunc ({num_simple_sets} sets)':<15} | {t_rx_total:10.4f} | {t_np_total:10.4f} | {t_rx_total/t_np_total:7.2f}x"
            )

        # Conditioning
        points = []
        for _ in range(3):
            pt = {
                variables[0]: random.uniform(-1, 1),
                variables[2]: random.choice(list(variables[2].domain.simple_sets)),
            }
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
            f"{config_str:<15} | {'-':<7} | {'Conditioning (x3)':<15} | {t_rx_total:10.4f} | {t_np_total:10.4f} | {t_rx_total/t_np_total:7.2f}x"
        )
        print("-" * 85)


if __name__ == "__main__":
    benchmark()
