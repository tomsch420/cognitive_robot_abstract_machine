import unittest
import numpy as np
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import ProbabilisticCircuit as RXProbabilisticCircuit, SumUnit, ProductUnit, leaf
from probabilistic_model.probabilistic_circuit.numpy.probabilistic_circuit import LayeredProbabilisticCircuit
from probabilistic_model.distributions.gaussian import GaussianDistribution
from probabilistic_model.distributions.distributions import SymbolicDistribution
from probabilistic_model.distributions.uniform import UniformDistribution
from random_events.variable import Continuous, Symbolic
from random_events.set import Set
from random_events.interval import closed, SimpleInterval, Bound
from sortedcontainers import SortedSet

class NumPyCircuitTestCase(unittest.TestCase):
    def test_gaussian_circuit(self):
        x = Continuous("x")
        y = Continuous("y")
        variables = SortedSet([x, y])

        rustworkx_probabilistic_circuit = RXProbabilisticCircuit()

        # Build a small circuit
        # P(x, y) = 0.6 * N(x|0, 1) * N(y|0, 1) + 0.4 * N(x|2, 1) * N(y|2, 1)

        leaf1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rustworkx_probabilistic_circuit)
        leaf2 = leaf(GaussianDistribution(variable=y, location=0, scale=1), rustworkx_probabilistic_circuit)
        product1 = ProductUnit(probabilistic_circuit=rustworkx_probabilistic_circuit)
        product1.add_subcircuit(leaf1)
        product1.add_subcircuit(leaf2)

        leaf3 = leaf(GaussianDistribution(variable=x, location=2, scale=1), rustworkx_probabilistic_circuit)
        leaf4 = leaf(GaussianDistribution(variable=y, location=2, scale=1), rustworkx_probabilistic_circuit)
        product2 = ProductUnit(probabilistic_circuit=rustworkx_probabilistic_circuit)
        product2.add_subcircuit(leaf3)
        product2.add_subcircuit(leaf4)

        root = SumUnit(probabilistic_circuit=rustworkx_probabilistic_circuit)
        root.add_subcircuit(product1, np.log(0.6))
        root.add_subcircuit(product2, np.log(0.4))

        # Convert to NumPy
        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)

        # Compare log-likelihood
        data = np.array([[0, 0], [1, 1], [2, 2]])
        rustworkx_log_likelihood = rustworkx_probabilistic_circuit.log_likelihood(data)
        layered_log_likelihood = layered_probabilistic_circuit.log_likelihood(data)
        np.testing.assert_allclose(rustworkx_log_likelihood, layered_log_likelihood)

        # Compare moments
        order = {x: 1, y: 1}
        center = {x: 0, y: 0}
        rustworkx_moment = rustworkx_probabilistic_circuit.moment(order, center)
        layered_moment = layered_probabilistic_circuit.moment(order, center)
        self.assertAlmostEqual(rustworkx_moment[x], layered_moment[x])
        self.assertAlmostEqual(rustworkx_moment[y], layered_moment[y])

    def test_sampling(self):
        x = Continuous("x")
        variables = SortedSet([x])
        rustworkx_probabilistic_circuit = RXProbabilisticCircuit()
        leaf1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rustworkx_probabilistic_circuit)
        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)
        samples = layered_probabilistic_circuit.sample(100)
        self.assertEqual(samples.shape, (100, 1))

    def test_cdf(self):
        x = Continuous("x")
        rustworkx_probabilistic_circuit = RXProbabilisticCircuit()
        leaf1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rustworkx_probabilistic_circuit)
        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)
        data = np.array([[0], [1], [2]])
        rustworkx_cumulative_distribution_function = rustworkx_probabilistic_circuit.cumulative_distribution_function(data)
        layered_cumulative_distribution_function = layered_probabilistic_circuit.cumulative_distribution_function(data)
        np.testing.assert_allclose(rustworkx_cumulative_distribution_function, layered_cumulative_distribution_function)

    def test_discrete_circuit(self):
        x = Symbolic("x", domain=Set.from_iterable([0, 1, 2]))
        rustworkx_probabilistic_circuit = RXProbabilisticCircuit()
        leaf1 = leaf(SymbolicDistribution(variable=x, probabilities={0: 0.1, 1: 0.2, 2: 0.7}), rustworkx_probabilistic_circuit)
        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)

        data = np.array([[0], [1], [2]])
        rustworkx_log_likelihood = rustworkx_probabilistic_circuit.log_likelihood(data)
        layered_log_likelihood = layered_probabilistic_circuit.log_likelihood(data)
        np.testing.assert_allclose(rustworkx_log_likelihood, layered_log_likelihood)

    def test_uniform_circuit(self):
        x = Continuous("x")
        rustworkx_probabilistic_circuit = RXProbabilisticCircuit()
        interval = SimpleInterval.from_data(0, 10, Bound.CLOSED, Bound.CLOSED)
        leaf1 = leaf(UniformDistribution(variable=x, interval=interval), rustworkx_probabilistic_circuit)
        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)

        data = np.array([[0], [5], [10]])
        rustworkx_log_likelihood = rustworkx_probabilistic_circuit.log_likelihood(data)
        layered_log_likelihood = layered_probabilistic_circuit.log_likelihood(data)
        np.testing.assert_allclose(rustworkx_log_likelihood, layered_log_likelihood)

    def test_mode(self):
        x = Continuous("x")
        rustworkx_probabilistic_circuit = RXProbabilisticCircuit()
        leaf1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rustworkx_probabilistic_circuit)
        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)

        rustworkx_mode, rustworkx_log_likelihood = rustworkx_probabilistic_circuit.log_mode()
        layered_mode, layered_log_likelihood = layered_probabilistic_circuit.log_mode()
        self.assertEqual(rustworkx_mode, layered_mode)
        self.assertAlmostEqual(rustworkx_log_likelihood, layered_log_likelihood)

    def test_truncation(self):
        from random_events.interval import closed
        from random_events.product_algebra import SimpleEvent
        x = Continuous("x")
        rustworkx_probabilistic_circuit = RXProbabilisticCircuit()
        leaf1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rustworkx_probabilistic_circuit)
        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)

        event = SimpleEvent.from_data({x: closed(-1, 1)}).as_composite_set()
        rustworkx_truncated, rustworkx_log_probability = rustworkx_probabilistic_circuit.log_truncated(event)
        layered_truncated, layered_log_probability = layered_probabilistic_circuit.log_truncated(event)

        self.assertAlmostEqual(rustworkx_log_probability, layered_log_probability)

        # Test log-likelihood of truncated model
        data = np.array([[0], [0.5]])
        rustworkx_truncated_log_likelihood = rustworkx_truncated.log_likelihood(data)
        layered_truncated_log_likelihood = layered_truncated.log_likelihood(data)
        np.testing.assert_allclose(rustworkx_truncated_log_likelihood, layered_truncated_log_likelihood)

    def test_marginal(self):
        x = Continuous("x")
        y = Continuous("y")
        rustworkx_probabilistic_circuit = RXProbabilisticCircuit()
        leaf1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rustworkx_probabilistic_circuit)
        leaf2 = leaf(GaussianDistribution(variable=y, location=2, scale=1), rustworkx_probabilistic_circuit)
        product1 = ProductUnit(probabilistic_circuit=rustworkx_probabilistic_circuit)
        product1.add_subcircuit(leaf1)
        product1.add_subcircuit(leaf2)

        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)

        rustworkx_marginal = rustworkx_probabilistic_circuit.marginal([x])
        layered_marginal = layered_probabilistic_circuit.marginal([x])

        data = np.array([[0]])
        rustworkx_log_likelihood = rustworkx_marginal.log_likelihood(data)
        layered_log_likelihood = layered_marginal.log_likelihood(data)
        np.testing.assert_allclose(rustworkx_log_likelihood, layered_log_likelihood)

if __name__ == '__main__':
    unittest.main()
