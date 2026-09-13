import unittest
import numpy as np
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import ProbabilisticCircuit as RXProbabilisticCircuit, SumUnit, ProductUnit, leaf
from probabilistic_model.probabilistic_circuit.numpy.probabilistic_circuit import ProbabilisticCircuit as NumPyProbabilisticCircuit
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

        rx_pc = RXProbabilisticCircuit()

        # Build a small circuit
        # P(x, y) = 0.6 * N(x|0, 1) * N(y|0, 1) + 0.4 * N(x|2, 1) * N(y|2, 1)

        l1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rx_pc)
        l2 = leaf(GaussianDistribution(variable=y, location=0, scale=1), rx_pc)
        p1 = ProductUnit(probabilistic_circuit=rx_pc)
        p1.add_subcircuit(l1)
        p1.add_subcircuit(l2)

        l3 = leaf(GaussianDistribution(variable=x, location=2, scale=1), rx_pc)
        l4 = leaf(GaussianDistribution(variable=y, location=2, scale=1), rx_pc)
        p2 = ProductUnit(probabilistic_circuit=rx_pc)
        p2.add_subcircuit(l3)
        p2.add_subcircuit(l4)

        root = SumUnit(probabilistic_circuit=rx_pc)
        root.add_subcircuit(p1, np.log(0.6))
        root.add_subcircuit(p2, np.log(0.4))

        # Convert to NumPy
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)

        # Compare LL
        data = np.array([[0, 0], [1, 1], [2, 2]])
        rx_ll = rx_pc.log_likelihood(data)
        np_ll = np_pc.log_likelihood(data)
        np.testing.assert_allclose(rx_ll, np_ll)

        # Compare moments
        order = {x: 1, y: 1}
        center = {x: 0, y: 0}
        rx_m = rx_pc.moment(order, center)
        np_m = np_pc.moment(order, center)
        self.assertAlmostEqual(rx_m[x], np_m[x])
        self.assertAlmostEqual(rx_m[y], np_m[y])

    def test_sampling(self):
        x = Continuous("x")
        variables = SortedSet([x])
        rx_pc = RXProbabilisticCircuit()
        l1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rx_pc)
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)
        samples = np_pc.sample(100)
        self.assertEqual(samples.shape, (100, 1))

    def test_cdf(self):
        x = Continuous("x")
        rx_pc = RXProbabilisticCircuit()
        l1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rx_pc)
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)
        data = np.array([[0], [1], [2]])
        rx_cdf = rx_pc.cumulative_distribution_function(data)
        np_cdf = np_pc.cumulative_distribution_function(data)
        np.testing.assert_allclose(rx_cdf, np_cdf)

    def test_discrete_circuit(self):
        x = Symbolic("x", domain=Set.from_iterable([0, 1, 2]))
        rx_pc = RXProbabilisticCircuit()
        l1 = leaf(SymbolicDistribution(variable=x, probabilities={0: 0.1, 1: 0.2, 2: 0.7}), rx_pc)
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)

        data = np.array([[0], [1], [2]])
        rx_ll = rx_pc.log_likelihood(data)
        np_ll = np_pc.log_likelihood(data)
        np.testing.assert_allclose(rx_ll, np_ll)

    def test_uniform_circuit(self):
        x = Continuous("x")
        rx_pc = RXProbabilisticCircuit()
        interval = SimpleInterval.from_data(0, 10, Bound.CLOSED, Bound.CLOSED)
        l1 = leaf(UniformDistribution(variable=x, interval=interval), rx_pc)
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)

        data = np.array([[0], [5], [10]])
        rx_ll = rx_pc.log_likelihood(data)
        np_ll = np_pc.log_likelihood(data)
        np.testing.assert_allclose(rx_ll, np_ll)

    def test_mode(self):
        x = Continuous("x")
        rx_pc = RXProbabilisticCircuit()
        l1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rx_pc)
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)
        
        rx_mode, rx_ll = rx_pc.log_mode()
        np_mode, np_ll = np_pc.log_mode()
        self.assertEqual(rx_mode, np_mode)
        self.assertAlmostEqual(rx_ll, np_ll)

    def test_truncation(self):
        from random_events.interval import closed
        from random_events.product_algebra import SimpleEvent
        x = Continuous("x")
        rx_pc = RXProbabilisticCircuit()
        l1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rx_pc)
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)

        event = SimpleEvent.from_data({x: closed(-1, 1)}).as_composite_set()
        rx_tr, rx_lp = rx_pc.log_truncated(event)
        np_tr, np_lp = np_pc.log_truncated(event)

        self.assertAlmostEqual(rx_lp, np_lp)

        # Test LL of truncated model
        data = np.array([[0], [0.5]])
        rx_tr_ll = rx_tr.log_likelihood(data)
        np_tr_ll = np_tr.log_likelihood(data)
        np.testing.assert_allclose(rx_tr_ll, np_tr_ll)

    def test_marginal(self):
        x = Continuous("x")
        y = Continuous("y")
        rx_pc = RXProbabilisticCircuit()
        l1 = leaf(GaussianDistribution(variable=x, location=0, scale=1), rx_pc)
        l2 = leaf(GaussianDistribution(variable=y, location=2, scale=1), rx_pc)
        p1 = ProductUnit(probabilistic_circuit=rx_pc)
        p1.add_subcircuit(l1)
        p1.add_subcircuit(l2)
        
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)
        
        rx_marg = rx_pc.marginal([x])
        np_marg = np_pc.marginal([x])
        
        data = np.array([[0]])
        rx_ll = rx_marg.log_likelihood(data)
        np_ll = np_marg.log_likelihood(data)
        np.testing.assert_allclose(rx_ll, np_ll)

if __name__ == '__main__':
    unittest.main()
