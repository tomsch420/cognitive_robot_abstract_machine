"""
LearnSPN structure learning of :cite:`gens2013learning`: split the variables into groups
the data shows independent below a product, else cluster the rows below a sum, and
recurse.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass

import numpy as np
from random_events.variable import Continuous, Symbolic, Variable
from scipy.sparse import csr_array
from scipy.sparse.csgraph import connected_components
from scipy.stats import chi2
from sklearn.cluster import KMeans
from sortedcontainers import SortedSet

from probabilistic_model.adapters.circuit_representations import CircuitRepresentations
from probabilistic_model.distributions.distributions import SymbolicDistribution
from probabilistic_model.distributions.gaussian import GaussianDistribution
from probabilistic_model.learning.layered.exceptions import (
    UnsupportedStructureVariableError,
)
from probabilistic_model.learning.layered.mutual_information import (
    DiscretizedData,
    PairwiseMutualInformation,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
    ProductUnit,
    SumUnit,
    Unit,
    leaf,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.gaussian_layer import (
    GaussianLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.tensorized.symbolic_encoding import (
    SymbolicEncoding,
)
from probabilistic_model.utils import MissingDict


# %% LearnSPN


@dataclass(kw_only=True)
class LearnSPN:
    """
    Learns the structure and the parameters of a tree-shaped circuit top down.

    Every call on a set of rows and variables returns, in this order of preference, a
    leaf for a single variable, a fully factorized product for too few rows, a product
    over the groups of variables that the G-test of independence separates, or a sum
    over two clusters of the rows weighted by their sizes.
    """

    variables: SortedSet
    """
    The variables, one per column of the data.
    """

    significance_level: float = 0.001
    """
    The p-value of the G-test below which two variables count as dependent.
    """

    minimum_rows: int = 50
    """
    The number of rows below which the variables are factorized fully.
    """

    pseudocount: float = 0.1
    """
    The count added to every state of a categorical leaf.
    """

    number_of_bins: int = 10
    """
    The number of quantile bins a continuous variable is cut into for the G-test.
    """

    seed: int = 0
    """
    The seed of the clustering.
    """

    def build_circuit(self, *, data: np.ndarray) -> LayeredProbabilisticCircuit:
        """
        :param data: The rows, whose symbolic values are hashes of domain elements.
        :return: The learned circuit, as a layered circuit with host parameters.
        """
        circuit = ProbabilisticCircuit()
        self.learn(
            circuit=circuit,
            data=data,
            rows=np.arange(len(data)),
            columns=np.arange(len(self.variables)),
        )
        return CircuitRepresentations().convert(circuit, LayeredProbabilisticCircuit)

    def learn(
        self,
        *,
        circuit: ProbabilisticCircuit,
        data: np.ndarray,
        rows: np.ndarray,
        columns: np.ndarray,
    ) -> Unit:
        """
        :param circuit: The circuit the units are added to.
        :param data: All rows.
        :param rows: The indices of the rows to learn from.
        :param columns: The indices of the variables to learn over.
        :return: The root unit of the learned subcircuit.
        """
        if len(columns) == 1:
            return self.leaf_unit(
                circuit=circuit, values=data[rows, columns[0]], column=int(columns[0])
            )
        if len(rows) < self.minimum_rows:
            return self.product_unit(
                circuit=circuit,
                children=[
                    self.leaf_unit(
                        circuit=circuit, values=data[rows, column], column=int(column)
                    )
                    for column in columns
                ],
            )
        groups = self.independent_groups(data=data, rows=rows, columns=columns)
        if len(groups) > 1:
            return self.product_unit(
                circuit=circuit,
                children=[
                    self.learn(circuit=circuit, data=data, rows=rows, columns=group)
                    for group in groups
                ],
            )
        clusters = self.clusters(data=data, rows=rows, columns=columns)
        if len(clusters) < 2:
            return self.product_unit(
                circuit=circuit,
                children=[
                    self.learn(
                        circuit=circuit,
                        data=data,
                        rows=rows,
                        columns=np.array([column]),
                    )
                    for column in columns
                ],
            )
        unit = SumUnit(probabilistic_circuit=circuit)
        for cluster in clusters:
            unit.add_subcircuit(
                self.learn(circuit=circuit, data=data, rows=cluster, columns=columns),
                np.log(len(cluster) / len(rows)),
            )
        return unit

    @staticmethod
    def product_unit(*, circuit: ProbabilisticCircuit, children: list[Unit]) -> Unit:
        """
        :param circuit: The circuit the unit is added to.
        :param children: The factors.
        :return: The product of the factors.
        """
        unit = ProductUnit(probabilistic_circuit=circuit)
        for child in children:
            unit.add_subcircuit(child)
        return unit

    def independent_groups(
        self, *, data: np.ndarray, rows: np.ndarray, columns: np.ndarray
    ) -> list[np.ndarray]:
        """
        :param data: All rows.
        :param rows: The indices of the rows.
        :param columns: The indices of the variables.
        :return: The connected components of the graph whose edges join the variables
            the G-test finds dependent.
        """
        discretized = DiscretizedData(
            variables=SortedSet(self.variables[column] for column in columns),
            data=data[np.ix_(rows, columns)],
            number_of_bins=self.number_of_bins,
        )
        mutual_information = PairwiseMutualInformation(
            discretized=discretized
        ).compute()
        # the G statistic is twice the number of rows times the mutual information in nats
        statistic = 2 * len(rows) * mutual_information
        freedom = np.outer(
            discretized.number_of_states - 1, discretized.number_of_states - 1
        )
        p_values = chi2.sf(statistic, np.maximum(freedom, 1))
        dependent = (p_values < self.significance_level) & (freedom > 0)
        np.fill_diagonal(dependent, False)
        number_of_components, labels = connected_components(
            csr_array(dependent), directed=False
        )
        return [columns[labels == label] for label in range(number_of_components)]

    def clusters(
        self, *, data: np.ndarray, rows: np.ndarray, columns: np.ndarray
    ) -> list[np.ndarray]:
        """
        :param data: All rows.
        :param rows: The indices of the rows.
        :param columns: The indices of the variables.
        :return: The non-empty clusters of two-means clustering of the rows, with
            symbolic values one-hot encoded and continuous values standardized.
        """
        features = np.concatenate(
            [
                self.clustering_features(values=data[rows, column], column=int(column))
                for column in columns
            ],
            axis=1,
        )
        labels = KMeans(n_clusters=2, n_init=1, random_state=self.seed).fit_predict(
            features
        )
        return [rows[labels == label] for label in range(2) if np.any(labels == label)]

    def clustering_features(self, *, values: np.ndarray, column: int) -> np.ndarray:
        """
        :param values: The values of one variable.
        :param column: The index of the variable.
        :return: The features of the values that the clustering compares.
        """
        variable: Variable = self.variables[column]
        if isinstance(variable, Symbolic):
            indices = SymbolicEncoding(variable).indices_of_hashes(values).astype(int)
            return np.eye(len(variable.domain.simple_sets))[indices]
        scale = max(float(values.std()), GaussianLayer.minimum_scale)
        return ((values - values.mean()) / scale)[:, None]

    def leaf_unit(
        self, *, circuit: ProbabilisticCircuit, values: np.ndarray, column: int
    ) -> Unit:
        """
        :param circuit: The circuit the unit is added to.
        :param values: The values of the variable.
        :param column: The index of the variable.
        :return: The maximum likelihood categorical leaf, with :attr:`pseudocount`, or
            Gaussian leaf of the values.
        :raises UnsupportedStructureVariableError: If the variable is neither symbolic
            nor continuous.
        """
        variable: Variable = self.variables[column]
        if isinstance(variable, Symbolic):
            encoding = SymbolicEncoding(variable)
            counts = (
                np.bincount(
                    encoding.indices_of_hashes(values).astype(int),
                    minlength=len(encoding.hashes),
                )
                + self.pseudocount
            )
            probabilities = counts / counts.sum()
            return leaf(
                SymbolicDistribution(
                    variable=variable,
                    probabilities=MissingDict(
                        float,
                        {
                            int(hash_value): float(probability)
                            for hash_value, probability in zip(
                                encoding.hashes, probabilities
                            )
                        },
                    ),
                ),
                circuit,
            )
        if isinstance(variable, Continuous):
            return leaf(
                GaussianDistribution(
                    variable=variable,
                    location=float(values.mean()),
                    scale=max(float(values.std()), GaussianLayer.minimum_scale),
                ),
                circuit,
            )
        raise UnsupportedStructureVariableError(variable=variable)
