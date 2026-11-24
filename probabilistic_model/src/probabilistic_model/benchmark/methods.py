"""
The learning methods and baselines of the circuit learning benchmark, each configured to
produce a layered circuit from the training rows of a dataset.
"""

from __future__ import annotations

# %% imports
import dataclasses
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import StrEnum

import jax.numpy as jnp
import numpy as np
import pandas as pd

from probabilistic_model.adapters.circuit_representations import CircuitRepresentations
from probabilistic_model.benchmark.datasets import BenchmarkDataset
from probabilistic_model.learning.structure.jpt.jpt import JointProbabilityTree
from probabilistic_model.learning.structure.jpt.variables import (
    infer_variables_from_dataframe,
)
from probabilistic_model.learning.layered.dropout import (
    MiniBatchGradientDescent,
    ProbabilisticDropout,
)
from probabilistic_model.learning.layered.expectation_maximization import (
    CircuitExpectationMaximization,
)
from probabilistic_model.learning.layered.hidden_chow_liu_tree import (
    HiddenChowLiuTree,
)
from probabilistic_model.learning.layered.latent_variable_distillation import (
    LatentVariableDistillation,
)
from probabilistic_model.learning.layered.learn_spn import LearnSPN
from probabilistic_model.learning.layered.strudel import Strudel
from probabilistic_model.learning.layered.pruning_and_growing import (
    PruningAndGrowing,
)
from probabilistic_model.learning.layered.statistics_adjustment import (
    StateCountSmoothing,
)
from probabilistic_model.learning.layered.regularization import (
    DataSoftening,
    EntropyRegularization,
)
from probabilistic_model.learning.structure.region_graph.region_graph import RegionGraph
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)


# %% names


class MethodName(StrEnum):
    """
    The methods of the benchmark.
    """

    RAT_SPN_GRADIENT_DESCENT = "rat_spn_gradient_descent"
    RAT_SPN_DROPOUT = "rat_spn_dropout"
    RAT_SPN_EXPECTATION_MAXIMIZATION = "rat_spn_expectation_maximization"
    RAT_SPN_MINI_BATCH_EXPECTATION_MAXIMIZATION = "rat_spn_mini_batch_em"
    RAT_SPN_ANEMONE = "rat_spn_anemone"
    JOINT_PROBABILITY_TREE = "joint_probability_tree"
    HIDDEN_CHOW_LIU_TREE = "hclt"
    REGULARIZED_HIDDEN_CHOW_LIU_TREE = "hclt_regularized"
    HIDDEN_CHOW_LIU_TREE_ANEMONE = "hclt_anemone"
    PRUNING_AND_GROWING = "pruning_and_growing"
    LEARN_SPN = "learn_spn"
    STRUDEL = "strudel"
    LATENT_VARIABLE_DISTILLATION = "latent_variable_distillation"


# %% methods


@dataclass(kw_only=True)
class BenchmarkMethod(ABC):
    """
    A way of learning a circuit from the training rows of a dataset.
    """

    seed: int = 0
    """
    The seed of the random initialization and the order of the rows.
    """

    @property
    @abstractmethod
    def name(self) -> MethodName:
        """
        :return: The name of the method.
        """
        raise NotImplementedError

    @abstractmethod
    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        """
        :param dataset: The dataset, of which only the training and validation rows are
            read.
        :return: The learned circuit.
        """
        raise NotImplementedError


@dataclass(kw_only=True)
class RandomTensorizedSumProductNetwork(BenchmarkMethod, ABC):
    """
    A RAT-SPN :cite:`peharz2020random` built from a random region graph.
    """

    depth: int = 3
    """
    The number of splits of a region graph repetition, at most the base two logarithm of
    the number of variables.
    """

    repetitions: int = 10
    """
    The number of random repetitions of the region graph.
    """

    units: int = 10
    """
    The number of input distributions of a leaf region and sum units of a region.
    """

    def random_circuit(
        self, *, dataset: BenchmarkDataset
    ) -> LayeredProbabilisticCircuit:
        """
        :param dataset: The dataset.
        :return: The randomly initialized RAT-SPN over the variables of the dataset,
            with jax parameters.
        """
        depth = min(self.depth, max(int(np.log2(len(dataset.variables))), 1))
        return (
            RegionGraph(
                variables=dataset.variables,
                partitions=2,
                depth=depth,
                repetitions=self.repetitions,
                seed=self.seed,
            )
            .create_random_region_graph()
            .to_probabilistic_circuit(input_units=self.units, sum_units=self.units)
            .to_namespace(jnp)
        )


@dataclass(kw_only=True)
class RandomTensorizedSumProductNetworkByGradientDescent(
    RandomTensorizedSumProductNetwork
):
    """
    A RAT-SPN trained generatively by mini-batch gradient descent, with or without
    probabilistic dropout.
    """

    epochs: int = 30
    """
    The number of passes over the data.
    """

    dropout: ProbabilisticDropout = field(default_factory=ProbabilisticDropout)
    """
    The dropout during training.
    """

    @property
    def name(self) -> MethodName:
        if self.dropout.input_probability or self.dropout.sum_probability:
            return MethodName.RAT_SPN_DROPOUT
        return MethodName.RAT_SPN_GRADIENT_DESCENT

    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        circuit = self.random_circuit(dataset=dataset)
        return MiniBatchGradientDescent(
            epochs=self.epochs, dropout=self.dropout, seed=self.seed
        ).fit(circuit=circuit, data=dataset.encoded(frame=dataset.train))


@dataclass(kw_only=True)
class RandomTensorizedSumProductNetworkByExpectationMaximization(
    RandomTensorizedSumProductNetwork
):
    """
    A RAT-SPN trained by one of the expectation maximization variants.
    """

    method_name: MethodName
    """
    The name the variant is reported under.
    """

    expectation_maximization: CircuitExpectationMaximization
    """
    The variant.
    """

    @property
    def name(self) -> MethodName:
        return self.method_name

    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        circuit = self.random_circuit(dataset=dataset)
        return self.expectation_maximization.fit(
            circuit=circuit, data=dataset.encoded(frame=dataset.train)
        )


@dataclass(kw_only=True)
class JointProbabilityTreeBaseline(BenchmarkMethod):
    """
    A joint probability tree :cite:`nyga2023joint`.
    """

    minimum_samples_per_leaf: float = 0.1
    """
    The fraction of the training rows a leaf holds at least.
    """

    @property
    def name(self) -> MethodName:
        return MethodName.JOINT_PROBABILITY_TREE

    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        variables = infer_variables_from_dataframe(
            pd.concat([dataset.train, dataset.validation, dataset.test])
        )
        circuit = JointProbabilityTree(
            minimum_samples_per_leaf=self.minimum_samples_per_leaf
        ).fit(dataset.train, variables)
        return CircuitRepresentations().convert(circuit, LayeredProbabilisticCircuit)


@dataclass(kw_only=True)
class HiddenChowLiuTreeMethod(BenchmarkMethod):
    """
    A hidden Chow-Liu tree circuit trained by mini-batch and then full-batch expectation
    maximization, the schedule of :cite:`liu2021tractable`, section 4.3.
    """

    method_name: MethodName = MethodName.HIDDEN_CHOW_LIU_TREE
    """
    The name the configuration is reported under.
    """

    hidden_states: int = 12
    """
    The number of states of every latent variable.
    """

    mini_batch: CircuitExpectationMaximization = field(
        default_factory=lambda: CircuitExpectationMaximization.mini_batch(epochs=20)
    )
    """
    The first phase of training.
    """

    full_batch: CircuitExpectationMaximization = field(
        default_factory=lambda: CircuitExpectationMaximization(epochs=20)
    )
    """
    The second phase of training.
    """

    @property
    def name(self) -> MethodName:
        return self.method_name

    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        data = dataset.encoded(frame=dataset.train)
        circuit = HiddenChowLiuTree(
            variables=dataset.variables,
            hidden_states=self.hidden_states,
            seed=self.seed,
        ).build_circuit(data=data)
        self.mini_batch.fit(circuit=circuit, data=data)
        return self.full_batch.fit(circuit=circuit, data=data)


@dataclass(kw_only=True)
class PruningAndGrowingMethod(BenchmarkMethod):
    """
    Pruning and growing :cite:`dang2022sparse` from a trained hidden Chow-Liu tree
    circuit, stopped by the validation rows.
    """

    start: HiddenChowLiuTreeMethod = field(default_factory=HiddenChowLiuTreeMethod)
    """
    How the initial circuit is learned.
    """

    pruning_and_growing: PruningAndGrowing = field(
        default_factory=lambda: PruningAndGrowing(
            parameter_learning=CircuitExpectationMaximization.mini_batch(epochs=10)
        )
    )
    """
    The structure learning.
    """

    @property
    def name(self) -> MethodName:
        return MethodName.PRUNING_AND_GROWING

    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        circuit = self.start.learn(dataset=dataset)
        return self.pruning_and_growing.fit(
            circuit=circuit,
            data=dataset.encoded(frame=dataset.train),
            validation_data=dataset.encoded(frame=dataset.validation),
        )


@dataclass(kw_only=True)
class LearnSPNMethod(BenchmarkMethod):
    """
    LearnSPN :cite:`gens2013learning`.
    """

    @property
    def name(self) -> MethodName:
        return MethodName.LEARN_SPN

    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        return LearnSPN(variables=dataset.variables, seed=self.seed).build_circuit(
            data=dataset.encoded(frame=dataset.train)
        )


@dataclass(kw_only=True)
class StrudelMethod(BenchmarkMethod):
    """
    Strudel :cite:`dang2020strudel`, stopped by the validation rows; symbolic variables
    only.
    """

    maximum_splits: int = 1000
    """
    The largest number of splits.
    """

    @property
    def name(self) -> MethodName:
        return MethodName.STRUDEL

    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        return Strudel(
            variables=dataset.variables, maximum_splits=self.maximum_splits
        ).build_circuit(
            data=dataset.encoded(frame=dataset.train),
            validation_data=dataset.encoded(frame=dataset.validation),
        )


@dataclass(kw_only=True)
class LatentVariableDistillationMethod(BenchmarkMethod):
    """
    Latent variable distillation :cite:`liu2023scaling` of a hidden Chow-Liu tree
    circuit with a masked autoencoder trained on the training rows.
    """

    @property
    def name(self) -> MethodName:
        return MethodName.LATENT_VARIABLE_DISTILLATION

    def learn(self, *, dataset: BenchmarkDataset) -> LayeredProbabilisticCircuit:
        return LatentVariableDistillation(
            variables=dataset.variables, seed=self.seed
        ).build_circuit(data=dataset.encoded(frame=dataset.train))


# %% all methods


@dataclass(kw_only=True)
class BenchmarkMethods:
    """
    Every method of the benchmark in its default configuration.
    """

    methods: list[BenchmarkMethod] = field(
        default_factory=lambda: [
            RandomTensorizedSumProductNetworkByGradientDescent(),
            RandomTensorizedSumProductNetworkByGradientDescent(
                dropout=ProbabilisticDropout(input_probability=0.1, sum_probability=0.1)
            ),
            RandomTensorizedSumProductNetworkByExpectationMaximization(
                method_name=MethodName.RAT_SPN_EXPECTATION_MAXIMIZATION,
                expectation_maximization=CircuitExpectationMaximization(epochs=30),
            ),
            RandomTensorizedSumProductNetworkByExpectationMaximization(
                method_name=MethodName.RAT_SPN_MINI_BATCH_EXPECTATION_MAXIMIZATION,
                expectation_maximization=CircuitExpectationMaximization.mini_batch(
                    epochs=30
                ),
            ),
            RandomTensorizedSumProductNetworkByExpectationMaximization(
                method_name=MethodName.RAT_SPN_ANEMONE,
                expectation_maximization=CircuitExpectationMaximization.anemone(
                    epochs=30
                ),
            ),
            JointProbabilityTreeBaseline(),
            HiddenChowLiuTreeMethod(),
            HiddenChowLiuTreeMethod(
                method_name=MethodName.REGULARIZED_HIDDEN_CHOW_LIU_TREE,
                mini_batch=dataclasses.replace(
                    CircuitExpectationMaximization.mini_batch(epochs=20),
                    data_softening=DataSoftening(keep_probability=0.998),
                    statistics_adjustments=[
                        StateCountSmoothing(),
                        EntropyRegularization(strength=0.001),
                    ],
                ),
                full_batch=CircuitExpectationMaximization(
                    epochs=20,
                    data_softening=DataSoftening(keep_probability=0.998),
                    statistics_adjustments=[
                        StateCountSmoothing(),
                        EntropyRegularization(strength=0.001),
                    ],
                ),
            ),
            HiddenChowLiuTreeMethod(
                method_name=MethodName.HIDDEN_CHOW_LIU_TREE_ANEMONE,
                mini_batch=CircuitExpectationMaximization.anemone(epochs=40),
                full_batch=CircuitExpectationMaximization(epochs=0),
            ),
            PruningAndGrowingMethod(),
            LearnSPNMethod(),
            StrudelMethod(),
            LatentVariableDistillationMethod(),
        ]
    )
    """
    The methods.
    """

    def find(self, *, name: MethodName) -> BenchmarkMethod:
        """
        :param name: The name of a method.
        :return: The method of that name.
        """
        return next(method for method in self.methods if method.name is name)
