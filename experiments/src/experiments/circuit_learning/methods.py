"""
The learning methods and baselines of the circuit learning benchmark, each producing a
layered circuit from the training rows of a dataset in a computation setting, iterative
methods training until their loss converges.
"""

from __future__ import annotations

# %% imports
import dataclasses
import enum
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import jax
import numpy as np
import optax
import pandas as pd

from experiments.circuit_learning.datasets import BenchmarkDataset
from experiments.circuit_learning.settings import (
    ArrayLibrary,
    BudgetedMemory,
    ComputationSetting,
    Device,
    EdgeRepresentation,
    FloatingPointType,
)
from probabilistic_model.adapters.circuit_representations import CircuitRepresentations
from probabilistic_model.learning.device_memory import DeviceMemory, HostMemory
from probabilistic_model.learning.gradient_descent import (
    AverageNegativeLogLikelihood,
    GradientDescent,
    JaxDeviceMemory,
)
from probabilistic_model.learning.jpt.jpt import JointProbabilityTree
from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
from probabilistic_model.learning.layered.dropout import (
    MiniBatchGradientDescent,
    ProbabilisticDropout,
)
from probabilistic_model.learning.layered.expectation_maximization import (
    Anemone,
    ExpectationMaximization,
    FullBatchExpectationMaximization,
    MiniBatchExpectationMaximization,
)
from probabilistic_model.learning.layered.hidden_chow_liu_tree import (
    HiddenChowLiuTree,
)
from probabilistic_model.learning.layered.latent_variable_distillation import (
    LatentVariableDistillation,
)
from probabilistic_model.learning.layered.learn_spn import LearnSPN
from probabilistic_model.learning.layered.pruning_and_growing import (
    PruningAndGrowing,
)
from probabilistic_model.learning.layered.regularization import (
    DataSoftening,
    EntropyRegularization,
)
from probabilistic_model.learning.layered.strudel import Strudel
from probabilistic_model.learning.region_graph.region_graph import RegionGraph
from probabilistic_model.learning.torch_gradient_descent import TorchGradientDescent
from probabilistic_model.learning.training_schedule import (
    Convergence,
    ConvergenceHistory,
    MiniBatches,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)

# %% names


class LearningMethod(enum.Enum):
    """
    A learning method of the benchmark, baselines included.
    """

    RAT_SPN_GRADIENT_DESCENT = "RAT-SPN, gradient descent"
    RAT_SPN_DROPOUT = "RAT-SPN, gradient descent with probabilistic dropout"
    RAT_SPN_EXPECTATION_MAXIMIZATION = "RAT-SPN, expectation maximization"
    RAT_SPN_MINI_BATCH_EXPECTATION_MAXIMIZATION = (
        "RAT-SPN, mini-batch expectation maximization"
    )
    RAT_SPN_ANEMONE = "RAT-SPN, Anemone"
    JOINT_PROBABILITY_TREE = "joint probability tree"
    HIDDEN_CHOW_LIU_TREE = "hidden Chow-Liu tree"
    REGULARIZED_HIDDEN_CHOW_LIU_TREE = (
        "hidden Chow-Liu tree, data softening and entropy regularization"
    )
    HIDDEN_CHOW_LIU_TREE_ANEMONE = "hidden Chow-Liu tree, Anemone"
    PRUNING_AND_GROWING = "pruning and growing"
    LEARN_SPN = "LearnSPN"
    STRUDEL = "Strudel"
    LATENT_VARIABLE_DISTILLATION = "latent variable distillation"


# %% training context


@dataclass(kw_only=True)
class LearnedCircuit:
    """
    A circuit a method learned and how its iterative training ended.
    """

    circuit: LayeredProbabilisticCircuit
    """
    The learned circuit, its parameters stored as the setting it was learned in says.
    """

    history: ConvergenceHistory | None
    """
    The losses of the epochs of the iterative training, ``None`` for methods without.
    """


@dataclass(kw_only=True)
class TrainingContext:
    """
    The limits a method trains within.
    """

    convergence: Convergence
    """
    When iterative training stops.
    """

    memory_budget_bytes: int
    """
    The most memory of its device a training step may allocate.
    """

    def device_memory(self, *, setting: ComputationSetting) -> DeviceMemory:
        """
        :param setting: The setting that trains.
        :return: The memory of its device within :attr:`memory_budget_bytes`.
        """
        memory = (
            JaxDeviceMemory(device=jax.devices("gpu")[0])
            if setting.uses_gpu
            else HostMemory()
        )
        return BudgetedMemory(memory=memory, budget_bytes=self.memory_budget_bytes)

    def rows_per_step(
        self,
        *,
        circuit: LayeredProbabilisticCircuit,
        setting: ComputationSetting,
        rows: np.ndarray,
    ) -> int:
        """
        :param circuit: The circuit, with host parameters.
        :param setting: The setting that trains.
        :param rows: The encoded training rows.
        :return: The most rows whose log-likelihood gradient fits in the memory budget,
            as XLA measures it for the circuit in jax with dense edges on the same
            device, at most all rows.
        """
        jax_setting = dataclasses.replace(
            setting,
            array_library=ArrayLibrary.JAX,
            edge_representation=EdgeRepresentation.DENSE,
        )
        placed = jax_setting.placed(circuit=circuit)
        step_memory = GradientDescent(
            placed.root, AverageNegativeLogLikelihood()
        ).step_memory(jax_setting.asarray(rows=placed.encoded(rows)))
        batch_size = MiniBatches.fitting_in_memory(
            memory=self.device_memory(setting=setting), step_memory=step_memory
        ).batch_size
        return min(batch_size, len(rows))


# %% methods


@dataclass(kw_only=True)
class BenchmarkMethod(ABC):
    """
    A way of learning a circuit from the training rows of a dataset.
    """

    seed: int = 69
    """
    The seed of the structure, the initial parameters and the order of the rows.
    """

    @property
    @abstractmethod
    def name(self) -> LearningMethod:
        """
        :return: The name of the method.
        """

    @abstractmethod
    def settings(self) -> list[ComputationSetting]:
        """
        :return: The settings the method can learn in, the one it is measured in
            first leading.
        """

    def supports(self, *, dataset: BenchmarkDataset) -> bool:
        """
        :param dataset: A dataset.
        :return: Whether the method learns from the kinds of variables of the dataset.
        """
        return True

    @abstractmethod
    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        """
        :param dataset: The dataset, of which only the training and validation rows are
            read.
        :param setting: The setting to learn in, one of :meth:`settings`.
        :param context: The limits of training.
        :return: The learned circuit.
        """


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
        :return: The randomly initialized RAT-SPN over the variables of the dataset.
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
        )


@dataclass(kw_only=True)
class RandomTensorizedSumProductNetworkByGradientDescent(
    RandomTensorizedSumProductNetwork
):
    """
    A RAT-SPN trained generatively by mini-batch gradient descent with Adam, in batches
    of the most rows that fit in memory.
    """

    learning_rate: float = 1e-2
    """
    The learning rate of Adam.
    """

    @property
    def name(self) -> LearningMethod:
        return LearningMethod.RAT_SPN_GRADIENT_DESCENT

    def settings(self) -> list[ComputationSetting]:
        return ComputationSetting.grid(
            array_libraries=(ArrayLibrary.JAX, ArrayLibrary.TORCH)
        )

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        host = self.random_circuit(dataset=dataset)
        rows = dataset.encoded(frame=dataset.train)
        mini_batches = MiniBatches(
            batch_size=context.rows_per_step(circuit=host, setting=setting, rows=rows),
            seed=self.seed,
        )
        circuit = setting.placed(circuit=host)
        data = setting.asarray(rows=circuit.encoded(rows))
        if setting.array_library is ArrayLibrary.TORCH:
            trainer = TorchGradientDescent(
                circuit.root,
                AverageNegativeLogLikelihood(),
                learning_rate=self.learning_rate,
            )
        else:
            trainer = GradientDescent(
                circuit.root,
                AverageNegativeLogLikelihood(),
                optax.adam(self.learning_rate),
            )
        run = trainer.minimize_until_converged(
            data, convergence=context.convergence, mini_batches=mini_batches
        )
        circuit.root = run.trained_root
        return LearnedCircuit(circuit=circuit, history=run.history)


@dataclass(kw_only=True)
class RandomTensorizedSumProductNetworkByDropout(RandomTensorizedSumProductNetwork):
    """
    A RAT-SPN trained generatively by mini-batch gradient descent under probabilistic
    dropout, in batches of the most rows that fit in memory.
    """

    dropout: ProbabilisticDropout = field(
        default_factory=lambda: ProbabilisticDropout(
            input_probability=0.1, sum_probability=0.1
        )
    )
    """
    The dropout during training.
    """

    learning_rate: float = 1e-2
    """
    The learning rate of Adam.
    """

    @property
    def name(self) -> LearningMethod:
        return LearningMethod.RAT_SPN_DROPOUT

    def settings(self) -> list[ComputationSetting]:
        return ComputationSetting.grid(array_libraries=(ArrayLibrary.JAX,))

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        host = self.random_circuit(dataset=dataset)
        rows = dataset.encoded(frame=dataset.train)
        trainer = MiniBatchGradientDescent(
            convergence=context.convergence,
            batch_size=context.rows_per_step(circuit=host, setting=setting, rows=rows),
            optimizer=optax.adam(self.learning_rate),
            dropout=self.dropout,
            seed=self.seed,
        )
        circuit = trainer.fit(circuit=setting.placed(circuit=host), data=rows)
        return LearnedCircuit(circuit=circuit, history=trainer.history)


@dataclass(kw_only=True)
class RandomTensorizedSumProductNetworkByExpectationMaximization(
    RandomTensorizedSumProductNetwork
):
    """
    A RAT-SPN trained by one of the expectation maximization variants.
    """

    method_name: LearningMethod
    """
    The name the variant is reported under.
    """

    expectation_maximization: ExpectationMaximization
    """
    The variant.
    """

    @property
    def name(self) -> LearningMethod:
        return self.method_name

    def settings(self) -> list[ComputationSetting]:
        return ComputationSetting.grid(
            array_libraries=(ArrayLibrary.JAX, ArrayLibrary.TORCH)
        )

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        host = self.random_circuit(dataset=dataset)
        rows = dataset.encoded(frame=dataset.train)
        expectation_maximization = dataclasses.replace(
            self.expectation_maximization,
            convergence=context.convergence,
            evaluation_batch_size=context.rows_per_step(
                circuit=host, setting=setting, rows=rows
            ),
            seed=self.seed,
        )
        circuit = expectation_maximization.fit(
            circuit=setting.placed(circuit=host), data=rows
        )
        return LearnedCircuit(circuit=circuit, history=expectation_maximization.history)


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
    def name(self) -> LearningMethod:
        return LearningMethod.JOINT_PROBABILITY_TREE

    def settings(self) -> list[ComputationSetting]:
        return [ComputationSetting.host()]

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        variables = infer_variables_from_dataframe(
            pd.concat([dataset.train, dataset.validation, dataset.test])
        )
        tree = JointProbabilityTree(
            minimum_samples_per_leaf=self.minimum_samples_per_leaf
        ).fit(dataset.train, variables)
        return LearnedCircuit(
            circuit=CircuitRepresentations().convert(
                tree, LayeredProbabilisticCircuit
            ),
            history=None,
        )


@dataclass(kw_only=True)
class HiddenChowLiuTreeMethod(BenchmarkMethod):
    """
    A hidden Chow-Liu tree circuit trained by mini-batch and then full-batch expectation
    maximization, the schedule of :cite:`liu2021tractable`, section 4.3, each phase
    until convergence.
    """

    method_name: LearningMethod = LearningMethod.HIDDEN_CHOW_LIU_TREE
    """
    The name the configuration is reported under.
    """

    hidden_states: int = 12
    """
    The number of states of every latent variable.
    """

    mini_batch: ExpectationMaximization = field(
        default_factory=MiniBatchExpectationMaximization
    )
    """
    The first phase of training.
    """

    full_batch: ExpectationMaximization | None = field(
        default_factory=FullBatchExpectationMaximization
    )
    """
    The second phase of training, none if ``None``.
    """

    @property
    def name(self) -> LearningMethod:
        return self.method_name

    def settings(self) -> list[ComputationSetting]:
        return ComputationSetting.grid(
            array_libraries=(ArrayLibrary.JAX, ArrayLibrary.TORCH)
        )

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        rows = dataset.encoded(frame=dataset.train)
        host = HiddenChowLiuTree(
            variables=dataset.variables,
            hidden_states=self.hidden_states,
            seed=self.seed,
        ).build_circuit(data=rows)
        evaluation_batch_size = context.rows_per_step(
            circuit=host, setting=setting, rows=rows
        )
        circuit = setting.placed(circuit=host)
        first = dataclasses.replace(
            self.mini_batch,
            convergence=context.convergence,
            evaluation_batch_size=evaluation_batch_size,
            seed=self.seed,
        )
        first.fit(circuit=circuit, data=rows)
        if self.full_batch is None:
            return LearnedCircuit(circuit=circuit, history=first.history)
        remaining = context.convergence.maximum_seconds
        if remaining is not None:
            remaining = max(remaining - first.history.seconds, 0.0)
        second = dataclasses.replace(
            self.full_batch,
            convergence=dataclasses.replace(
                context.convergence, maximum_seconds=remaining
            ),
            evaluation_batch_size=evaluation_batch_size,
            seed=self.seed,
        )
        second.fit(circuit=circuit, data=rows)
        return LearnedCircuit(
            circuit=circuit,
            history=ConvergenceHistory(
                epoch_losses=first.history.epoch_losses + second.history.epoch_losses,
                converged=second.history.converged,
                seconds=first.history.seconds + second.history.seconds,
            ),
        )


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
            parameter_learning=MiniBatchExpectationMaximization(epochs=10)
        )
    )
    """
    The structure learning.
    """

    @property
    def name(self) -> LearningMethod:
        return LearningMethod.PRUNING_AND_GROWING

    def settings(self) -> list[ComputationSetting]:
        return self.start.settings()

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        start = self.start.learn(dataset=dataset, setting=setting, context=context)
        circuit = self.pruning_and_growing.fit(
            circuit=start.circuit,
            data=dataset.encoded(frame=dataset.train),
            validation_data=dataset.encoded(frame=dataset.validation),
        )
        return LearnedCircuit(circuit=circuit, history=start.history)


@dataclass(kw_only=True)
class LearnSPNMethod(BenchmarkMethod):
    """
    LearnSPN :cite:`gens2013learning`.
    """

    @property
    def name(self) -> LearningMethod:
        return LearningMethod.LEARN_SPN

    def settings(self) -> list[ComputationSetting]:
        return [ComputationSetting.host()]

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        return LearnedCircuit(
            circuit=LearnSPN(variables=dataset.variables, seed=self.seed).build_circuit(
                data=dataset.encoded(frame=dataset.train)
            ),
            history=None,
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
    def name(self) -> LearningMethod:
        return LearningMethod.STRUDEL

    def settings(self) -> list[ComputationSetting]:
        return [ComputationSetting.host()]

    def supports(self, *, dataset: BenchmarkDataset) -> bool:
        return dataset.is_symbolic

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        return LearnedCircuit(
            circuit=Strudel(
                variables=dataset.variables, maximum_splits=self.maximum_splits
            ).build_circuit(
                data=dataset.encoded(frame=dataset.train),
                validation_data=dataset.encoded(frame=dataset.validation),
            ),
            history=None,
        )


@dataclass(kw_only=True)
class LatentVariableDistillationMethod(BenchmarkMethod):
    """
    Latent variable distillation :cite:`liu2023scaling` of a hidden Chow-Liu tree
    circuit with a masked autoencoder trained on the training rows; the autoencoder
    trains with torch and the circuit with jax, both on the GPU.
    """

    @property
    def name(self) -> LearningMethod:
        return LearningMethod.LATENT_VARIABLE_DISTILLATION

    def settings(self) -> list[ComputationSetting]:
        return [
            ComputationSetting(
                array_library=ArrayLibrary.JAX,
                device=Device.GPU,
                floating_point_type=FloatingPointType.FLOAT64,
                edge_representation=EdgeRepresentation.SPARSE,
            )
        ]

    def learn(
        self,
        *,
        dataset: BenchmarkDataset,
        setting: ComputationSetting,
        context: TrainingContext,
    ) -> LearnedCircuit:
        return LearnedCircuit(
            circuit=LatentVariableDistillation(
                variables=dataset.variables, seed=self.seed
            ).build_circuit(data=dataset.encoded(frame=dataset.train)),
            history=None,
        )


# %% all methods


@dataclass(kw_only=True)
class BenchmarkMethods:
    """
    Every method of the benchmark in its default configuration.
    """

    methods: list[BenchmarkMethod] = field(
        default_factory=lambda: [
            RandomTensorizedSumProductNetworkByGradientDescent(),
            RandomTensorizedSumProductNetworkByDropout(),
            RandomTensorizedSumProductNetworkByExpectationMaximization(
                method_name=LearningMethod.RAT_SPN_EXPECTATION_MAXIMIZATION,
                expectation_maximization=FullBatchExpectationMaximization(),
            ),
            RandomTensorizedSumProductNetworkByExpectationMaximization(
                method_name=LearningMethod.RAT_SPN_MINI_BATCH_EXPECTATION_MAXIMIZATION,
                expectation_maximization=MiniBatchExpectationMaximization(),
            ),
            RandomTensorizedSumProductNetworkByExpectationMaximization(
                method_name=LearningMethod.RAT_SPN_ANEMONE,
                expectation_maximization=Anemone(),
            ),
            JointProbabilityTreeBaseline(),
            HiddenChowLiuTreeMethod(),
            HiddenChowLiuTreeMethod(
                method_name=LearningMethod.REGULARIZED_HIDDEN_CHOW_LIU_TREE,
                mini_batch=MiniBatchExpectationMaximization(
                    data_softening=DataSoftening(keep_probability=0.998),
                    entropy_regularization=EntropyRegularization(strength=0.001),
                ),
                full_batch=FullBatchExpectationMaximization(
                    data_softening=DataSoftening(keep_probability=0.998),
                    entropy_regularization=EntropyRegularization(strength=0.001),
                ),
            ),
            HiddenChowLiuTreeMethod(
                method_name=LearningMethod.HIDDEN_CHOW_LIU_TREE_ANEMONE,
                mini_batch=Anemone(),
                full_batch=None,
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

    def find(self, *, name: LearningMethod) -> BenchmarkMethod:
        """
        :param name: The name of a method.
        :return: The method of that name.
        """
        return next(method for method in self.methods if method.name is name)
