"""
The gradient of the summed log-likelihood of a batch with respect to the parameters of a
layered circuit, on the array backends that differentiate.
"""

from __future__ import annotations

# %% imports
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import jax
import jax.numpy as jnp
import numpy as np
from array_api_compat import is_jax_array, is_torch_array
from typing_extensions import Any, Callable, Self

from probabilistic_model.learning.layered.exceptions import (
    UndifferentiableBackendError,
)
from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    BackendArray,
)
from probabilistic_model.probabilistic_circuit.tensorized.has_parameter_arrays import (
    HasParameterArrays,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import (
    Layer,
)


# %% parameter transformations


@dataclass(kw_only=True)
class ParameterTransformation(ABC):
    """
    A differentiable map applied to the parameters of a circuit before its likelihood is
    evaluated, so that the gradient is taken through it.
    """

    @abstractmethod
    def transformed(
        self, *, root: Layer, parameters: list[BackendArray]
    ) -> list[BackendArray]:
        """
        :param root: The root layer of the circuit.
        :param parameters: Arrays in the order of :attr:`Layer.circuit_parameters`.
        :return: The arrays the likelihood is evaluated with.
        """
        raise NotImplementedError


@dataclass(kw_only=True)
class IdentityTransformation(ParameterTransformation):
    """
    Evaluates the likelihood with the parameters themselves.
    """

    def transformed(
        self, *, root: Layer, parameters: list[BackendArray]
    ) -> list[BackendArray]:
        return parameters


# %% gradients


@dataclass(kw_only=True)
class GradientOfBatch:
    """
    The summed log-likelihood of a batch and its gradient.
    """

    log_likelihood: float
    """
    The log-likelihood of the batch, summed over its rows.
    """

    gradients: list[np.ndarray]
    """
    The gradient with respect to every array of :attr:`Layer.circuit_parameters`.
    """

    def without_impossible_rows(self) -> GradientOfBatch:
        """
        :return: The gradient with the undefined contributions of rows the circuit gives
            probability zero replaced by zero, so that such rows have no flow.
        """
        return GradientOfBatch(
            log_likelihood=self.log_likelihood,
            gradients=[
                np.nan_to_num(gradient, nan=0.0, posinf=0.0, neginf=0.0)
                for gradient in self.gradients
            ],
        )


@dataclass(kw_only=True)
class LogLikelihoodGradient(ABC):
    """
    Computes :class:`GradientOfBatch` for the circuit rooted at a layer.
    """

    root: Layer
    """
    The root layer of the circuit.
    """

    transformation: ParameterTransformation = field(
        default_factory=IdentityTransformation
    )
    """
    The map applied to the parameters before the likelihood is evaluated.
    """

    def summed_log_likelihood(
        self, parameters: list[BackendArray], data: BackendArray
    ) -> BackendArray:
        """
        :param parameters: Arrays in the order of :attr:`Layer.circuit_parameters`.
        :param data: The encoded rows of the batch.
        :return: The summed log-likelihood of the rows.
        """
        transformed = self.transformation.transformed(
            root=self.root, parameters=parameters
        )
        log_likelihoods = self.root.copy_with_circuit_parameters(
            transformed
        ).log_likelihood_of_nodes(events=data)
        return log_likelihoods[:, 0].sum()

    @abstractmethod
    def compute(self, *, data: BackendArray) -> GradientOfBatch:
        """
        :param data: The encoded rows of the batch, on the backend of the parameters.
        :return: The summed log-likelihood of the batch and its gradient at the current
            parameters of :attr:`root`.
        """
        raise NotImplementedError

    @classmethod
    def from_root(
        cls,
        *,
        root: Layer,
        transformation: ParameterTransformation | None = None,
    ) -> Self:
        """
        :param root: The root layer of the circuit.
        :param transformation: The map applied to the parameters, none if ``None``.
        :return: The gradient computation of the backend that stores the parameters.
        :raises UndifferentiableBackendError: If that backend does not differentiate.
        """
        transformation = (
            IdentityTransformation() if transformation is None else transformation
        )
        parameter = root.circuit_parameters[0]
        if is_jax_array(parameter):
            return JaxLogLikelihoodGradient(root=root, transformation=transformation)
        if is_torch_array(parameter):
            return TorchLogLikelihoodGradient(root=root, transformation=transformation)
        raise UndifferentiableBackendError(parameter_type=type(parameter))


@dataclass(kw_only=True)
class JaxLogLikelihoodGradient(LogLikelihoodGradient):
    """
    The gradient by :func:`jax.value_and_grad`, compiled once per circuit structure.
    """

    compiled: Callable[..., Any] = field(init=False)
    """
    The compiled function from parameters and data to the value and the gradient.
    """

    def __post_init__(self) -> None:
        self.compiled = jax.jit(jax.value_and_grad(self.summed_log_likelihood))

    def compute(self, *, data: BackendArray) -> GradientOfBatch:
        value, gradients = self.compiled(
            self.root.circuit_parameters, jnp.asarray(data)
        )
        return GradientOfBatch(
            log_likelihood=float(value),
            gradients=[np.asarray(gradient) for gradient in gradients],
        ).without_impossible_rows()


@dataclass(kw_only=True)
class TorchLogLikelihoodGradient(LogLikelihoodGradient):
    """
    The gradient by torch's automatic differentiation.
    """

    def compute(self, *, data: BackendArray) -> GradientOfBatch:
        parameters = [
            parameter.detach().clone().requires_grad_(True)
            for parameter in self.root.circuit_parameters
        ]
        value = self.summed_log_likelihood(parameters, data)
        value.backward()
        return GradientOfBatch(
            log_likelihood=float(value.detach()),
            gradients=[
                HasParameterArrays.host_copy(parameter.grad) for parameter in parameters
            ],
        ).without_impossible_rows()
