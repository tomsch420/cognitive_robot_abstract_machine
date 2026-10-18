"""
The variables of a grasp model that describe the grasp, apart from those of its result.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

import numpy as np
from numpy.typing import NDArray
from random_events.variable import Variable
from typing_extensions import List

from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
)
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp

# %% the variables


class SurfaceGraspVariablePrefix(StrEnum):
    """
    How the names of a grasp model's variables begin.
    """

    GRASP = "SurfaceGrasp."
    """
    Every variable of the model.
    """

    RESULT = "SurfaceGrasp.result."
    """
    The variables of the grasp's result.
    """


@dataclass
class SurfaceGraspVariables:
    """
    The variables of a circuit over surface grasps that describe the grasp itself.
    """

    circuit: ProbabilisticCircuit
    """
    A circuit over surface grasps and their results.
    """

    @property
    def of_the_grasp(self) -> List[Variable]:
        """
        The circuit's variables of the grasp, in the circuit's order.
        """
        return [
            variable
            for variable in self.circuit.variables
            if variable.name.startswith(SurfaceGraspVariablePrefix.GRASP)
            and not variable.name.startswith(SurfaceGraspVariablePrefix.RESULT)
        ]

    @property
    def columns_of_the_grasp(self) -> List[int]:
        """
        Where the variables of the grasp are among all of the circuit's variables.
        """
        grasp_variables = set(self.of_the_grasp)
        return [
            index
            for index, variable in enumerate(self.circuit.variables)
            if variable in grasp_variables
        ]

    def points(self, grasps: List[SurfaceGrasp]) -> NDArray[np.float64]:
        """
        :param grasps: Grasps.
        :return: The grasps as points over :attr:`of_the_grasp`, one row each.
        """
        names = [
            variable.name.removeprefix(SurfaceGraspVariablePrefix.GRASP)
            for variable in self.of_the_grasp
        ]
        rows = [vars(grasp) for grasp in grasps]
        return np.array([[row[name] for name in names] for row in rows], dtype=float)
