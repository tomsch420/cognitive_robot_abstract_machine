from __future__ import annotations

import datetime
import logging
from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

import numpy as np
import pandas
import pandas as pd

from giskardpy.qp.qp_data_symbolic import QPDataSymbolic
from giskardpy.utils.utils import create_path

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    pass

date_str = datetime.datetime.now().strftime("%Yy-%mm-%dd--%Hh-%Mm-%Ss")


@dataclass
class QuadraticProgramDebugger:
    """
    This class is designed to help you debug Giskard's quadratic programs (QP) by using names of constraints and
    degrees of freedom to create panda arrays with names rows and columns.
    """

    qp_data_symbolic: QPDataSymbolic
    """
    The symbolic casadi expressions for computing the QP components.
    """
    current_solution: np.ndarray | None = field(default=None)
    """
    The solution of the QP, None if there is none.
    """
    direct_limits: pandas.DataFrame = field(init=False)
    """
    This panda array gives you insights into the decision variables of the QP.
    It contains columns for direct upper and lower bounds, last solution and weights.
    """
    equality_constraints: pandas.DataFrame = field(init=False)
    """
    This panda array gives insight in the equality constraints.
    It contains columns for the equality bounds, the result of the equality matrix * decision variables (without slack),
    and the slack, which is essentially how much the constraints are violated. 
    """
    equality_matrix: pandas.DataFrame = field(init=False)
    """
    Panda array representing the equality constraint matrix.
    """
    inequality_constraints: pandas.DataFrame = field(init=False)
    """
    Panda array giving insights into the inequality constraints.
    It contains columns for the inequality bounds, the result of the inequality matrix * decision variables (without slack),
    and the slack, which is essentially how much the constraints are violated. 
    """
    inequality_matrix: pandas.DataFrame = field(init=False)
    """
    Panda array representing the inequality constraint matrix.
    """

    def __post_init__(self):
        self.update(self.current_solution)

    def update(self, current_solution: np.ndarray):
        """
        Updates the debugger with a new solution.
        """
        self.current_solution = current_solution
        last_solution = (
            np.ones(self.qp_data_symbolic.box_lower_constraints.shape[0]) * np.nan
        )
        if self.current_solution is not None:
            last_solution[self.quadratic_weight_filter] = self.current_solution

        self.current_solution = last_solution
        self.create_direct_limits()
        self.create_equality_constraints()
        self.create_inequality_constraints()

    @property
    def quadratic_weight_filter(self) -> np.ndarray:
        """
        Returns a filter for the quadratic weights.
        """
        quadratic_weight_filter = np.ones(
            self.qp_data_symbolic.quadratic_weights.shape[0]
        )
        quadratic_weight_filter[self.qp_data_symbolic.num_non_slack_variables :] = (
            self.qp_data_symbolic.quadratic_weights.evaluate()[
                self.qp_data_symbolic.num_non_slack_variables :
            ]
            != 0
        )
        return quadratic_weight_filter.astype(bool)

    def create_direct_limits(self):
        """
        Creates a panda array for decision variable insights.
        """
        self.direct_limits = pd.DataFrame(
            {
                "lower bounds": self.qp_data_symbolic.box_lower_constraints.evaluate(),
                "solution": self.current_solution,
                "upper bounds": self.qp_data_symbolic.box_upper_constraints.evaluate(),
                "quadratic weight": self.qp_data_symbolic.quadratic_weights.evaluate(),
                "linear weight": self.qp_data_symbolic.linear_weights.evaluate(),
            },
            self.free_variable_names,
            dtype=float,
        )

    def create_equality_constraints(self):
        """
        Creates panda arrays for equality constraint insights.
        """
        eq_matrix_dofs_np = self.qp_data_symbolic.eq_matrix_dofs.evaluate()
        eq_matrix_slack_np = self.qp_data_symbolic.eq_matrix_slack.evaluate()
        Ex = eq_matrix_dofs_np @ self.current_solution[: eq_matrix_dofs_np.shape[1]]
        bounds = self.qp_data_symbolic.eq_bounds.evaluate()
        self.equality_constraints = pd.DataFrame(
            {
                "Ex": Ex,
                "slack": bounds - Ex,
                "bounds": bounds,
            },
            self.equality_constr_names,
            dtype=float,
        )
        self.equality_matrix = pd.DataFrame(
            eq_matrix_dofs_np,
            self.equality_constr_names,
            self.degree_of_freedom_names,
            dtype=float,
        )

    def create_inequality_constraints(self):
        """
        Creates panda arrays for inequality constraint insights.
        """
        neq_matrix_dofs_np = self.qp_data_symbolic.neq_matrix_dofs.evaluate()
        neq_matrix_slack_np = self.qp_data_symbolic.neq_matrix_slack.evaluate()
        Ex = neq_matrix_dofs_np @ self.current_solution[: neq_matrix_dofs_np.shape[1]]
        lower_bounds = self.qp_data_symbolic.neq_lower_bounds.evaluate()
        upper_bounds = self.qp_data_symbolic.neq_upper_bounds.evaluate()
        if len(self.inequality_constr_names) > 0:
            self.inequality_constraints = pd.DataFrame(
                {
                    "lower_bounds": lower_bounds,
                    "Ax": Ex,
                    # "slack": bounds - Ex,
                    "upper_bounds": upper_bounds,
                },
                self.inequality_constr_names,
                dtype=float,
            )
            self.inequality_matrix = pd.DataFrame(
                neq_matrix_dofs_np,
                self.inequality_constr_names,
                self.degree_of_freedom_names,
                dtype=float,
            )

    @property
    def free_variable_names(self) -> list[str]:
        """
        Returns the names of all free variables.
        """
        return self.qp_data_symbolic.free_variable_names

    @property
    def degree_of_freedom_names(self) -> list[str]:
        """
        Returns the names of all degrees of freedom.
        """
        names = []
        for derivative in ["vel", "jerk"]:
            for k in range(self.qp_data_symbolic.config.prediction_horizon):
                if (
                    derivative == "vel"
                    and k > self.qp_data_symbolic.config.prediction_horizon - 3
                ):
                    continue
                for dof in self.qp_data_symbolic.degrees_of_freedom:
                    names.append(f"{dof.name}_{derivative}_k_{k}")
        return names

    @property
    def equality_constr_names(self):
        """
        Returns the names of all equality constraints.
        """
        return self.qp_data_symbolic.eq_constraint_names

    @property
    def inequality_constr_names(self):
        """
        Returns the names of all inequality constraints.
        """
        return self.qp_data_symbolic.neq_constraint_names
