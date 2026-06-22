"""
Per-degree-of-freedom decision-variable limits, weights, and the MPC-based velocity profiles
that keep each degree of freedom within its position limits across the prediction horizon.
"""

from __future__ import annotations

import logging
from copy import copy
from dataclasses import dataclass
from itertools import product
from typing import TYPE_CHECKING
from uuid import UUID

import giskardpy.utils.math as gm
import krrood.symbolic_math.symbolic_math as sm
import numpy as np
from giskardpy.qp.exceptions import (
    InfeasibleException,
    MismatchedLimitLengthsError,
    VelocityLimitUnreachableException,
)
from giskardpy.qp.pos_in_vel_limits import (
    shifted_velocity_profile,
    compute_slowdown_asap_vel_profile,
)
from giskardpy.qp.solvers.qp_solver import QPSolver
from giskardpy.utils.math import mpc
from krrood.symbolic_math.symbolic_math import Scalar, FloatVariable
from semantic_digital_twin.spatial_types.derivatives import Derivatives, DerivativeMap
from semantic_digital_twin.world_description.degree_of_freedom import DegreeOfFreedom
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
)

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from giskardpy.qp.qp_controller_config import QPControllerConfig


def max_velocity_from_horizon_and_jerk_qp(
    prediction_horizon: int,
    vel_limit: float,
    acc_limit: float,
    jerk_limit: float,
    dt: float,
    solver_class: type[QPSolver],
):
    """
    Computes the highest velocity reachable within the prediction horizon under the given
    acceleration and jerk limits, by solving an MPC that maximizes velocity without a binding
    velocity limit.
    """
    upper_limits = (
        (vel_limit,) * prediction_horizon,
        (acc_limit,) * prediction_horizon,
        (jerk_limit,) * prediction_horizon,
    )
    lower_limits = (
        (-vel_limit,) * prediction_horizon,
        (-acc_limit,) * prediction_horizon,
        (-jerk_limit,) * prediction_horizon,
    )
    return mpc(
        upper_limits=upper_limits,
        lower_limits=lower_limits,
        current_values=(0, 0),
        dt=dt,
        ph=prediction_horizon,
        q_weight=(0, 0, 0),
        lin_weight=(-1, 0, 0),
        solver_class=solver_class,
        link_to_current_vel=False,
    )


@dataclass
class DirectLimits:
    """
    Represents weights and limits of decision variables in a QP.
    All fields must have the same length.
    """

    lower_bounds: sm.Vector
    """
    Lower box limit of each decision variable.
    """

    upper_bounds: sm.Vector
    """
    Upper box limit of each decision variable.
    """

    quadratic_weights: sm.Vector
    """
    Quadratic objective weight of each decision variable.
    """

    linear_weights: sm.Vector
    """
    Linear objective weight of each decision variable.
    """

    names: list[str]
    """
    Human readable name of each decision variable, used for debugging.
    """

    def __post_init__(self):
        """
        Ensures all bound, weight, and name fields describe the same number of decision variables.
        """
        lengths = {
            "lower_bounds": self.lower_bounds.shape[0],
            "upper_bounds": self.upper_bounds.shape[0],
            "quadratic_weights": self.quadratic_weights.shape[0],
            "linear_weights": self.linear_weights.shape[0],
            "names": len(self.names),
        }
        if len(set(lengths.values())) > 1:
            raise MismatchedLimitLengthsError(field_lengths=lengths)

    @classmethod
    def empty(cls) -> DirectLimits:
        """
        Creates a DirectLimits without any decision variables.
        """
        return cls(
            lower_bounds=sm.Vector([]),
            upper_bounds=sm.Vector([]),
            quadratic_weights=sm.Vector([]),
            linear_weights=sm.Vector([]),
            names=[],
        )


@dataclass
class DofLimitProfiler:
    """
    Computes the per-degree-of-freedom velocity, acceleration, and jerk bounds across the
    prediction horizon, including the MPC-based position-aware slowdown profiles.
    """

    config: QPControllerConfig
    """
    Controller configuration providing horizon length, time step, and solver.
    """

    def _compute_position_constrained_velocity_bounds(
        self,
        dof_symbols: DerivativeMap[FloatVariable],
        lower_limits: DerivativeMap[float],
        upper_limits: DerivativeMap[float],
        solver_class: type[QPSolver],
        time_step: float,
        prediction_horizon: int,
    ) -> tuple[sm.Vector, sm.Vector, sm.Vector, sm.Scalar]:
        """
        Computes per-step velocity bounds that keep the degree of freedom within its position
        limits, slowing it down early enough to stop before a limit is reached.
        Returns the lower and upper velocity bounds, a goal velocity profile, and a flag marking
        when the first step is already at rest against a limit.
        """
        velocity_limit = upper_limits.velocity
        if lower_limits.position is None:
            velocity_upper_bound = sm.Vector.ones(prediction_horizon) * velocity_limit
            velocity_lower_bound = -velocity_upper_bound
            goal_profile = sm.Vector.zeros(prediction_horizon)
            skip_first = sm.Scalar.const_false()
            return velocity_lower_bound, velocity_upper_bound, goal_profile, skip_first

        acceleration_limit = upper_limits.acceleration
        jerk_limit = upper_limits.jerk
        position_range = upper_limits.position - lower_limits.position
        velocity_limit = min(velocity_limit * time_step, position_range / 2) / time_step
        profile = gm.simple_mpc(
            vel_limit=velocity_limit,
            acc_limit=acceleration_limit,
            jerk_limit=jerk_limit,
            current_vel=velocity_limit,
            current_acc=0,
            dt=time_step,
            ph=prediction_horizon,
            q_weight=(0, 0, 0),
            lin_weight=(-1, 0, 0),
            solver_class=solver_class,
        )
        mpc_velocity_profile = profile[:prediction_horizon]
        mpc_acceleration_profile = profile[prediction_horizon : prediction_horizon * 2]
        position_error_lower_bound = lower_limits.position - dof_symbols.position
        position_error_upper_bound = upper_limits.position - dof_symbols.position
        velocity_lower_bound, _ = shifted_velocity_profile(
            vel_profile=mpc_velocity_profile,
            acc_profile=mpc_acceleration_profile,
            distance=-position_error_lower_bound,
            dt=time_step,
        )
        velocity_lower_bound *= -1
        velocity_upper_bound, _ = shifted_velocity_profile(
            vel_profile=mpc_velocity_profile,
            acc_profile=mpc_acceleration_profile,
            distance=position_error_upper_bound,
            dt=time_step,
        )
        one_step_change = jerk_limit * time_step**2
        one_step_change_lower_bound = sm.min(
            sm.max(Scalar(0), position_error_lower_bound / time_step),
            Scalar(one_step_change),
        )
        one_step_change_lower_bound = sm.limit(
            one_step_change_lower_bound, -velocity_limit, velocity_limit
        )
        one_step_change_upper_bound = sm.max(
            sm.min(Scalar(0), position_error_upper_bound / time_step),
            -Scalar(one_step_change),
        )
        one_step_change_upper_bound = sm.limit(
            one_step_change_upper_bound, -velocity_limit, velocity_limit
        )
        velocity_lower_bound[0] = sm.if_greater(
            position_error_lower_bound,
            0,
            one_step_change_lower_bound,
            copy(velocity_lower_bound[0]),
        )
        velocity_upper_bound[0] = sm.if_less(
            position_error_upper_bound,
            0,
            one_step_change_upper_bound,
            copy(velocity_upper_bound[0]),
        )
        goal_profile = sm.max(velocity_lower_bound, 0) + sm.min(velocity_upper_bound, 0)
        skip_first = sm.logic_or(
            velocity_lower_bound[0] >= 0, velocity_upper_bound[0] <= 0
        )
        return velocity_lower_bound, velocity_upper_bound, goal_profile, skip_first

    def compute_horizon_bounds(
        self,
        dof_symbols: DerivativeMap[FloatVariable],
        lower_limits: DerivativeMap[float],
        upper_limits: DerivativeMap[float],
        solver_class: type[QPSolver],
        time_step: float,
        prediction_horizon: int,
        epsilon: float = 0.00001,
    ) -> DegreeOfFreedomLimits[sm.Vector]:
        """
        Computes the velocity, acceleration, and jerk bounds for one degree of freedom across the
        whole prediction horizon, relaxing the jerk limit on the first steps when the position
        goal would otherwise be unreachable.
        """
        jerk_limit = upper_limits.jerk
        acceleration_limit = upper_limits.acceleration

        velocity_lower_bound, velocity_upper_bound, goal_profile, skip_first = (
            self._compute_position_constrained_velocity_bounds(
                dof_symbols=dof_symbols,
                lower_limits=lower_limits,
                upper_limits=upper_limits,
                solver_class=solver_class,
                time_step=time_step,
                prediction_horizon=prediction_horizon,
            )
        )

        acceleration_profile = (
            sm.Vector.ones(velocity_upper_bound.shape[0]) * acceleration_limit
        )
        jerk_profile = sm.Vector.ones(velocity_upper_bound.shape[0]) * jerk_limit

        projected_velocity_profile, _, _ = compute_slowdown_asap_vel_profile(
            dof_symbols.velocity,
            dof_symbols.acceleration,
            goal_profile,
            Scalar(jerk_limit),
            Scalar(time_step),
            prediction_horizon,
            skip_first,
        )
        _, _, projected_jerk_profile_violated = compute_slowdown_asap_vel_profile(
            dof_symbols.velocity,
            dof_symbols.acceleration,
            goal_profile,
            Scalar(np.inf),
            Scalar(time_step),
            prediction_horizon,
            skip_first,
        )
        velocity_lower_bound_violated = sm.logic_or(
            sm.logic_any(projected_velocity_profile < velocity_lower_bound - epsilon),
            sm.abs(projected_velocity_profile[-1]) >= epsilon,
        )
        velocity_upper_bound_violated = sm.logic_or(
            sm.logic_any(projected_velocity_profile > velocity_upper_bound + epsilon),
            sm.abs(projected_velocity_profile[-1]) >= epsilon,
        )
        needs_relaxed_jerk_limits = sm.logic_or(
            velocity_lower_bound_violated, velocity_upper_bound_violated
        )
        jerk_profile[0] = sm.if_else(
            needs_relaxed_jerk_limits,
            sm.max(Scalar(jerk_limit), sm.abs(projected_jerk_profile_violated[0])),
            sm.Scalar(jerk_limit),
        )
        jerk_profile[1] = sm.if_else(
            needs_relaxed_jerk_limits,
            sm.max(Scalar(jerk_limit), sm.abs(projected_jerk_profile_violated[1])),
            sm.Scalar(jerk_limit),
        )
        jerk_profile[2] = sm.if_else(
            needs_relaxed_jerk_limits,
            sm.max(Scalar(jerk_limit), sm.abs(projected_jerk_profile_violated[2])),
            sm.Scalar(jerk_limit),
        )

        velocity_lower_bound = sm.min(velocity_lower_bound, velocity_upper_bound)
        velocity_upper_bound = sm.max(velocity_lower_bound, velocity_upper_bound)
        acceleration_lower_bounds = -acceleration_profile
        acceleration_upper_bounds = acceleration_profile
        jerk_lower_bounds = sm.min(jerk_profile, -jerk_profile) * time_step**2
        jerk_upper_bounds = sm.max(jerk_profile, -jerk_profile) * time_step**2
        return DegreeOfFreedomLimits[sm.Vector](
            lower=DerivativeMap(
                velocity=velocity_lower_bound,
                acceleration=acceleration_lower_bounds,
                jerk=jerk_lower_bounds,
            ),
            upper=DerivativeMap(
                velocity=velocity_upper_bound,
                acceleration=acceleration_upper_bounds,
                jerk=jerk_upper_bounds,
            ),
        )

    def find_best_jerk_limit(
        self,
        prediction_horizon: int,
        time_step: float,
        target_velocity_limit: float,
        solver_class: type[QPSolver],
        almost_equal_threshold: float = 0.0001,
    ) -> float:
        """
        Finds the jerk limit under which the controller just barely reaches the target velocity
        within the prediction horizon.
        :param target_velocity_limit: The velocity limit that should be reachable.
        :return: The jerk limit that achieves target_velocity_limit.
        """
        jerk_limit = (4 * target_velocity_limit) / time_step**2
        upper_bound = jerk_limit
        lower_bound = 0
        best_vel_limit = 0
        best_jerk_limit = 0
        i = -1
        for i in range(100):
            vel_limit = max_velocity_from_horizon_and_jerk_qp(
                prediction_horizon=prediction_horizon,
                vel_limit=1000,
                acc_limit=np.inf,
                jerk_limit=jerk_limit,
                dt=time_step,
                solver_class=solver_class,
            )[0]
            if abs(vel_limit - target_velocity_limit) < abs(
                best_vel_limit - target_velocity_limit
            ):
                best_vel_limit = vel_limit
                best_jerk_limit = jerk_limit
            if abs(vel_limit - target_velocity_limit) < almost_equal_threshold:
                break
            if vel_limit > target_velocity_limit:
                upper_bound = jerk_limit
                jerk_limit = round((jerk_limit + lower_bound) / 2, 4)
            else:
                lower_bound = jerk_limit
                jerk_limit = round((jerk_limit + upper_bound) / 2, 4)
        logger.debug(
            f"best velocity limit: {best_vel_limit} "
            f"(target = {target_velocity_limit}) with jerk limit: {best_jerk_limit} after {i + 1} iterations"
        )
        return best_jerk_limit

    def compute(
        self,
        degree_of_freedom: DegreeOfFreedom,
        max_derivative: Derivatives,
    ) -> DegreeOfFreedomLimits[sm.Vector]:
        """
        Computes the horizon bounds for a single degree of freedom, filling in missing
        acceleration and jerk limits and raising when its velocity limit is unreachable.
        """
        config = self.config
        lower_limits = DerivativeMap()
        upper_limits = DerivativeMap()

        # %% pos limits
        if not degree_of_freedom.has_position_limits():
            lower_limits.position = upper_limits.position = None
        else:
            lower_limits.position = degree_of_freedom.limits.lower.position
            upper_limits.position = degree_of_freedom.limits.upper.position

        # %% vel limits
        lower_limits.velocity = degree_of_freedom.limits.lower.velocity
        upper_limits.velocity = degree_of_freedom.limits.upper.velocity

        # %% acc limits
        if degree_of_freedom.limits.lower.acceleration is None:
            lower_limits.acceleration = -np.inf
        else:
            lower_limits.acceleration = degree_of_freedom.limits.lower.acceleration
        if degree_of_freedom.limits.upper.acceleration is None:
            upper_limits.acceleration = np.inf
        else:
            upper_limits.acceleration = degree_of_freedom.limits.upper.acceleration

        # %% jerk limits
        if degree_of_freedom.limits.upper.jerk is None:
            upper_limits.jerk = self.find_best_jerk_limit(
                config.prediction_horizon,
                config.mpc_dt,
                upper_limits.velocity,
                solver_class=config.qp_solver_class,
            )
            lower_limits.jerk = -upper_limits.jerk
        else:
            upper_limits.jerk = degree_of_freedom.limits.upper.jerk
            lower_limits.jerk = degree_of_freedom.limits.lower.jerk

        try:
            return self.compute_horizon_bounds(
                dof_symbols=degree_of_freedom.variables,
                lower_limits=lower_limits,
                upper_limits=upper_limits,
                solver_class=config.qp_solver_class,
                time_step=config.mpc_dt,
                prediction_horizon=config.prediction_horizon,
            )
        except InfeasibleException:
            max_reachable_vel = max_velocity_from_horizon_and_jerk_qp(
                prediction_horizon=config.prediction_horizon,
                vel_limit=100,
                acc_limit=upper_limits.acceleration,
                jerk_limit=upper_limits.jerk,
                dt=config.mpc_dt,
                solver_class=config.qp_solver_class,
            )[0]
            if max_reachable_vel < upper_limits.velocity:
                exception = VelocityLimitUnreachableException(
                    degree_of_freedom_name=degree_of_freedom.name,
                    velocity_limit=upper_limits.velocity,
                    prediction_horizon=config.prediction_horizon,
                    jerk_limit=upper_limits.jerk,
                    mpc_dt=config.mpc_dt,
                    max_reachable_velocity=max_reachable_vel,
                )
                logger.error(str(exception))
                raise exception
            else:
                raise


@dataclass
class DofLimits:
    """
    Builds a :class:`DirectLimits` holding the bounds and weights of the robot's free variables
    (velocity and jerk decision variables across the prediction horizon).
    """

    @classmethod
    def create(
        cls,
        degrees_of_freedom: list[DegreeOfFreedom],
        config: QPControllerConfig,
    ) -> DirectLimits:
        """
        Builds the :class:`DirectLimits` for the given degrees of freedom and configuration.
        """
        self = cls()
        lower_bounds, upper_bounds = self.free_variable_bounds(
            degrees_of_freedom, config
        )
        quadratic_weights, linear_weights = self.init_weights(
            degrees_of_freedom, config
        )
        return DirectLimits(
            lower_bounds=lower_bounds,
            upper_bounds=upper_bounds,
            quadratic_weights=quadratic_weights,
            linear_weights=linear_weights,
            names=self.make_names(degrees_of_freedom, config),
        )

    def active_slots(
        self,
        degrees_of_freedom: list[DegreeOfFreedom],
        config: QPControllerConfig,
    ):
        """
        Yields every active decision variable slot as a ``(derivative, time_step, dof)`` tuple.
        The order defines the layout shared by bounds, weights, and names so they stay aligned.
        """
        max_derivative = config.max_derivative
        for derivative, time_step in product(
            [Derivatives.velocity, Derivatives.jerk],
            range(config.prediction_horizon),
        ):
            if time_step >= config.prediction_horizon - (max_derivative - derivative):
                continue
            for degree_of_freedom in degrees_of_freedom:
                yield derivative, time_step, degree_of_freedom

    def make_names(
        self, degrees_of_freedom: list[DegreeOfFreedom], config: QPControllerConfig
    ) -> list[str]:
        """
        Creates a debug name for every free variable slot.
        """
        short_label = {Derivatives.velocity: "vel", Derivatives.jerk: "jerk"}
        return [
            f"{dof.name}_{short_label[derivative]}_k_{time_step}"
            for derivative, time_step, dof in self.active_slots(
                degrees_of_freedom, config
            )
        ]

    def free_variable_bounds(
        self,
        degrees_of_freedom: list[DegreeOfFreedom],
        config: QPControllerConfig,
    ) -> tuple[sm.Vector, sm.Vector]:
        """
        Computes the lower and upper box limits of every free variable slot.
        """
        max_derivative = config.max_derivative
        lower_bounds = []
        upper_bounds = []
        profiler = DofLimitProfiler(config)
        cache: dict[UUID, DegreeOfFreedomLimits[sm.Vector]] = {}
        for degree_of_freedom in degrees_of_freedom:
            cache[degree_of_freedom.id] = profiler.compute(
                degree_of_freedom=degree_of_freedom,
                max_derivative=max_derivative,
            )
        for derivative, t, degree_of_freedom in self.active_slots(
            degrees_of_freedom, config
        ):
            lower_bounds.append(cache[degree_of_freedom.id].lower[derivative][t])
            upper_bounds.append(cache[degree_of_freedom.id].upper[derivative][t])

        return sm.Vector(lower_bounds), sm.Vector(upper_bounds)

    def init_weights(
        self,
        degrees_of_freedom: list[DegreeOfFreedom],
        config: QPControllerConfig,
    ) -> tuple[sm.Vector, sm.Vector]:
        """
        Computes the quadratic and linear objective weights of every free variable slot.
        """
        quadratic_weights = []
        for derivative, t, degree_of_freedom in self.active_slots(
            degrees_of_freedom, config
        ):
            normalized_weight = self.normalize_dof_weight(
                limit=degree_of_freedom.limits.upper[derivative],
                base_weight=config.get_dof_weight(degree_of_freedom.name, derivative),
                t=t,
                horizon=config.prediction_horizon - 3,
                alpha=config.horizon_weight_gain_scalar,
            )
            quadratic_weights.append(normalized_weight)
        return sm.Vector(quadratic_weights), sm.Vector.zeros(len(quadratic_weights))

    def normalize_dof_weight(
        self,
        limit: float | None,
        base_weight: float,
        t: int,
        horizon: int,
        alpha: float,
    ) -> sm.Scalar:
        """
        Scales a free variable weight by its limit so derivatives become comparable, and ramps it
        over the horizon so later time steps are penalized more.
        """

        def linear(x_in: float, weight: float, h: int, alpha: float) -> float:
            start = weight * alpha
            a = (weight - start) / h
            return a * x_in + start

        if limit is None:
            return 0.0
        weight = linear(t, base_weight, horizon, alpha)

        return weight * (1 / limit) ** 2
