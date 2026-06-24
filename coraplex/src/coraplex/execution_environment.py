from __future__ import annotations

import logging
from dataclasses import dataclass, field

from coraplex.datastructures.enums import ExecutionType
from coraplex.plans.executables import GiskardExecutable

logger = logging.getLogger(__name__)


@dataclass
class ExecutionEnvironment:
    """
    Base class for managing execution context of all actions within.

    Instances of this class is to be used with a "with" context block

    Example:

        >>> with ExecutionEnvironment(ExecutionType.SIMULATED):
        >>>     SequentialPlan(context, NavigateActionDescription, ...)
    """

    execution_type: ExecutionType
    """
    The type of the execution environment.
    """

    previous_type: ExecutionType = field(init=False, default=None)
    """
    Type of the execution environment before setting it, used for nested
    environments.
    """

    def __enter__(self):
        """
        Entering function for 'with' scope, saves the previously set
        :py:attr:`~pycram.plans.executables.GiskardExecutable.execution_type` and sets it to the type of this
        environment.
        """
        self.previous_type = GiskardExecutable.execution_type
        GiskardExecutable.execution_type = self.execution_type

    def __exit__(self, _type, value, traceback):
        """
        Exit method for the 'with' scope, sets the
        :py:attr:`~pycram.plans.executables.GiskardExecutable.execution_type` to the previously used one.
        """
        GiskardExecutable.execution_type = self.previous_type

    def __call__(self):
        return self


# These are imported, so they don't have to be initialized when executing with
simulated_robot = ExecutionEnvironment(ExecutionType.SIMULATED)
real_robot = ExecutionEnvironment(ExecutionType.REAL)
semi_real_robot = ExecutionEnvironment(ExecutionType.SEMI_REAL)
no_execution = ExecutionEnvironment(ExecutionType.NO_EXECUTION)
