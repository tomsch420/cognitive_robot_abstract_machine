from dataclasses import dataclass
from typing import Union, Optional

import semantic_world.spatial_types.spatial_types as cas
from giskardpy.data_types.exceptions import MonitorInitalizationException
from giskardpy.god_map import god_map
from giskardpy.motion_statechart.monitors.monitors import PayloadMonitor
from giskardpy.qp.solvers.qp_solver_ids import SupportedQPSolver


@dataclass
class SetQPSolver(PayloadMonitor):
    qp_solver_id: Union[SupportedQPSolver, int]

    def __post_init__(self):
        qp_solver_id = SupportedQPSolver(self.qp_solver_id)
        god_map.qp_controller.set_qp_solver(qp_solver_id)

    def __call__(self):
        self.state = True
