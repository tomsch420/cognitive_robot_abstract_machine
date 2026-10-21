"""
Tiago opens and closes a drawer in a RoboCasa kitchen, simulated in MuJoCo: RoboCasa's
``OpenDrawer`` and ``CloseDrawer`` tasks. It takes hold of the drawer's handle and pulls
or pushes the drawer along its rails, holding the handle by the grip of its fingers
alone.

Run the opening task with::

    python -m experiments.tiago_robocasa.drawers OpenDrawer --record
"""

from __future__ import annotations

import logging
from abc import ABC
from dataclasses import dataclass, field

from typing_extensions import Callable

from coraplex.datastructures.dataclasses import Context
from krrood.ormatic.utils import classproperty
from coraplex.plans.plan import Plan
from experiments.hsr_robocasa.robocasa_task import FixtureReference
from experiments.tiago_robocasa.handles import TiagoHandleTask
from experiments.tiago_robocasa.trials import Trials, parser_of, settings_from
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Drawer,
    Handle,
)
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


# %% a task at the kitchen's top drawer


@dataclass
class TiagoDrawerTask(TiagoHandleTask, ABC):
    """
    One of RoboCasa's drawer tasks, performed by Tiago: it takes hold of the drawer's
    handle and draws the drawer along its rails by it.
    """

    @property
    def fixture(self) -> Body:
        """
        :return: The housing the drawer slides out of.
        """
        return self.task.referenced_fixtures[FixtureReference.DRAWER]

    @property
    def drawer(self) -> Drawer:
        """
        :return: The annotation of the part that slides out of the housing the task
            refers to.
        """
        below_the_housing = (
            self.world.compute_descendent_child_kinematic_structure_entities(
                self.fixture
            )
        )
        return [
            annotation
            for annotation in self.world.get_semantic_annotations_by_type(Drawer)
            if annotation.root in below_the_housing
        ][0]

    @property
    def handle(self) -> Handle:
        """
        :return: The drawer's handle.
        """
        return self.drawer.handle


# %% the tasks


@dataclass
class OpenDrawer(TiagoDrawerTask):
    """
    Tiago pulls the drawer out.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The end of the drawer's rails.
        """
        return float(self.mechanism.dof.limits.upper.position)

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the drawer now stands open.
        """
        return self.travel >= self.open_ratio


@dataclass
class CloseDrawer(TiagoDrawerTask):
    """
    Tiago pushes the drawer back in, which RoboCasa has opened.
    """

    handle_reach: float = 0.6
    """
    Measured on a RoboCasa drawer RoboCasa has drawn half out, its handle 0.79 m high:

    the hand gets onto it from 0.60 m to 0.65 m in front of the base and from nowhere
    else. The base stands where it does for a closed drawer; the handle has come out
    towards it.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The shut end of the drawer's rails.
        """
        return float(self.mechanism.dof.limits.lower.position)

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the drawer now stands closed.
        """
        return self.travel <= self.closed_ratio


@dataclass
class OpenAndCloseDrawer(TiagoDrawerTask):
    """
    Tiago pulls the drawer out and pushes it back in, in one episode of RoboCasa's
    ``OpenDrawer`` task.

    The drawer is closed from where opening it left Tiago: the base follows the hand
    out along the drawer's travel, which leaves the drawn-out handle where it was in
    front of Tiago.
    """

    closing: bool = field(init=False, default=False)
    """
    Whether the drawer is being pushed back in, once it has been pulled out.
    """

    widest_travel: float = field(init=False, default=0.0)
    """
    How far the drawer stood out, as a share of its travel, once it was pulled out.
    """

    @classproperty
    def task_name(cls) -> str:
        """
        RoboCasa's ``OpenDrawer``, which composes the kitchen with the drawer shut.
        """
        return OpenDrawer.__name__

    @property
    def goal_position(self) -> float:
        """
        :return: The end of the drawer's rails while it is pulled out, the shut end
            while it is pushed back in.
        """
        limits = self.mechanism.dof.limits
        return float(limits.lower.position if self.closing else limits.upper.position)

    @property
    def stages(self) -> list[Callable[[Context], Plan]]:
        """
        :return: Pulling the drawer out, then pushing it back in.
        """
        return [self.build_plan, self.pushing_it_back_in]

    def pushing_it_back_in(self, context: Context) -> Plan:
        """
        :param context: The context to plan in.
        :return: Pushing the drawer back in from where pulling it out left Tiago.
        """
        self.widest_travel = self.travel
        self.closing = True
        return self.working_the_handle(context)

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the drawer was pulled out and now stands closed again.
        """
        return (
            self.widest_travel >= self.open_ratio and self.travel <= self.closed_ratio
        )

    @property
    def outcome(self) -> str:
        """
        :return: How far the drawer was pulled out and where it stands now.
        """
        report = (
            f"{self.moving_part.name.name} was pulled out to {self.widest_travel:.0%}"
            f" of its travel and stands at {self.travel:.0%}"
        )
        if self.disturbed_fixtures:
            report += f"; also moved: {', '.join(self.disturbed_fixtures)}"
        return report


TASKS = {
    "OpenDrawer": OpenDrawer,
    "CloseDrawer": CloseDrawer,
    "OpenAndCloseDrawer": OpenAndCloseDrawer,
}
"""
The drawer tasks Tiago can be asked for, by the name RoboCasa gives them.
"""


# %% running one from the command line


def main() -> None:
    """
    Run the drawer task the command line names over one kitchen after another.
    """
    logging.basicConfig(level=logging.INFO, force=True)
    arguments = parser_of(__doc__, list(TASKS)).parse_args()
    Trials(
        task=TASKS[arguments.task],
        settings=settings_from(arguments),
        attempts=arguments.attempts,
        tries_every_kitchen=arguments.benchmark,
    ).run()


if __name__ == "__main__":
    main()
