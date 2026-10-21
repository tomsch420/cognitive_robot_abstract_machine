"""
The Toyota HSR opens and closes a drawer in a RoboCasa kitchen, simulated in MuJoCo:
RoboCasa's ``OpenDrawer`` and ``CloseDrawer`` tasks. The HSR takes hold of the
drawer's handle, and pulls or pushes the drawer along its rails.

Run the opening task with::

    python -m experiments.hsr_robocasa.drawers
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from typing_extensions import Callable

import numpy as np
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.container import (
    CloseAction,
    ContainerAction,
    OpenAction,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Drawer,
    Handle,
)
from semantic_digital_twin.world_description.world_entity import Body

from experiments.hsr_robocasa.handles import HSRHandleTask
from experiments.hsr_robocasa.robocasa_task import ArmLink, FixtureReference


@dataclass
class HSRTaskAtDrawer(HSRHandleTask, ABC):
    """
    One of RoboCasa's tasks at its top drawer, performed by the HSR.
    """

    counter_height: float = 0.84
    """
    Lowered less than for the other tasks: RoboCasa takes a drawer for the top drawer
    its drawer tasks work only while the drawer's middle stands 0.7 m to 0.9 m high.
    """

    @property
    def drawer(self) -> Drawer:
        """
        The annotation of the drawer the task works: the part sliding out of the
        housing the task refers to.
        """
        housing = self.task.referenced_fixtures[FixtureReference.DRAWER]
        below_the_housing = (
            self.world.compute_descendent_child_kinematic_structure_entities(housing)
        )
        return [
            annotation
            for annotation in self.world.get_semantic_annotations_by_type(Drawer)
            if annotation.root in below_the_housing
        ][0]

    @property
    def handle(self) -> Handle:
        """
        The drawer's handle.
        """
        return self.drawer.handle


@dataclass
class HSRDrawerTask(HSRTaskAtDrawer, ABC):
    """
    One of RoboCasa's drawer tasks, performed by the HSR: it takes hold of the
    drawer's handle and moves the drawer along its rails.
    """

    open_ratio: float = 0.95
    """
    RoboCasa's drawer tasks count a drawer open from 95 % of its travel on.
    """

    closed_ratio: float = 0.05
    """
    RoboCasa's drawer tasks count a drawer closed up to 5 % of its travel.
    """

    @property
    @abstractmethod
    def drawer_action(self) -> Callable[..., ContainerAction]:
        """
        What builds the action that moves the drawer as the task asks.
        """

    @property
    def handled_bodies(self) -> list[Body]:
        """
        The drawer and its handle.
        """
        return self.colliding_bodies_below(self.drawer.root)

    @property
    def bodies_touching_handled_bodies(self) -> list[Body]:
        """
        The wrist's links and the forearm, which reaches out level with the hand and
        which the drawer's front comes towards as the drawer opens.
        """
        return self.arm_links(list(ArmLink))

    @property
    def first_target(self) -> np.ndarray:
        """
        The middle of the drawer's handle.
        """
        return self.handle_middle

    def build_plan(self, context: Context) -> Plan:
        return self.handle_plan(context, self.drawer_action)


# %% the tasks


@dataclass
class OpenDrawer(HSRDrawerTask):
    """
    Open the drawer.
    """

    @property
    def drawer_action(self) -> Callable[..., ContainerAction]:
        return OpenAction


@dataclass
class CloseDrawer(HSRDrawerTask):
    """
    Close the drawer, which RoboCasa has opened.
    """

    @property
    def drawer_action(self) -> Callable[..., ContainerAction]:
        return CloseAction


if __name__ == "__main__":
    experiment = OpenDrawer()
    experiment.build_world()
    experiment.run()
