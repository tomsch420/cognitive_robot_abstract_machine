"""
The Toyota HSR opens and closes the oven's door in a RoboCasa kitchen, simulated in
MuJoCo: RoboCasa's ``OpenOven`` and ``CloseOven`` tasks. The HSR takes hold of the bar
along the top of the door and lets the door down about its hinge at the bottom, or
swings it back up.

Run the opening task with::

    python -m experiments.hsr_robocasa.doors
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from functools import partial

from typing_extensions import Callable

import numpy as np
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.container import (
    CloseAction,
    ContainerAction,
    OpenAction,
)
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Door,
    Handle,
)
from semantic_digital_twin.world_description.world_entity import Body

from experiments.hsr_robocasa.handles import HSRHandleTask
from experiments.hsr_robocasa.robocasa_task import ArmLink, FixtureReference

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType


@dataclass
class HSROvenDoorTask(HSRHandleTask, ABC):
    """
    One of RoboCasa's oven door tasks, performed by the HSR: it takes hold of the bar
    along the top of the door and moves the door about its hinge.
    """

    layout: LayoutType = LayoutType.LAYOUT002
    """
    A kitchen with an oven in a tall stack, which RoboCasa's oven tasks need: layout 1
    has none.
    """

    @property
    @abstractmethod
    def door_action(self) -> Callable[..., ContainerAction]:
        """
        What builds the action that moves the door as the task asks.
        """

    @property
    def door(self) -> Door:
        """
        The annotation of the oven's door.
        """
        oven = self.task.referenced_fixtures[FixtureReference.FIXTURE]
        below_the_oven = (
            self.world.compute_descendent_child_kinematic_structure_entities(oven)
        )
        return [
            annotation
            for annotation in self.world.get_semantic_annotations_by_type(Door)
            if annotation.root in below_the_oven
        ][0]

    @property
    def handle(self) -> Handle:
        """
        The door itself: its bar is a part of it rather than a body of its own.
        """
        return Handle(root=self.door.root)

    @property
    def handle_middle(self) -> np.ndarray:
        """
        The middle of the door's bar: the piece of the door standing out furthest in
        front.
        """
        pieces = self.door.root.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_boxes
        return min(pieces, key=lambda piece: piece.min_y).center.to_np()[:3]

    @property
    def handled_bodies(self) -> list[Body]:
        """
        The door.
        """
        return self.colliding_bodies_below(self.door.root)

    @property
    def bodies_touching_handled_bodies(self) -> list[Body]:
        """
        The hand, which takes hold of the door's own bar, and the wrist's links and the
        forearm, which the door's top comes towards as the door opens.
        """
        return (
            self.arm_links(list(ArmLink))
            + self.robot.all_arms[0].end_effector.bodies_with_collision
        )

    @property
    def first_target(self) -> np.ndarray:
        """
        The middle of the door's bar.
        """
        return self.handle_middle

    def build_plan(self, context: Context) -> Plan:
        return self.handle_plan(context, self.door_action)


# %% the tasks


@dataclass
class OpenOven(HSROvenDoorTask):
    """
    Open the oven's door.
    """

    @property
    def door_action(self) -> Callable[..., ContainerAction]:
        return OpenAction


@dataclass
class CloseOven(HSROvenDoorTask):
    """
    Close the oven's door, which RoboCasa has opened.
    """

    handle_reach: float = 0.59
    """
    RoboCasa has let the door down, so its bar faces the floor and is taken from below
    and in front, the hand tilted up as the door is. Tilted up that steeply, the arm
    reaches the bar only 0.46 m to 0.60 m ahead, and where taking hold of it starts
    its approach, lower down, only from 0.52 m on.
    """

    approach_clearance: float = 0.05
    """
    How far, in metres, the hand stays off the bar where taking hold of it starts its
    approach: further out, that start lies too low for the arm, coming from below.
    """

    closing_tolerance: float = 0.002
    """
    How far, in radians, the door may stand open once closed: RoboCasa counts an oven
    closed only within half a percent of its door's travel.
    """

    @property
    def stages(self) -> list[Callable[[Context], Plan]]:
        """
        Swings the door up, which leaves it about 0.05 rad open, as the hand holds the
        bar loosely from below; then takes hold of the bar again, from the front where
        the door came to rest, and presses the door shut.
        """
        return [self.build_plan, self.build_plan]

    @property
    def door_action(self) -> Callable[..., ContainerAction]:
        """
        Closes the door all the way, to its joint's lower limit.
        """
        return partial(
            CloseAction,
            closed_position=self.moving_part.parent_connection.dof.limits.lower.position,
            closing_tolerance=self.closing_tolerance,
            approach_clearance=self.approach_clearance,
        )


if __name__ == "__main__":
    experiment = OpenOven()
    experiment.build_world()
    experiment.run()
