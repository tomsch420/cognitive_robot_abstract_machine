"""
Tiago opens and closes a hinged door in a RoboCasa kitchen, simulated in MuJoCo:
RoboCasa's ``OpenCabinet``, ``CloseCabinet``, ``OpenMicrowave`` and ``CloseMicrowave``
tasks. It takes hold of the door's handle and leads the door about its hinge, holding
the handle by the grip of its fingers alone.

Run the cabinet task with::

    python -m experiments.tiago_robocasa.doors OpenCabinet --record
"""

from __future__ import annotations

import logging
import math
from abc import ABC
from dataclasses import dataclass

import numpy as np

from experiments.hsr_robocasa.robocasa_task import FixtureReference
from experiments.tiago_robocasa.handles import TiagoHandleTask
from experiments.tiago_robocasa.trials import Trials, parser_of, settings_from
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Door,
    Handle,
)
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


# %% a task at a hinged door


@dataclass
class TiagoDoorTask(TiagoHandleTask, ABC):
    """
    One of RoboCasa's hinged door tasks, performed by Tiago: it takes hold of the door's
    handle and leads the door about its hinge by it.

    A fixture with two doors has the first of them worked. A door whose handle is
    moulded into it, as a microwave's is, is held by the pieces of it that stand out
    furthest towards the room.
    """

    handle_reach: float = 0.6
    """
    How far in front of its base's centre Tiago takes hold of the handle, in metres:

    a door handle stands higher than a drawer's, and the arm reaching up reaches less
    far ahead.
    """

    high_handle: float = 1.3
    """
    From what height up a handle is taken with the hand tilted up, in metres.
    """

    high_handle_pitch: float = -0.8
    """
    How far the hand is tilted up to take a high handle, in radians: measured in front
    of a RoboCasa microwave, level it reaches 1.5 m and tilted up this far 1.6 m.
    """

    handle_depth: float = 0.01
    """
    How far behind the door's outermost piece another piece may lie and still count as
    part of a moulded handle, in metres.
    """

    open_ratio: float = 0.9
    """
    The share of its travel from which on RoboCasa counts a hinged door open.
    """

    @property
    def fixture(self) -> Body:
        """
        :return: The cabinet or appliance the door closes.
        """
        return self.task.referenced_fixtures[FixtureReference.FIXTURE]

    @property
    def door(self) -> Door:
        """
        :return: The annotation of the door the task works.
        """
        below_the_fixture = (
            self.world.compute_descendent_child_kinematic_structure_entities(
                self.fixture
            )
        )
        return [
            annotation
            for annotation in self.world.get_semantic_annotations_by_type(Door)
            if annotation.root in below_the_fixture
        ][0]

    @property
    def handle(self) -> Handle:
        """
        :return: The door's handle, or the door itself where its handle is moulded
            into it.
        """
        if self.door.handle is None:
            return Handle(root=self.door.root)
        return self.door.handle

    @property
    def outward(self) -> np.ndarray:
        """
        :return: The level unit vector the closed door faces the room along: across the
            door leaf's thinnest level side, away from the fixture it closes.
        """
        lowest, highest = self.box_around(self.colliding_bodies_below(self.door.root))
        across = np.zeros(3)
        across[int(np.argmin((highest - lowest)[:2]))] = 1.0
        away = (lowest + highest) / 2 - self.extent(self.fixture).center.to_np()[:3]
        return across * math.copysign(1.0, float(away @ across))

    @property
    def handle_box(self) -> tuple[np.ndarray, np.ndarray]:
        """
        :return: The lowest and highest corner of the handle, or of the door's pieces
            standing out furthest towards the room where its handle is moulded into it.
        """
        if self.door.handle is not None:
            return super().handle_box
        pieces = self.door.root.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_boxes
        outward = self.outward
        reach = [
            max(
                np.array([piece.min_x, piece.min_y, 0.0]) @ outward,
                np.array([piece.max_x, piece.max_y, 0.0]) @ outward,
            )
            for piece in pieces
        ]
        return self.corners_around(
            [
                piece
                for piece, piece_reach in zip(pieces, reach)
                if piece_reach >= max(reach) - self.handle_depth
            ]
        )

    @property
    def handle_grasp_pitch(self) -> float:
        """
        :return: :attr:`high_handle_pitch` on a handle higher than
            :attr:`high_handle`, level otherwise.
        """
        return (
            self.high_handle_pitch if self.handle_middle[2] > self.high_handle else 0.0
        )

    @property
    def headings_to_try(self) -> tuple[float, ...]:
        """
        :return: The one way that faces the closed door.
        """
        return (float(np.arctan2(-self.outward[1], -self.outward[0])),)


@dataclass
class TiagoOpensADoor(TiagoDoorTask, ABC):
    """
    Tiago swings the door open.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The open end of the hinge's travel.
        """
        return float(self.mechanism.dof.limits.upper.position)

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the door now stands open.
        """
        return self.travel >= self.open_ratio


@dataclass
class TiagoClosesADoor(TiagoDoorTask, ABC):
    """
    Tiago swings the door shut, which RoboCasa has opened.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The shut end of the hinge's travel.
        """
        return float(self.mechanism.dof.limits.lower.position)

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the door now stands closed.
        """
        return self.travel <= self.closed_ratio


# %% the tasks


@dataclass
class OpenCabinet(TiagoOpensADoor):
    """
    Tiago opens a cabinet's door.
    """


@dataclass
class CloseCabinet(TiagoClosesADoor):
    """
    Tiago closes a cabinet's door.
    """


@dataclass
class OpenMicrowave(TiagoOpensADoor):
    """
    Tiago opens the microwave's door.
    """


@dataclass
class CloseMicrowave(TiagoClosesADoor):
    """
    Tiago closes the microwave's door.
    """


TASKS = {
    task.__name__: task
    for task in (OpenCabinet, CloseCabinet, OpenMicrowave, CloseMicrowave)
}
"""
The hinged door tasks Tiago can be asked for, by the name RoboCasa gives them.
"""

# %% running one from the command line


def main() -> None:
    """
    Run the door task the command line names over one kitchen after another.
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
