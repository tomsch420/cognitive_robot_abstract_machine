"""
Tiago turns the sink's water on or off in a RoboCasa kitchen, simulated in MuJoCo:
RoboCasa's ``TurnOnSinkFaucet`` and ``TurnOffSinkFaucet`` tasks. It takes hold of the
faucet's lever and turns it about its hinge, holding it by the grip of its fingers
alone.

Run the task with::

    python -m experiments.tiago_robocasa.faucet TurnOnSinkFaucet --record
"""

from __future__ import annotations

import logging
from abc import ABC
from dataclasses import dataclass
from enum import StrEnum

from experiments.tiago_robocasa.handles import TiagoHandleTask
from experiments.tiago_robocasa.trials import Trials, parser_of, settings_from
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


class SinkObjectPrefix(StrEnum):
    """
    How RoboCasa begins the names of the objects it lays into the sink.
    """

    DISTRACTOR = "distr_sink"
    """
    The objects it leaves lying in the basin to be in the way.
    """


class SinkPart(StrEnum):
    """
    How RoboCasa's sink names the bodies this task works with, at the end of their
    names.
    """

    BASIN = "_main_group_main"
    """
    The sink itself, which the lever stands behind.
    """

    LEVER = "_main_group_handle"
    """
    The lever that turns the water on and off.
    """


# %% a task at the sink's faucet


@dataclass
class TiagoFaucetTask(TiagoHandleTask, ABC):
    """
    One of RoboCasa's faucet tasks, performed by Tiago: it takes hold of the lever and
    turns it about its hinge.
    """

    handle_reach: float = 0.9
    """
    How far in front of its base's centre Tiago takes hold of the lever, in metres:
    the lever stands at the back of the sink, more than half a metre behind the
    counter's edge.
    """

    water_on_position: float = 0.4
    """
    How far the lever has to be turned for RoboCasa to count the water running, in
    radians.
    """

    def sink_part(self, part: SinkPart) -> Body:
        """
        :param part: Which part of the sink.
        :return: That part's body.
        """
        return next(
            body
            for body in self.world.bodies
            if body.name.name.startswith("sink") and body.name.name.endswith(part)
        )

    @property
    def fixture(self) -> Body:
        """
        :return: The sink.
        """
        return self.sink_part(SinkPart.BASIN)

    @property
    def handle(self) -> Handle:
        """
        :return: The faucet's lever.
        """
        return Handle(root=self.sink_part(SinkPart.LEVER))

    @property
    def handled_bodies(self) -> list[Body]:
        """
        :return: The sink, the counter it is set into and what lies in its basin:
            the lever stands a few centimetres above the counter, behind the basin, and
            the arm reaching it passes over them closer than the clearance it is
            otherwise held to.
        """
        in_the_basin = [
            body
            for body in self.world.bodies
            if body.name.name.startswith(SinkObjectPrefix.DISTRACTOR)
            and body.has_collision()
        ]
        return super().handled_bodies + self.bodies_under_the_handle + in_the_basin

    @property
    def square_position(self) -> float:
        """
        :return: The middle of the lever's travel, where it stands upright with the
            water off.
        """
        return 0.0

    @property
    def headings_to_try(self) -> tuple[float, ...]:
        """
        :return: Every quarter turn: the lever turns about an axis along the counter,
            not one that tells which side of the sink the room is on.
        """
        return self.quarter_turns

    @property
    def water_is_on(self) -> bool:
        """
        :return: Whether the lever is turned far enough for the water to run.
        """
        return self.mechanism.position > self.water_on_position


# %% the tasks


@dataclass
class TurnOnSinkFaucet(TiagoFaucetTask):
    """
    Tiago turns the water on.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The end of the lever's travel that runs the water.
        """
        return float(self.mechanism.dof.limits.upper.position)

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the water now runs.
        """
        return self.water_is_on


@dataclass
class TurnOffSinkFaucet(TiagoFaucetTask):
    """
    Tiago turns the water off, which RoboCasa has turned on.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The middle of the lever's travel, where the water stops.
        """
        return 0.0

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the water has stopped.
        """
        return not self.water_is_on


TASKS = {task.__name__: task for task in (TurnOnSinkFaucet, TurnOffSinkFaucet)}
"""
The faucet tasks Tiago can be asked for, by the name RoboCasa gives them.
"""


# %% running one from the command line


def main() -> None:
    """
    Run the faucet task the command line names over one kitchen after another.
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
