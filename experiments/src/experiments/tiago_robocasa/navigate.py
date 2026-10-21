"""
Tiago drives across a RoboCasa kitchen, simulated in MuJoCo: RoboCasa's
``NavigateKitchen`` task. It starts in front of one fixture and drives, along a way
planned through the kitchen's free space, to stand in front of another.

Run it with::

    python -m experiments.tiago_robocasa.navigate NavigateKitchen --record
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.motions.robot_body import MoveJointsMotion
from experiments.scene_recording import ViewerCamera
from experiments.tiago_robocasa.exceptions import NoClearStandingPose
from experiments.tiago_robocasa.robocasa_task import TiagoRoboCasaTask
from experiments.tiago_robocasa.trials import Trials, parser_of, settings_from
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


class NavigationReference(StrEnum):
    """
    The names RoboCasa's navigation task refers to its fixtures by.
    """

    SOURCE = "src_fixture"
    """
    The fixture Tiago starts in front of.
    """

    TARGET = "target_fixture"
    """
    The fixture Tiago is to drive to.
    """


@dataclass
class NavigateKitchen(TiagoRoboCasaTask):
    """
    Tiago drives from the fixture RoboCasa starts it at to stand in front of the one
    RoboCasa names as the destination.

    The destination is the nearest stand in front of the target fixture that keeps
    Tiago clear of the kitchen, rather than the pose RoboCasa scores its own robot
    against, which the loaded task does not carry.
    """

    destination_reach: float = 0.6
    """
    How far from the middle of the target fixture the nearest stand tried lies, in
    metres: half a fixture's depth and the base's own radius.
    """

    arrival_tolerance: float = 0.2
    """
    How close to the destination the base has to come for the task to count as done,
    in metres: the distance RoboCasa counts a navigation done within.
    """

    driving_arm_positions: tuple[float, ...] = (
        1.2,
        -1.18,
        1.13,
        2.07,
        -1.95,
        -0.03,
        0.04,
    )
    """
    The joint positions, from the shoulder out, each arm is folded to for driving: the
    elbows tucked in against the torso, so that Tiago reaches 0.41 m out from the
    centre of its base rather than the 0.53 m of the park pose.
    """

    destination: Pose | None = field(init=False, default=None)
    """
    Where Tiago is to end up, once the plan is built.
    """

    def middle_of(self, fixture: Body) -> np.ndarray:
        """
        :param fixture: A fixture of the kitchen.
        :return: The middle of the box around all of it.
        """
        lowest, highest = self.box_around(self.colliding_bodies_below(fixture))
        return (lowest + highest) / 2

    @property
    def source(self) -> Body:
        """
        :return: The fixture Tiago starts in front of.
        """
        return self.task.referenced_fixtures[NavigationReference.SOURCE]

    @property
    def target(self) -> Body:
        """
        :return: The fixture Tiago drives to.
        """
        return self.task.referenced_fixtures[NavigationReference.TARGET]

    @property
    def first_target(self) -> np.ndarray:
        """
        :return: The middle of the fixture Tiago starts in front of.
        """
        return self.middle_of(self.source)

    def build_plan(self, context: Context) -> Plan:
        """
        Find the nearest clear stand in front of the target fixture and drive there
        from where Tiago stands.

        :param context: The context to plan in.
        :return: The plan.
        :raises NoClearStandingPose: If Tiago touches the kitchen at every stand in
            front of the target fixture.
        """
        arm_joints = [
            joint for arm in self.robot.all_arms for joint in arm.active_connections
        ]
        parked = [float(joint.position) for joint in arm_joints]
        folded = list(self.driving_arm_positions) * len(self.robot.all_arms)
        self.set_positions(arm_joints, folded)
        drive = self.robot.root.parent_connection
        start = HomogeneousTransformationMatrix(
            drive.origin.to_np().copy(), reference_frame=drive.parent
        )
        search = self.stand_that_reaches(
            lambda: True, self.middle_of(self.target), self.destination_reach
        )
        drive.origin = start
        if search.stand is None:
            self.set_positions(arm_joints, parked)
            raise NoClearStandingPose(target=search.target, tried=search.tried)
        self.destination = search.stand
        legs = self.drive_to(self.destination)
        self.set_positions(arm_joints, parked)
        logger.info(
            "driving from the %s to the %s",
            self.source.name.name,
            self.target.name.name,
        )
        folding = MoveJointsMotion(
            names=[joint.name.name for joint in arm_joints], positions=folded
        )
        return sequential([folding] + legs, context=context).plan

    def set_positions(
        self, joints: list[ActiveConnection1DOF], positions: list[float]
    ) -> None:
        """
        Set joints of Tiago in the planning world, without moving the simulated robot.

        :param joints: The joints.
        :param positions: Their positions, in the same order.
        """
        for joint, position in zip(joints, positions):
            joint.position = position

    @property
    def recording_camera(self) -> ViewerCamera:
        """
        :return: A camera looking down on the way between the two fixtures, from far
            enough to see both.
        """
        source, target = self.middle_of(self.source), self.middle_of(self.target)
        return ViewerCamera(
            look_at=tuple(float(value) for value in (source + target) / 2),
            distance=float(np.linalg.norm(source - target)) + 3.0,
            azimuth=140.0,
            elevation=-55.0,
        )

    @property
    def distance_to_the_destination(self) -> float:
        """
        :return: How far the base stands from the destination, in metres.
        """
        here = self.stands_at.to_np()[:2, 3]
        there = self.destination.to_np()[:2, 3]
        return float(np.linalg.norm(here - there))

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the base came to stand within :attr:`arrival_tolerance` of the
            destination.
        """
        return (
            self.destination is not None
            and self.distance_to_the_destination <= self.arrival_tolerance
        )

    @property
    def outcome(self) -> str:
        """
        :return: How far from the destination the base came to stand.
        """
        if self.destination is None:
            return "no destination was found"
        return (
            f"stands {self.distance_to_the_destination:.2f} m from the stand in front"
            f" of the {self.target.name.name}"
        )


TASKS = {"NavigateKitchen": NavigateKitchen}
"""
The navigation task Tiago can be asked for, by the name RoboCasa gives it.
"""


# %% running it from the command line


def main() -> None:
    """
    Run the navigation task over one kitchen after another.
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
