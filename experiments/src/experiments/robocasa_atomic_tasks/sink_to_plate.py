"""
Moving an object lying in the sink onto a plate on the counter with the PandaOmron, from
where RoboCasa spawns everything: the spout is swung out of the way, the object is taken
out by a grasp that was learned to lift it, tried again where it stays in the sink, and
set down on the plate.

Each step is a plan of its own, planned from the world as the step before left it.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import timedelta

from typing_extensions import Callable

from coraplex.robot_plans.actions.core.robot_body import MoveFingersToWidthAction
from cramph.node import StatechartNode
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.skills import (
    LowerTheToolUntilTouchingAction,
    RaiseTheToolAction,
)
from experiments.robocasa_atomic_tasks.sink_actions import (
    BackOffAction,
    LeaveTheSinkAction,
    LiftTheHandOutOfTheSinkAction,
    HoldTheObjectAboveAction,
    StandAtASpotAction,
    StandingSpot,
    TakeOutOfTheSinkParameters,
    TurnTheSpoutWithTheGripperAction,
)
from krrood.entity_query_language.factories import evaluate_condition, variable
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

# %% moving the object


@dataclass(kw_only=True)
class MoveFromTheSinkOntoAPlate:
    """
    Moves an object lying in a sink's basin onto a plate.

    The defaults are those of the PandaOmron.
    """

    episode: RoboCasaEpisode
    """
    The episode the object is moved in, simulated physically.
    """

    graspable: IsGraspable
    """
    The object moved.
    """

    plate: Body
    """
    The plate the object is dropped onto.
    """

    basin: Body
    """
    The sink's basin the object lies in.
    """

    spout: Body
    """
    The part of the sink's spout that turns.
    """

    spout_joint: ActiveConnection1DOF
    """
    The joint the spout turns about.
    """

    spout_aside_position: float = -1.0472
    """
    The position of :attr:`spout_joint` the spout is swung to, out of the way of the
    hand.
    """

    taking_out: TakeOutOfTheSinkParameters = field(
        default_factory=TakeOutOfTheSinkParameters
    )
    """
    How the object is taken out of the sink.
    """

    attempts: int = 3
    """
    How often taking the object out of the sink is tried at most.
    """

    closing_force: float = 28.36
    """
    How hard each finger presses at most while the object is moved, in newtons.
    """

    release_opening: float = 0.99
    """
    How far apart the fingers open to let go, as a fraction of how far apart they are
    when fully open.
    """

    spot_beside_the_plate: StandingSpot = field(
        default_factory=lambda: StandingSpot(x=-0.3853, y=-0.7584, yaw=1.5765)
    )
    """
    Where the robot stands beside the plate while it lets go.
    """

    carry_height: float = 0.12
    """
    How far the object's origin is above the plate's origin when it is carried there,
    before it is lowered onto the plate, in metres.
    """

    carry_velocity: float = 0.1
    """
    Speed of the tool frame on the way to above the plate, in m/s.
    """

    settling_time_on_the_plate: timedelta = timedelta(seconds=1.2)
    """
    How long the object is left to settle once dropped.
    """

    lift_off_height: float = 0.06
    """
    How far the open hand rises off the object once it lies on the plate, in metres.
    """

    back_off_distance: float = 0.3
    """
    How far the base backs off once the object is dropped, in metres.
    """

    def perform(self) -> None:
        """
        Limit the finger force, swing the spout aside, take the object out of the sink
        and drop it onto the plate.

        :raises PlanFailure: Why a step failed, or why the last attempt at taking the
            object out did.
        """
        arm = self.episode.arm
        self.episode.limit_finger_force(force=self.closing_force)
        self.episode.run_steps_in_turn(
            [
                *self.lifting_the_hand_out(),
                lambda: TurnTheSpoutWithTheGripperAction(
                    arm=arm,
                    spout=self.spout,
                    spout_joint=self.spout_joint,
                    goal_position=self.spout_aside_position,
                    basin=self.basin,
                ),
            ]
        ).raise_if_failed()
        self.take_the_object_out()
        self.drop_the_object_onto_the_plate()

    def take_the_object_out(self) -> None:
        """
        Take the object out of the sink, at most :attr:`attempts` times; before each
        attempt after the first, the robot gets clear and leads its hand out of the sink
        as the spout's swing leaves it.

        :raises PlanFailure: Why the last attempt failed, if none succeeded.
        """
        arm, robot, p = self.episode.arm, self.episode.robot, self.taking_out
        object_root = self.graspable.root
        for attempt in range(1, self.attempts + 1):
            back_to_the_start = []
            if attempt > 1:
                self.episode.get_clear()
                back_to_the_start = [
                    lambda: LeaveTheSinkAction(arm=arm, basin=self.basin)
                ]
            outcome = self.episode.run_steps_in_turn(
                [
                    *back_to_the_start,
                    lambda: StandAtASpotAction(
                        spot=p.spot_for_the_arm_posture, beside=object_root
                    ),
                    lambda: p.posture_step(arm=arm, robot=robot),
                    lambda: StandAtASpotAction(
                        spot=p.spot_beside_the_object, beside=object_root
                    ),
                    lambda: p.lift_step(
                        arm=arm, basin=self.basin, graspable=self.graspable
                    ),
                ]
            )
            if outcome.succeeded:
                return
            logger.info(
                "Taking the object out of the sink failed in attempt %d: %s",
                attempt,
                type(outcome.failure).__name__,
            )
        outcome.raise_if_failed()

    def drop_the_object_onto_the_plate(self) -> None:
        """
        Stand beside the plate, hold the object above it, lower it until it touches the
        plate, let go, let it settle, lift the hand off it and back the base off; the
        robot may touch the object meanwhile.

        :raises PlanFailure: Why a step failed.
        """
        arm = self.episode.arm
        gripper = arm.end_effector
        held = self.graspable.root
        with self.episode.allowances.rules_applied(
            ignore_rules=[
                AllowCollisionBetweenGroups(
                    body_group_a=list(self.episode.robot.bodies_with_collision),
                    body_group_b=[held],
                )
            ]
        ):
            self.episode.run_steps_in_turn(
                [
                    lambda: StandAtASpotAction(
                        spot=self.spot_beside_the_plate, beside=self.plate
                    ),
                    lambda: HoldTheObjectAboveAction(
                        arm=arm,
                        held=held,
                        above=self.plate,
                        height=self.carry_height,
                        cruise_linear_velocity=self.carry_velocity,
                    ),
                    lambda: LowerTheToolUntilTouchingAction(
                        arm=arm, bodies=[held], others=[self.plate]
                    ),
                    lambda: MoveFingersToWidthAction(
                        two_finger_gripper=gripper,
                        width=self.release_opening * gripper.separation_when_open,
                    ),
                ]
            ).raise_if_failed()
            self.episode.let_physics_run(self.settling_time_on_the_plate)
            self.episode.run_steps_in_turn(
                [
                    lambda: RaiseTheToolAction(arm=arm, height=self.lift_off_height),
                    lambda: BackOffAction(distance=self.back_off_distance),
                ]
            ).raise_if_failed()

    def lifting_the_hand_out(self) -> list[Callable[[], StatechartNode]]:
        """
        :return: The step lifting the hand out of the basin, where it is inside it now;
            none where it is not.
        """
        hand = variable(Body, self.episode.arm.end_effector.bodies_with_collision)
        if not evaluate_condition(InsideOf(body=hand, other=self.basin)):
            return []
        return [
            lambda: LiftTheHandOutOfTheSinkAction(
                arm=self.episode.arm, basin=self.basin
            )
        ]
