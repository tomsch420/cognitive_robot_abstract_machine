"""
Tiago works the small appliances on a RoboCasa kitchen's counter, simulated in MuJoCo:
RoboCasa's ``TurnOnToaster`` and ``SlideToasterOvenRack`` tasks. It takes hold of the
part that moves, a toaster's lever or a toaster oven's rack, and slides it along its
joint, holding it by the grip of its fingers alone.

Run a task with::

    python -m experiments.tiago_robocasa.appliances TurnOnToaster --record
"""

from __future__ import annotations

import logging
import re
from abc import ABC
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
from typing_extensions import Callable

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from experiments.scene_recording import ViewerCamera
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from experiments.tiago_robocasa.handles import HandleHold, TiagoHandleTask
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from experiments.tiago_robocasa.trials import Trials, parser_of, settings_from
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


class AppliancePart(StrEnum):
    """
    How RoboCasa names the bodies of the appliances these tasks work.
    """

    TOASTER = "toaster_"
    """
    The start of the name of every body of a toaster, and of a toaster oven.
    """

    TOASTER_OVEN = "toaster_oven"
    """
    The start of the name of every body of a toaster oven, and the name RoboCasa's
    rack task refers to the toaster oven by.
    """

    HOUSING = "_main"
    """
    The end of the name of an appliance's housing.
    """

    LEVER = "lever"
    """
    The end of the name of a lever that lowers the bread into a toaster's slot.
    """

    ELECTRIC_KETTLE = "electric_kettle"
    """
    The name RoboCasa's kettle tasks refer to the electric kettle by.
    """

    SWITCH = "_switch"
    """
    The end of the name of the switch that turns an electric kettle on.
    """

    SHELF = r"_(rack|tray)\d+"
    """
    The end of the name of a toaster oven's rack or tray, as a regular expression.
    """


# %% a task at a small appliance


@dataclass
class TiagoApplianceTask(TiagoHandleTask, ABC):
    """
    One of RoboCasa's tasks at a small appliance on the counter, performed by Tiago: it
    takes hold of the part that moves where that stands out of the appliance's housing,
    and slides it along its joint.
    """

    handle_reach: float = 0.7
    """
    How far in front of its base's centre Tiago takes hold of the part, in metres: the
    appliance stands at the back of the counter.
    """

    waits_in_joint_space: bool = True

    recording_distance: float = 1.6
    """
    How far the recording camera stands from the part, in metres: close enough to see
    the fingers on a part a few centimetres wide.
    """

    @property
    def parts_prefix(self) -> str:
        """
        :return: The start of the name of every body of the appliance.
        """
        return self.fixture.name.name.removesuffix(AppliancePart.HOUSING)

    @property
    def handle_box(self) -> tuple[np.ndarray, np.ndarray]:
        """
        :return: The lowest and highest corner of the piece of the part that stands
            furthest out of the housing, which the fingers close on: the part's body
            also takes in what of it lies inside the housing.
        """
        housing = self.extent(self.fixture).center.to_np()[:2]
        pieces = self.handle.root.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_boxes
        outermost = max(
            pieces,
            key=lambda piece: np.linalg.norm(piece.center.to_np()[:2] - housing),
        )
        return self.corners_around([outermost])

    @property
    def handled_bodies(self) -> list[Body]:
        """
        :return: The appliance and the counter it stands on: the part stands a few
            centimetres above the counter, and the arm reaching it passes over the
            counter closer than the clearance it is otherwise held to.
        """
        return super().handled_bodies + self.bodies_under_the_handle

    @property
    def headings_to_try(self) -> tuple[float, ...]:
        """
        :return: The one way that faces the appliance, read off which way the part
            stands out of its housing.
        """
        outward = self.handle_middle - self.extent(self.fixture).center.to_np()[:3]
        return (float(np.arctan2(-outward[1], -outward[0])),)

    @property
    def recording_camera(self) -> ViewerCamera:
        """
        :return: A camera looking down at the part from close by.
        """
        return self.camera_on(self.handle_middle, self.recording_distance)


# %% the tasks


@dataclass
class TiagoPressTask(TiagoApplianceTask, ABC):
    """
    Tiago presses an appliance's lever or switch down with its closed fingertips,
    which turns the appliance on.

    The lever springs back up once the hand lifts off it, so whether the appliance was
    turned on is also read while the hand still holds it down.
    """

    press_overshoot: float = 0.0
    """
    How far below the bottom of the lever's travel the fingertips are aimed, in
    metres: a hand is taken to have arrived within :attr:`tool_frame_tolerance` of
    where it is aimed, which may be more than a short lever travels.
    """

    press_tolerance: float = 0.05
    """
    How close the fingertips have to come to where they are aimed as they press the
    lever down, in metres: the fingertips come to rest on the lever and the housing
    around it a few centimetres short of that.
    """

    press_height: float = 0.015
    """
    How far above the middle of the lever's tab the closed fingertips are brought
    before they press down, in metres: half the tab's height and half a fingertip's.
    """

    turned_on_ratio: float = 0.9
    """
    How far down the lever has to be pushed, as a share of its travel, for RoboCasa to
    count the appliance turned on.
    """

    deepest_travel: float = field(init=False, default=0.0)
    """
    How far down the lever stood, as a share of its travel, while the hand held it
    down.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The bottom of the lever's travel.
        """
        return float(self.mechanism.dof.limits.upper.position)

    @property
    def stages(self) -> list[Callable[[Context], Plan]]:
        """
        :return: Pushing the lever down and holding it there, then letting go of it.
        """
        return [self.pushing_the_lever_down, self.letting_go_of_the_lever]

    def raised(self, pose: Pose, height: float | None = None) -> Pose:
        """
        :param pose: A pose of the hand in the kitchen's frame.
        :param height: How much higher, in metres; :attr:`press_height` if not given.
        :return: That pose that much higher.
        """
        matrix = pose.to_np().copy()
        matrix[2, 3] += self.press_height if height is None else height
        return HomogeneousTransformationMatrix(
            matrix, reference_frame=self.world.root
        ).pose

    def drawing_the_fixture(self) -> list[BaseMotion | ActionDescription]:
        """
        Press rather than grip: the fingers would close on a tab barely a centimetre
        high and slip off it as the hand pushes down.

        :return: Closing the fingers, bringing the fingertips in front of the lever and
            onto the top of its tab, and pressing the lever down to the bottom of its
            slot.
        """
        grasp = self.handle_grasp()
        self.hold = HandleHold(
            grasp=grasp,
            pre_grasp=self.raised(self.pre_grasp()),
            drawn_to=self.pull_goal,
        )
        on_the_tab = self.raised(grasp.world_T_grasp)
        return [
            ParkArmsAction(self.robot.all_arms),
            MoveGripperMotion(
                GripperState.CLOSE,
                self.arm.end_effector,
                allow_gripper_collision=True,
            ),
            *self.reaching_the_waiting_pose(),
            MoveToolCenterPointMotion(
                on_the_tab,
                self.arm,
                allow_gripper_collision=True,
                position_threshold=self.tool_frame_tolerance,
            ),
            MoveToolCenterPointMotion(
                self.raised(
                    self.shifted(on_the_tab, self.hold.drawn_to),
                    -self.press_overshoot,
                ),
                self.arm,
                allow_gripper_collision=True,
                max_linear_velocity=self.pull_velocity,
                position_threshold=self.press_tolerance,
            ),
        ]

    def letting_go_of_the_handle(self) -> list[BaseMotion | ActionDescription]:
        """
        Lift off rather than open: the closed fingertips rest on the tab, and the hand
        held below the tab's start would sweep through the housing backing out.

        :return: Lifting the fingertips straight up off the tab, backing the hand out to
            where it waited, and parking the arms.
        """
        return [
            MoveToolCenterPointMotion(
                self.raised(self.hold.grasp.world_T_grasp),
                self.arm,
                allow_gripper_collision=True,
                position_threshold=self.tool_frame_tolerance,
            ),
            MoveToolCenterPointMotion(
                self.hold.pre_grasp,
                self.arm,
                allow_gripper_collision=True,
                position_threshold=self.tool_frame_tolerance,
            ),
            ParkArmsAction(self.robot.all_arms),
        ]

    def pushing_the_lever_down(self, context: Context) -> Plan:
        """
        :param context: The context to plan in.
        :return: Standing at the appliance and pressing the lever down.
        """
        self.stand_at_the_handle()
        return sequential(self.drawing_the_fixture(), context=context).plan

    def letting_go_of_the_lever(self, context: Context) -> Plan:
        """
        Note how far down the lever was pushed before the hand lets go of it.

        :param context: The context to plan in.
        :return: Letting go of the lever and backing the hand away.
        """
        self.deepest_travel = self.travel
        return sequential(self.letting_go_of_the_handle(), context=context).plan

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the lever was pushed far enough down to turn the appliance on,
            while the hand held it or by where it came to rest.
        """
        return max(self.deepest_travel, self.travel) >= self.turned_on_ratio

    @property
    def outcome(self) -> str:
        """
        :return: How far down the lever was pushed and where it stands now.
        """
        report = (
            f"{self.handle.root.name.name} was pushed down {self.deepest_travel:.0%}"
            f" of its travel and stands at {self.travel:.0%}"
        )
        if self.disturbed_fixtures:
            report += f"; also moved: {', '.join(self.disturbed_fixtures)}"
        return report


@dataclass
class TurnOnToaster(TiagoPressTask):
    """
    Tiago presses the toaster's lever down, which lowers the bread and turns the toaster
    on.
    """

    @property
    def fixture(self) -> Body:
        """
        :return: The toaster's housing.
        """
        return next(
            body
            for body in self.world.bodies
            if body.name.name.startswith(AppliancePart.TOASTER)
            and not body.name.name.startswith(AppliancePart.TOASTER_OVEN)
            and body.name.name.endswith(AppliancePart.HOUSING)
        )

    @property
    def handle(self) -> Handle:
        """
        :return: The first of the toaster's levers.
        """
        levers = sorted(
            (
                body
                for body in self.world.bodies
                if body.name.name.startswith(self.parts_prefix)
                and body.name.name.endswith(AppliancePart.LEVER)
            ),
            key=lambda body: body.name.name,
        )
        return Handle(root=levers[0])


@dataclass
class TurnOnElectricKettle(TiagoPressTask):
    """
    Tiago presses the switch at the foot of the electric kettle's handle down, which
    turns the kettle on.
    """

    turned_on_ratio: float = 0.95

    press_overshoot: float = 0.045

    @property
    def fixture(self) -> Body:
        """
        :return: The kettle's base.
        """
        return self.task.referenced_fixtures[AppliancePart.ELECTRIC_KETTLE]

    @property
    def handle(self) -> Handle:
        """
        :return: The kettle's switch.
        """
        return Handle(
            root=next(
                body
                for body in self.world.bodies
                if body.name.name == self.parts_prefix + AppliancePart.SWITCH
            )
        )


@dataclass
class SlideToasterOvenRack(TiagoApplianceTask):
    """
    Tiago slides a rack or tray of the toaster oven, whose door RoboCasa has opened,
    all the way out if RoboCasa left it half out, or all the way in if RoboCasa left it
    out.
    """

    half_out_ratio: float = 0.75
    """
    The share of its travel below which RoboCasa counts as having left the rack half
    out, to be pulled the rest of the way: it leaves a rack to be pulled at half its
    travel and one to be pushed at the end of it.
    """

    pulls_out: bool | None = field(init=False, default=None)
    """
    Whether the rack is to be pulled out rather than pushed in, once the plan is built.
    """

    front_depth: float = 0.015
    """
    How far behind the rack's foremost piece a piece may lie and still belong to the
    rack's front edge, which the fingers close on, in metres.
    """

    @property
    def fixture(self) -> Body:
        """
        :return: The toaster oven's housing.
        """
        return self.task.referenced_fixtures[AppliancePart.TOASTER_OVEN]

    @property
    def handle(self) -> Handle:
        """
        :return: The rack or tray RoboCasa slid out, which is the one furthest along
            its rails.
        """
        shelf = re.compile(re.escape(self.parts_prefix) + AppliancePart.SHELF)
        shelves = [
            body
            for body in self.world.bodies
            if shelf.fullmatch(body.name.name) and body.has_collision()
        ]
        return Handle(
            root=max(shelves, key=lambda body: body.parent_connection.position)
        )

    @property
    def outward(self) -> np.ndarray:
        """
        :return: The unit direction, in the kitchen's frame, the rack slides out along.
        """
        limits = self.mechanism.dof.limits
        slide = (
            self.shift_to(limits.upper.position)[:3, 3]
            - self.shift_to(limits.lower.position)[:3, 3]
        )
        return slide / np.linalg.norm(slide)

    @property
    def handle_box(self) -> tuple[np.ndarray, np.ndarray]:
        """
        :return: The lowest and highest corner of the rack's front edge.
        """
        pieces = self.handle.root.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_boxes
        reach_out = [float(piece.center.to_np()[:3] @ self.outward) for piece in pieces]
        foremost = max(reach_out)
        return self.corners_around(
            [
                piece
                for piece, out in zip(pieces, reach_out)
                if out >= foremost - self.front_depth
            ]
        )

    @property
    def headings_to_try(self) -> tuple[float, ...]:
        """
        :return: The one way that faces the rack head on.
        """
        return (float(np.arctan2(-self.outward[1], -self.outward[0])),)

    @property
    def goal_position(self) -> float:
        """
        :return: The end of the rack's rails if it is to be pulled out, the shut end if
            it is to be pushed in.
        """
        limits = self.mechanism.dof.limits
        return float(limits.upper.position if self.pulls_out else limits.lower.position)

    def build_plan(self, context: Context) -> Plan:
        """
        Read off where RoboCasa left the rack which way it is to slide, then slide it.

        :param context: The context to plan in.
        :return: The plan.
        """
        self.pulls_out = self.travel < self.half_out_ratio
        self.waits_in_joint_space = self.pulls_out
        logger.info(
            "sliding the %s %s",
            self.handle.root.name.name,
            "out" if self.pulls_out else "in",
        )
        return super().build_plan(context)

    def drawing_the_fixture(self) -> list[BaseMotion | ActionDescription]:
        """
        A rack to be pushed in is pushed with the closed fingertips rather than held:
        the arm reaches its front edge, sticking out over the open door, but not the
        pose in front of it the hand would wait in.

        :return: As :meth:`TiagoHandleTask.drawing_the_fixture` for a rack to be pulled
            out; for one to be pushed in, closing the fingers, bringing the fingertips
            onto the rack's front edge and pushing the rack in.
        """
        if self.pulls_out:
            return super().drawing_the_fixture()
        grasp = self.handle_grasp()
        self.hold = HandleHold(
            grasp=grasp, pre_grasp=grasp.world_T_grasp, drawn_to=self.pull_goal
        )
        return [
            ParkArmsAction(self.robot.all_arms),
            MoveGripperMotion(
                GripperState.CLOSE,
                self.arm.end_effector,
                allow_gripper_collision=True,
            ),
            *self.arm_motions_to(grasp.world_T_grasp),
            MoveToolCenterPointMotion(
                self.shifted(grasp.world_T_grasp, self.hold.drawn_to),
                self.arm,
                allow_gripper_collision=True,
                max_linear_velocity=self.pull_velocity,
                position_threshold=self.tool_frame_tolerance,
            ),
        ]

    def letting_go_of_the_handle(self) -> list[BaseMotion | ActionDescription]:
        """
        :return: As :meth:`TiagoHandleTask.letting_go_of_the_handle` for a rack pulled
            out; for one pushed in, drawing the fingertips back out of the toaster oven
            by :attr:`retreat_distance` and parking the arms.
        """
        if self.pulls_out:
            return super().letting_go_of_the_handle()
        pushed = self.shifted(self.hold.grasp.world_T_grasp, self.hold.drawn_to).to_np()
        withdrawn = pushed.copy()
        withdrawn[:3, 3] += self.outward * self.retreat_distance
        return [
            MoveToolCenterPointMotion(
                HomogeneousTransformationMatrix(
                    withdrawn, reference_frame=self.world.root
                ).pose,
                self.arm,
                allow_gripper_collision=True,
                position_threshold=self.tool_frame_tolerance,
            ),
            ParkArmsAction(self.robot.all_arms),
        ]

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the rack now stands all the way out or all the way in, as the
            task wants it.
        """
        if self.pulls_out:
            return self.travel >= self.open_ratio
        return self.travel <= self.closed_ratio


TASKS = {
    task.__name__: task
    for task in (TurnOnToaster, TurnOnElectricKettle, SlideToasterOvenRack)
}
"""
The appliance tasks Tiago can be asked for, by the name RoboCasa gives them.
"""


# %% running one from the command line


def main() -> None:
    """
    Run the appliance task the command line names over one kitchen after another.
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
