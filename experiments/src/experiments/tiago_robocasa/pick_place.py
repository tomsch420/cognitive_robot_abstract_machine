"""
Tiago performs RoboCasa's pick-and-place tasks: it takes the object RoboCasa sampled off
where it lies and puts it where the task asks for, holding it by the grip of its fingers
alone.

Run one with (the ``experiments`` package must be importable)::

    python -m experiments.tiago_robocasa.pick_place PickPlaceCounterToSink
    python -m experiments.tiago_robocasa.pick_place PickPlaceCounterToSink --record
"""

from __future__ import annotations

import logging
import math
from functools import cached_property
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any

import numpy as np

from coraplex.datastructures.dataclasses import Context
from krrood.ormatic.utils import classproperty
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from experiments.hsr_robocasa.robocasa_task import FixtureReference
from experiments.tiago_robocasa.drawers import TiagoDrawerTask
from experiments.tiago_robocasa.exceptions import NoPlaceToLetGo
from experiments.tiago_robocasa.trials import Trials, parser_of, settings_from
from experiments.tiago_robocasa.robocasa_task import TiagoRoboCasaTask
from semantic_digital_twin.adapters.multi_sim import (
    ContactDimensionality,
    MujocoGeom,
)
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    CounterTop,
    Sink,
)
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


# %% the objects a pick and place task names


class TaskObject(StrEnum):
    """
    The names RoboCasa gives the bodies of a pick-and-place task.
    """

    FOOD = "obj_main"
    """
    The object the task moves.
    """

    CONTAINER = "container_main"
    """
    What it is to be put into.
    """


# %% picking something up and putting it somewhere


@dataclass
class TiagoPickPlaceTask(TiagoRoboCasaTask):
    """
    Tiago takes the task's object off where it lies and puts it into the receptacle.
    """

    finger_clearance: float = 0.08
    """
    How far clear of the object the shut fingers wait before the hand moves on to the
    grasp, in metres, on top of how far they reach past it.
    """

    carry_velocity: float = 0.1
    """
    How fast the hand travels while it is not about to touch anything, in metres per
    second.
    """

    approach_velocity: float = 0.03
    """
    How fast the hand creeps the last stretch onto the grasp, in metres per second.
    """

    closing_velocity: float = 0.05
    """
    How fast the fingers close, in metres per second.
    """

    stall_minimum_time: float = 0.3
    """
    How long the fingers have to be stalled before the grasp counts as closed, in
    seconds.
    """

    arrival_tolerance: float = 0.03
    """
    How close the hand has to come to a goal before it counts as reached, in metres.

    Loose enough that a simulated arm, which trails what it is asked for while it
    moves, is taken to have arrived: held to millimetres it never is, and the motion
    gives up with the hand short of the grasp.
    """

    heading_tolerance: float = 0.15
    """
    How closely the hand has to face a goal before it counts as reached, in radians.
    """

    lift: float = 0.15
    """
    How far the object is lifted clear of what it rested on, in metres.
    """

    placing_reach: float = 0.65
    """
    How far in front of its base Tiago lets the object go, in metres.
    """

    release_clearance: float = 0.02
    """
    How far above where it comes to rest the object's underside is let go, in metres,
    where the hand sets it down.
    """

    drop_from: float = 0.02
    """
    How far above the receptacle the object's underside is let go, in metres.

    A basin takes a drop and a worktop does not, so a task that puts things into one
    says how far above it lets go. Reaching down into a basin is beyond the arm anyway:
    it holds the object over the rim, and gravity does the rest.
    """

    floor_margin: float = 0.03
    """
    How far in from the edge of the receptacle's floor the object is let go, in metres.
    """

    release_step: float = 0.1
    """
    How far apart the places over the receptacle's floor are that letting go is tried
    at, in metres. Coarse, because what is let go above a receptacle falls into it
    wherever over its floor it is let go, and every place costs the arm a solve.
    """

    approach_from_above: float = 0.3
    """
    How far above where the object is let go the hand comes in, in metres.

    The way from where the object was picked up to the receptacle passes over whatever
    else stands on the worktop -- a coffee machine, a kettle -- and a hand carried at
    the height it will let go at sweeps the forearm through it. Coming in from above
    that and dropping onto the place keeps the arm over everything.
    """

    places_to_try: int = 4
    """
    How many of the places over the receptacle are tried from each stand, nearest
    first: a stand the nearest few cannot be reached from is the wrong stand, and every
    place costs the arm a solve.
    """

    release_target: np.ndarray | None = field(default=None, init=False)
    """
    Where the object is let go, once a stand has been found that the arm reaches it
    from.
    """

    grasp_taken: GraspCandidate | None = field(default=None, init=False)
    """
    The grasp the object is picked up by, which is how it sits in the hand for the rest
    of the task.
    """

    rested_at: np.ndarray | None = field(default=None, init=False)
    """
    Where the middle of the object sat when the kitchen was composed.
    """

    minimum_lift: float = 0.05
    """
    How far the object has to have risen above where it rested, in metres, to count as
    having been taken off its support, which is what the pick alone is judged by.
    """

    object_name: str = TaskObject.FOOD
    """
    The name of the body of the object the task moves.
    """

    receptacle_name: str = TaskObject.CONTAINER
    """
    The name of the root body of what the object is to be put into.
    """

    def build_world(self) -> World:
        world = super().build_world()
        self.rested_at = self.extent(self.food.root).center.to_np()[:3].copy()
        return world

    def work_in(self, kitchen: TiagoRoboCasaTask) -> None:
        super().work_in(kitchen)
        self.rested_at = self.extent(self.food.root).center.to_np()[:3].copy()

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the object is in the receptacle, which is all the task asks.

        Not that it rose on the way: a sink's floor lies below the counter the object
        came from, so an object that ends up in one has gone down, not up.
        """
        return self.object_is_in_the_receptacle

    @property
    def lifted(self) -> float:
        """
        :return: How far the object stands above where it rested, in metres.
        """
        return float(self.extent(self.food.root).center.to_np()[2] - self.rested_at[2])

    @property
    def outcome(self) -> str:
        """
        :return: Where the object went, and whether it ended up in the receptacle.
        """
        return (
            f"the object went from {self.rested_at.round(3)} to "
            f"{self.extent(self.food.root).center.to_np()[:3].round(3)}, rising "
            f"{self.lifted * 100:.1f} cm"
            + ("; it is in the receptacle" if self.object_is_in_the_receptacle else "")
        )

    @property
    def first_target(self) -> np.ndarray:
        """
        :return: Where the object is taken hold of: its middle.
        """
        return self.extent(self.food.root).center.to_np()[:3]

    @property
    def handled_bodies(self) -> list[Body]:
        """
        :return: The receptacle and the kitchen's worktops.

        Taking something off a worktop and reaching over it to put it into a receptacle
        set into one brings the whole arm, not only the hand, closer to them than the
        clearance it is otherwise held to: the arm is held off them by eight
        centimetres and a kitchen does not leave that much.
        """
        return self.colliding_bodies_below(self.receptacle) + [
            body
            for counter in self.world.get_semantic_annotations_by_type(CounterTop)
            for body in self.colliding_bodies_below(counter.root)
        ]

    @property
    def grasp_heading(self) -> float:
        """
        :return: The direction the hand approaches in, as a yaw in the kitchen's frame,
            which is the way Tiago is turned where it stands.
        """
        return self.heading

    @property
    def food(self) -> HasGraspCandidates:
        """
        :return: The annotation of the object the task moves.
        """
        body = self.world.get_body_by_name(self.object_name)
        return [
            annotation
            for annotation in self.world.get_semantic_annotations_by_type(
                HasGraspCandidates
            )
            if annotation.root is body
        ][0]

    @property
    def receptacle(self) -> Body:
        """
        :return: The root of what the object is to be put into.
        """
        return self.world.get_body_by_name(self.receptacle_name)

    def grasp(self) -> GraspCandidate:
        """
        :return: The grasp that takes the object at its middle, approaching towards
            :attr:`grasp_heading` and from above at :attr:`grasp_pitch`.
        """
        world_T_grasp = HomogeneousTransformationMatrix.from_xyz_rpy(
            *self.first_target,
            pitch=self.grasp_pitch,
            yaw=self.grasp_heading,
            reference_frame=self.world.root,
        )
        return GraspCandidate(
            self.food, self.world.transform(world_T_grasp.pose, self.food.root)
        )

    @property
    def release_height(self) -> float:
        """
        :return: The height the object's underside is let go at, in metres: the top of
            the receptacle, so that the hand stays out of it and the object falls the
            rest of the way in.
        """
        _, highest = self.box_around(self.colliding_bodies_below(self.receptacle))
        return float(highest[2])

    def release_points(self) -> list[np.ndarray]:
        """
        Where the centre of the object could be let go: over the receptacle's floor and
        just clear of its rim, nearest its middle first.

        The middle rather than the near edge, although the near edge is the easier
        reach: the edge of a sink is where the kettle and the coffee machine stand, and
        an arm that holds the object over it passes through them.

        The object is dropped rather than set down. Reaching to the floor of a sink is
        beyond the arm from anywhere Tiago can stand, while holding the object over the
        rim is not, and what is let go there falls in.

        :return: The places, in the kitchen's frame, the middlemost first.
        """
        floor = self.receptacle_floor
        extent = self.extent(self.food.root)
        height = self.release_height + extent.height / 2 + self.drop_from
        middle = floor.center.to_np()[:2]
        places = [
            np.array([x, y, height])
            for x in self.places_across(
                floor.min_x, floor.max_x, extent.depth / 2 + self.floor_margin
            )
            for y in self.places_across(
                floor.min_y, floor.max_y, extent.width / 2 + self.floor_margin
            )
        ]
        return sorted(
            places, key=lambda place: float(np.linalg.norm(place[:2] - middle))
        )

    def places_across(self, lower: float, upper: float, inset: float) -> list[float]:
        """
        :param lower: Where the receptacle's floor starts along one axis.
        :param upper: Where it ends.
        :param inset: How far in from either end the object has to stay.
        :return: The places to try along that axis, or its middle alone where the
            object only fits there.
        """
        if upper - lower < 2 * inset:
            return [(lower + upper) / 2]
        return list(
            np.arange(
                lower + inset, upper - inset + self.release_step / 2, self.release_step
            )
        )

    def can_let_go_over_the_receptacle(self) -> bool:
        """
        Try the middlemost places the object could be let go at from where Tiago stands
        and keep the first one the arm reaches as :attr:`release_target`.

        :return: Whether the arm reaches any of them.
        """
        for place in self.release_points()[: self.places_to_try]:
            self.release_target = place
            if self.can_reach(self.release_tool_pose()):
                return True
        self.release_target = None
        return False

    @property
    def headings_to_face_the_receptacle(self) -> tuple[float, ...]:
        """
        :return: Which ways to try facing the receptacle: the way Tiago already faces
            first, since the receptacle is usually set into the same counter as what was
            picked up, and the kitchen's other quarter turns after, since it is not
            always. Not the ways a grasp comes in: those say how to take the object up,
            not where to stand to put it down.
        """
        return (self.heading,) + tuple(
            heading for heading in self.quarter_turns if heading != self.heading
        )

    def let_go_where_the_arm_reaches(self) -> Pose:
        """
        Pick the place over the receptacle the object is let go at, from where it picked
        the object up if the arm reaches that far: a receptacle set into the same counter
        is worked without driving anywhere.

        Somewhere else to stand is looked for in front of the receptacle, facing the way
        Tiago already faces first.

        :return: Where Tiago stands to let the object go.
        :raises NoPlaceToLetGo: If the arm reaches no place over the receptacle from
            anywhere it can stand.
        """
        if self.can_let_go_over_the_receptacle():
            logger.info(
                "letting go at %s, from where it picked the object up",
                self.release_target.round(3),
            )
            return self.stands_at
        logger.info(
            "nowhere over the %s can be reached from where the object was picked up;"
            " looking for somewhere to stand in front of it",
            self.receptacle.name.name,
        )
        search = self.stand_that_reaches(
            self.can_let_go_over_the_receptacle,
            self.receptacle_floor.center.to_np()[:3],
            self.placing_reach,
            headings=self.headings_to_face_the_receptacle,
        )
        if search.stand is None:
            raise NoPlaceToLetGo(
                receptacle=self.receptacle.name.name, tried=search.clear
            )
        logger.info(
            "letting go at %s, from in front of the %s",
            self.release_target.round(3),
            self.receptacle.name.name,
        )
        return search.stand

    def goal(self) -> Pose:
        """
        :return: Where the object is to be put: its centre at :attr:`release_target`,
            turned as it stands now.
        """
        body = self.food.root
        world_T_food = body.global_pose.to_np()
        world_T_food[:3, 3] += (
            self.release_target - self.extent(body).center.to_np()[:3]
        )
        return HomogeneousTransformationMatrix(
            world_T_food, reference_frame=self.world.root
        ).pose

    def reaches_the_pre_grasp(self) -> bool:
        """
        :return: Whether the arm gets the hand to where it waits before taking the
            object, worked out afresh because that pose follows Tiago as Tiago is moved
            about.
        """
        return self.can_reach(self.pre_grasp())

    def pre_grasp(self) -> Pose:
        """
        :return: Where the hand waits before it moves onto the grasp, read afresh so it
            follows the robot as the robot is moved about.
        """
        grasp = self.grasp()
        return (
            self.pick_up_of(grasp)
            .grasp_pose_sequence(grasp.world_T_grasp, self.arm.end_effector, grasp)
            .pre_grasp
        )

    def pick_up_of(self, grasp: GraspCandidate) -> PickUpAction:
        """
        :param grasp: The grasp to take.
        :return: The pick up taking it, with the hand creeping the last stretch onto it
            and the fingers alone carrying what they close on.
        """
        arm = self.arm
        return PickUpAction(
            grasp,
            arm,
            approach_clearance=(
                arm.end_effector.fingers_reach_beyond_the_grasp + self.finger_clearance
            ),
            retreat_distance=self.lift,
            pre_approach_linear_velocity=self.carry_velocity,
            final_approach_linear_velocity=self.approach_velocity,
            lift_linear_velocity=self.carry_velocity,
            grasp_closing_velocity=self.closing_velocity,
            grasp_stall_minimum_time=self.stall_minimum_time,
            tolerate_grasp_stall=True,
            held_by_contact_alone=True,
            position_threshold=self.arrival_tolerance,
            orientation_threshold=self.heading_tolerance,
        )

    def release_tool_pose(self) -> Pose:
        """
        :return: Where the tool frame has to be for the object it holds to sit where the
            task wants it, which is the goal carried back through the grasp the object
            is held by.
        """
        return HomogeneousTransformationMatrix(
            self.goal().to_np() @ self.grasp_taken.grasp_pose.to_np(),
            reference_frame=self.world.root,
        ).pose

    def build_plan(self, context: Context) -> Plan:
        """
        Stand where the hand reaches the object, bring the arm there, take the object,
        carry it to the receptacle along a way planned through the kitchen's free space,
        and put it in.

        :param context: The context to plan in.
        :return: The plan.
        """
        self.stand_where_the_hand_arrives(self.reaches_the_pre_grasp)
        self.grasp_taken = self.grasp()
        pick_up = self.pick_up_of(self.grasp_taken)
        taking = [
            ParkArmsAction(self.robot.all_arms),
            *self.arm_motions_to(self.pre_grasp()),
            pick_up,
        ]
        pick_stand = self.robot.root.parent_connection.origin
        place_stand = self.let_go_where_the_arm_reaches()
        self.robot.root.parent_connection.origin = pick_stand
        return sequential(
            taking
            + self.drive_to(place_stand)
            + self.letting_the_object_go()
            + [ParkArmsAction(self.robot.all_arms)],
            context=context,
        ).plan

    def letting_the_object_go(self) -> list[BaseMotion]:
        """
        Hold the object where it is to be let go, open the fingers and lift the hand
        off it.

        The hand does not lower the object the last stretch, as setting one down does:
        over a sink the arm is already at full stretch holding it over the rim, and the
        goal below that is one it reaches on paper and stalls short of in the physics.

        :return: The motions.
        """
        return [
            MoveToolCenterPointMotion(
                self.release_tool_pose(),
                self.arm,
                allow_gripper_collision=True,
                max_linear_velocity=self.carry_velocity,
                position_threshold=self.release_tolerance,
                orientation_threshold=self.heading_tolerance,
            ),
            MoveGripperMotion(
                GripperState.OPEN,
                self.arm.end_effector,
                allow_gripper_collision=True,
            ),
            MoveToolCenterPointMotion(
                self.carried_above_the_release(),
                self.arm,
                allow_gripper_collision=True,
                hold_gripper=GripperState.OPEN,
                max_linear_velocity=self.carry_velocity,
                position_threshold=self.release_tolerance,
                orientation_threshold=self.heading_tolerance,
            ),
        ]

    def carried_above_the_release(self) -> Pose:
        """
        :return: Where the hand goes once it has let the object go: straight up off it,
            so that the fingers do not sweep it along as they leave.
        """
        world_T_tool = self.release_tool_pose().to_np()
        world_T_tool[2, 3] += self.approach_from_above
        return HomogeneousTransformationMatrix(
            world_T_tool, reference_frame=self.world.root
        ).pose

    release_tolerance: float = 0.08
    """
    How close the hand has to come to where it lets the object go, and to where it
    lifts off it afterwards, in metres: an arm at full stretch over a basin stalls a
    few centimetres short of the release point, and an object let go there still falls
    onto the basin's floor.
    """

    resting_tolerance: float = 0.05
    """
    How far above the receptacle's floor the object's underside may come to rest and
    still count as resting on it, in metres: in a sink it may land on what is already
    in the basin.
    """

    @property
    def object_is_in_the_receptacle(self) -> bool:
        """
        :return: Whether the object has come to rest on one of the receptacle's floors,
            which is what being put there means, whether the receptacle is a basin it
            falls into or a top it lands on. A floor is any level piece of the
            receptacle at least as wide as the object, so that each basin of a double
            sink counts and a rim the object is perched on does not. An object still
            held over the receptacle, or lying under it, as in a drawer below a
            counter, is not in it.
        """
        box = self.extent(self.food.root)
        here = box.center.to_np()[:3]
        return any(
            piece.min_x <= here[0] <= piece.max_x
            and piece.min_y <= here[1] <= piece.max_y
            and abs(box.min_z - piece.max_z) <= self.resting_tolerance
            for piece in self.receptacle_pieces
            if piece.width >= box.width and piece.depth >= box.depth
        )

    @property
    def receptacle_pieces(self) -> list[VolumetricBoundingBox]:
        """
        :return: The convex pieces the receptacle's collision geometry is built from.
        """
        return [
            piece
            for body in self.colliding_bodies_below(self.receptacle)
            for piece in body.collision.as_bounding_box_collection_in_frame(
                self.world.root
            ).bounding_boxes
        ]

    @property
    def receptacle_floor(self) -> VolumetricBoundingBox:
        """
        :return: The floor of the receptacle the object is let go over, as the widest
            of its pieces.
        """
        return max(self.receptacle_pieces, key=lambda piece: piece.width * piece.depth)

    def prepare_contacts(self) -> None:
        """
        Have the object's contacts resolve the twisting and rolling friction it is
        given, so it neither turns in the hand nor rolls on once it is let go.
        """
        for shape in self.food.root.collision.shapes:
            shape.get_simulator_property_of_type(MujocoGeom).contact_dimensionality = (
                ContactDimensionality.SLIDING_TORSIONAL_AND_ROLLING
            )


# %% the tasks


@dataclass
class PickPlaceCounterToSink(TiagoPickPlaceTask):
    """
    Tiago takes the object off the counter and puts it in the sink.
    """

    seed: int = 0
    """
    The seed RoboCasa samples the task with: the object near the counter's edge, right
    of the sink.
    """

    food_category: str | None = None
    """
    Which kind of object RoboCasa is to put on the counter, or nothing to let it sample
    whatever it likes.

    Left open: holding the category fixed pins the object to the same corner of the
    counter every time, and Tiago cannot stand at that corner.
    """

    @property
    def task_parameters(self) -> dict[str, Any]:
        """
        :return: What RoboCasa is told about the task beyond its name.
        """
        if self.food_category is None:
            return {}
        return {"obj_groups": self.food_category}

    @property
    def receptacle(self) -> Body:
        """
        :return: The sink, which is a fixture of the kitchen rather than an object the
            task placed.
        """
        return self.world.get_semantic_annotations_by_type(Sink)[0].root

    drop_from: float = 0.2
    """
    Well above the rim: the arm holds the object over a sink at full stretch and stalls
    short of any goal below that, so it lets go high and the carton falls in.
    """

    @property
    def release_height(self) -> float:
        """
        :return: The sink's rim, which the object is dropped in from above, read off the
            sink rather than off what the counters were set to.
        """
        return float(self.extent(self.receptacle).max_z)


@dataclass
class PickPlaceCounterToStove(TiagoPickPlaceTask):
    """
    Tiago takes the food off the counter and puts it in the pan on the stove.

    The pan stands on the hob at about counter height, so the hand sets the food down
    onto a surface rather than reaching down into a basin.
    """

    seed: int = 2
    """
    The seed RoboCasa samples the task with.
    """


@dataclass
class PickPlaceDrawerToCounter(TiagoPickPlaceTask, TiagoDrawerTask):
    """
    Tiago takes the object out of the open drawer and puts it on the counter above it.

    RoboCasa composes this task with the drawer already drawn out, which is what puts
    the object within reach; opening a drawer is :class:`OpenDrawer`'s own task.
    """

    seed: int = 9
    """
    The seed RoboCasa samples the task with.
    """

    grasp_pitch: float = math.pi / 3
    """
    Steeper than over a counter: the hand reaches down into an open drawer, and a
    shallower approach brings the wrist against the drawer's front.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The end of the drawer's rails, which RoboCasa has already drawn the
            drawer most of the way to when it composes this task.
        """
        return float(self.mechanism.dof.limits.upper.position)

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the object now rests on the counter.
        """
        return self.object_is_in_the_receptacle

    @property
    def receptacle(self) -> Body:
        """
        :return: The counter above the drawer.
        """
        return self.task.referenced_fixtures[FixtureReference.COUNTER]

    @property
    def handled_bodies(self) -> list[Body]:
        """
        :return: The drawer's fixture, which Tiago stands against, and the counter it
            reaches across.
        """
        return super().handled_bodies + self.colliding_bodies_below(self.receptacle)


@dataclass
class PickPlaceCounterToDrawer(TiagoPickPlaceTask, TiagoDrawerTask):
    """
    Tiago takes the object off the counter and drops it into the drawer below, which
    RoboCasa has drawn out.
    """

    drop_from: float = 0.1
    """
    Above the drawer's sides: an arm reaching down into a drawer brings the wrist
    against its front, so the object is let go over it and falls in.
    """

    @property
    def goal_position(self) -> float:
        """
        :return: The end of the drawer's rails, which RoboCasa has drawn the drawer
            towards.
        """
        return float(self.mechanism.dof.limits.upper.position)

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the object now lies in the drawer.
        """
        return self.object_is_in_the_receptacle

    @property
    def receptacle(self) -> Body:
        """
        :return: The part of the drawer that slides out, which the object goes into.
        """
        return self.drawer.root

    @property
    def release_height(self) -> float:
        """
        :return: The top of the drawer's sides.
        """
        _, highest = self.box_around(self.colliding_bodies_below(self.receptacle))
        return float(highest[2])

    @property
    def handled_bodies(self) -> list[Body]:
        """
        :return: The drawer's fixture, which Tiago stands against, and the counter the
            object is taken from.
        """
        return super().handled_bodies + self.colliding_bodies_below(
            self.task.referenced_fixtures[FixtureReference.COUNTER]
        )


@dataclass
class CounterContact:
    """
    What an object lies on: a body of the kitchen and the piece of it under the object.
    """

    body: Body
    """
    The body.
    """

    top: VolumetricBoundingBox
    """
    The piece of the body the object lies on.
    """


@dataclass
class MoveAlongTheCounter(PickPlaceCounterToSink):
    """
    Tiago picks the object up off the counter and lets it go just above the same
    counter a hand's breadth or two to one side, in the scene of RoboCasa's
    ``PickPlaceCounterToSink`` task.

    Where it is set down is fixed relative to where it lay rather than chosen by
    RoboCasa: along the counter, to either side.
    """

    shifts: tuple[float, ...] = (0.3, -0.3, 0.2, -0.2)
    """
    How far along the counter from where it lay the object may be set down, in
    metres, the first the arm reaches being taken.
    """

    minimum_shift: float = 0.15
    """
    How far the object has to end up from where it lay for the task to count as done,
    in metres.
    """

    contact_tolerance: float = 0.03
    """
    How far above a piece of the kitchen the object's underside may be and still count
    as lying on it, in metres.
    """

    drop_from: float = 0.1
    """
    A hand's breadth above the counter: measured beside a RoboCasa sink, the arm
    reaches no point along the counter lower than that, with the object in hand, from
    where it picked the object up.
    """

    @classproperty
    def task_name(cls) -> str:
        """
        RoboCasa's ``PickPlaceCounterToSink``, which lays the object on the counter.
        """
        return PickPlaceCounterToSink.__name__

    @cached_property
    def counter(self) -> CounterContact:
        """
        :return: The body the object lies on when first asked, before Tiago has
            touched it, and the piece of it under the object.
        """
        box = self.extent(self.food.root)
        here = box.center.to_np()[:3]
        robot_bodies = set(self.robot.bodies_with_collision)
        return next(
            CounterContact(body=body, top=piece)
            for body in self.world.bodies
            if body.has_collision()
            and body is not self.food.root
            and body not in robot_bodies
            for piece in body.collision.as_bounding_box_collection_in_frame(
                self.world.root
            ).bounding_boxes
            if piece.min_x <= here[0] <= piece.max_x
            and piece.min_y <= here[1] <= piece.max_y
            and abs(box.min_z - piece.max_z) <= self.contact_tolerance
        )

    @property
    def receptacle(self) -> Body:
        """
        :return: The counter the object lies on.
        """
        return self.counter.body

    @property
    def release_height(self) -> float:
        """
        :return: The top of the counter.
        """
        return float(self.counter.top.max_z)

    def release_points(self) -> list[np.ndarray]:
        """
        :return: The places along the counter the object may be set down at, centred,
            at :attr:`shifts` from where it lay.
        """
        top = self.counter.top
        along = np.zeros(3)
        along[0 if top.width >= top.depth else 1] = 1.0
        height = (
            self.release_height
            + self.extent(self.food.root).height / 2
            + self.drop_from
        )
        return [
            np.array([*(self.rested_at[:2] + shift * along[:2]), height])
            for shift in self.shifts
        ]

    @property
    def task_is_done(self) -> bool:
        """
        :return: Whether the object lies on the counter again, at least
            :attr:`minimum_shift` from where it lay.
        """
        here = self.extent(self.food.root).center.to_np()[:3]
        moved = float(np.linalg.norm(here[:2] - self.rested_at[:2]))
        return self.object_is_in_the_receptacle and moved >= self.minimum_shift


TASKS = {
    "PickPlaceCounterToSink": PickPlaceCounterToSink,
    "PickPlaceCounterToStove": PickPlaceCounterToStove,
    "PickPlaceDrawerToCounter": PickPlaceDrawerToCounter,
    "PickPlaceCounterToDrawer": PickPlaceCounterToDrawer,
    "MoveAlongTheCounter": MoveAlongTheCounter,
}
"""
The RoboCasa tasks Tiago can be asked for, by the name RoboCasa gives them.
"""


# %% running one from the command line


def main() -> None:
    """
    Run the task the command line names over one kitchen after another.
    """
    logging.basicConfig(level=logging.INFO, force=True)
    arguments = parser_of(__doc__, list(TASKS)).parse_args()
    settings = settings_from(arguments)
    Trials(
        task=TASKS[arguments.task],
        settings=settings,
        attempts=arguments.attempts,
        tries_every_kitchen=arguments.benchmark,
    ).run()


if __name__ == "__main__":
    main()
