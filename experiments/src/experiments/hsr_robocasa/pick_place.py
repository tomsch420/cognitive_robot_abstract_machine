"""
The Toyota HSR performs RoboCasa's pick-and-place tasks in a RoboCasa kitchen,
simulated in MuJoCo: it picks an object up where the task puts it and sets it into the
receptacle the task names.

RoboCasa composes the kitchen and samples the objects; the HSR replaces the robot
RoboCasa would use. The object is held by nothing but the friction between it and the
fingers.

Run the stove task with::

    python -m experiments.hsr_robocasa.pick_place
"""

from __future__ import annotations

import math
from abc import ABC
from dataclasses import dataclass
from enum import StrEnum
from typing_extensions import Any, Callable

import numpy as np
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.container import OpenAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from segmind.detectors.agent_event_detector_nodes import GraspDetector
from segmind.detectors.coarse_event_detector_nodes import PlacingDetector
from segmind.event_segmentation import Segmind
from semantic_digital_twin.adapters.multi_sim import (
    ContactDimensionality,
    MujocoGeom,
)
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    CounterTop,
    Sink,
)
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.world_entity import Body

from experiments.hsr_robocasa.drawers import HSRTaskAtDrawer
from experiments.hsr_robocasa.robocasa_task import (
    ArmLink,
    FixtureReference,
    HSRRoboCasaTask,
)

# %% the task's objects


class TaskObject(StrEnum):
    """
    The bodies RoboCasa adds for a pick-and-place task, by the names it gives them.
    """

    FOOD = "obj_main"
    """
    The object to pick up and set down, food in most tasks.
    """

    CONTAINER = "container_main"
    """
    The container the object is to be set into, where the task has one.
    """


@dataclass
class HSRPickPlaceTask(HSRRoboCasaTask, ABC):
    """
    One of RoboCasa's pick-and-place tasks, performed by the HSR.
    """

    food_category: str | None = None
    """
    The RoboCasa object category the object to pick up is sampled from; ``None``
    leaves the choice to the task.
    """

    placing_reach: float = 0.6
    """
    How far in front of its base's centre the HSR sets the food down, in metres.
    """

    lift: float = 0.1
    """
    How far the HSR lifts the food from where it took hold of it, in metres, before
    carrying it to the receptacle.
    """

    release_clearance: float = 0.03
    """
    How far above :attr:`release_height` the HSR lets the food go, in metres.
    """

    floor_margin: float = 0.05
    """
    How far in from the edge of the receptacle's floor the food is set down, in
    metres.
    """

    @property
    def task_parameters(self) -> dict[str, Any]:
        """
        The object category the food is to be sampled from, when :attr:`food_category`
        names one.
        """
        return {} if self.food_category is None else {"obj_groups": self.food_category}

    @property
    def handled_bodies(self) -> list[Body]:
        """
        The task's objects and the receptacle.
        """
        return [
            body
            for root in [*self.task.manipulated_objects, self.receptacle]
            for body in self.colliding_bodies_below(root)
        ]

    @property
    def first_target(self) -> np.ndarray:
        """
        Where the food is taken hold of.
        """
        return self.grasp_point

    @property
    def grasp_point(self) -> np.ndarray:
        """
        Where the food is taken hold of: its middle.
        """
        return self.extent(self.food.root).center.to_np()[:3]

    @property
    def grasp_heading(self) -> float:
        """
        The direction the hand approaches the food in, and the HSR faces while taking
        it, as a yaw in the kitchen's frame: towards the counter.
        """
        return math.pi / 2

    @property
    def food(self) -> HasGraspCandidates:
        """
        The annotation of the food.
        """
        food = self.world.get_body_by_name(TaskObject.FOOD)
        return [
            annotation
            for annotation in self.world.get_semantic_annotations_by_type(
                HasGraspCandidates
            )
            if annotation.root is food
        ][0]

    @property
    def receptacle(self) -> Body:
        """
        The root of what the object is to be set into: the task's container.
        """
        return self.world.get_body_by_name(TaskObject.CONTAINER)

    @property
    def receptacle_floor(self) -> VolumetricBoundingBox:
        """
        The floor of the receptacle, in the kitchen's frame: the widest of the convex
        pieces RoboCasa builds its collision geometry from.
        """
        pieces = [
            piece
            for body in self.colliding_bodies_below(self.receptacle)
            for piece in body.collision.as_bounding_box_collection_in_frame(
                self.world.root
            ).bounding_boxes
        ]
        return max(pieces, key=lambda piece: piece.width * piece.depth)

    def build_segmind(self) -> Segmind:
        """
        :return: SegMind watching the food be taken hold of and put down.
        """
        return Segmind.create_for_bodies(
            self.world, [self.food.root], [GraspDetector, PlacingDetector]
        )

    def grasp(self) -> GraspCandidate:
        """
        :return: The grasp that takes the food at :attr:`grasp_point`, approaching it
            towards :attr:`grasp_heading` and from above at :attr:`grasp_pitch`, the
            fingers closing across that heading.
        """
        food = self.food.root
        world_T_grasp = HomogeneousTransformationMatrix.from_xyz_rpy(
            *self.grasp_point,
            pitch=self.grasp_pitch,
            yaw=self.grasp_heading,
            reference_frame=self.world.root,
        )
        food_T_grasp = self.world.transform(world_T_grasp.pose, food)
        return GraspCandidate(self.food, food_T_grasp)

    @property
    def release_point(self) -> np.ndarray:
        """
        Where the centre of the food is to be let go: its underside
        :attr:`release_clearance` above :attr:`release_height`, over the part of the
        receptacle's floor nearest the HSR but at least :attr:`floor_margin` in from
        its edge.
        """
        floor = self.receptacle_floor
        food_extent = self.extent(self.food.root)
        floor_center = floor.center.to_np()[:3]
        return np.array(
            [
                floor_center[0],
                min(
                    floor_center[1],
                    floor.min_y + food_extent.width / 2 + self.floor_margin,
                ),
                self.release_height + food_extent.height / 2 + self.release_clearance,
            ]
        )

    @property
    def release_height(self) -> float:
        """
        The height above which the food is let go: the top of the receptacle's floor.
        """
        return self.receptacle_floor.max_z

    def goal(self) -> Pose:
        """
        :return: Where the food is to be put: its centre at :attr:`release_point`,
            turned as it is now.
        """
        food = self.food.root
        world_T_food = food.global_pose.to_np()
        world_T_food[:3, 3] += self.release_point - self.extent(food).center.to_np()[:3]
        return HomogeneousTransformationMatrix(
            world_T_food, reference_frame=self.world.root
        ).pose

    def build_plan(self, context: Context) -> Plan:
        """
        Stand in line with the food, move the arm to where the pick up starts its
        approach, pick the food up, drive to the receptacle holding it, and put it in.
        """
        robot = context.robot
        arm = robot.all_arms[0]
        grasp = self.grasp()
        pick_up = PickUpAction(grasp, arm, retreat_distance=self.lift)
        pre_grasp = pick_up.grasp_pose_sequence(
            grasp.world_T_grasp, arm.end_effector, grasp
        ).pre_grasp
        standing_pose = self.standing_pose(
            self.first_target, self.reach, self.grasp_heading
        )
        return sequential(
            [
                ParkArmsAction(robot.all_arms),
                NavigateAction(standing_pose),
                *self.arm_motions_to(pre_grasp, standing_pose),
                pick_up,
                NavigateAction(
                    self.standing_pose(self.release_point, self.placing_reach)
                ),
                PlaceAction(self.food, self.goal()),
                ParkArmsAction(robot.all_arms),
            ],
            context=context,
        ).plan

    def prepare_contacts(self) -> None:
        """
        Have the food's contacts resolve the twisting and rolling friction RoboCasa
        gives it, so that it neither turns in the hand nor rolls on without end.
        """
        for shape in self.food.root.collision.shapes:
            shape.get_simulator_property_of_type(MujocoGeom).contact_dimensionality = (
                ContactDimensionality.SLIDING_TORSIONAL_AND_ROLLING
            )


# %% the tasks


@dataclass
class PickPlaceCounterToStove(HSRPickPlaceTask):
    """
    Pick the food from the plate on the counter and place it in the pan on the stove.
    """

    seed: int = 2
    """
    The seed RoboCasa samples the task with: the plate on the counter left of the
    stove, near its edge.
    """

    food_category: str | None = "cheese"
    """
    A block of cheese, which lies still on its plate.
    """


@dataclass
class PickPlaceStoveToCounter(HSRPickPlaceTask):
    """
    Pick the food from the pan on the stove and place it on the plate or in the bowl
    on the counter.
    """

    seed: int = 12
    """
    The seed RoboCasa samples the task with: the pan on the front left burner.
    """

    food_category: str | None = "lemon"
    """
    A lemon, small enough for the HSR's hand.
    """


@dataclass
class PickPlaceCounterToSink(HSRPickPlaceTask):
    """
    Pick the object from the counter and place it in the sink.
    """

    seed: int = 0
    """
    The seed RoboCasa samples the task with: the object near the counter's edge right
    of the sink.
    """

    food_category: str | None = "canned_food"
    """
    A can, standing upright.
    """

    @property
    def receptacle(self) -> Body:
        return self.world.get_semantic_annotations_by_type(Sink)[0].root

    @property
    def release_height(self) -> float:
        """
        The sink's rim, level with the counter: the object is dropped into the sink
        from just above it.
        """
        return self.counter_height


@dataclass
class PickPlaceSinkToCounter(HSRPickPlaceTask):
    """
    Pick the food from the sink and place it on the plate on the counter.
    """

    seed: int = 4
    """
    The seed RoboCasa samples the task with: a flat plate on the counter right of the
    sink.
    """

    food_category: str | None = "yogurt"
    """
    A pot of yogurt.
    """

    grasp_pitch: float = math.pi / 3
    """
    Steeper than on the counter, so that the wrist passes above the sink's front wall
    while the hand reaches down to the food below the rim.
    """

    placing_reach: float = 0.52
    """
    Closer than when taking the food from the sink: held at :attr:`grasp_pitch`, the
    food has to be lifted above the plate, and the arm reaches that high only up to
    0.54 m in front of the base.
    """

    lift: float = 0.2
    """
    Out of the sink and above the counter's top: carried lower, the food's underside
    stays below the top and catches on the counter's edge beside the sink.
    """

    tool_frame_tolerance: float = 0.02
    """
    Reaching down into the sink, the arm comes within its collision buffer of the
    counter's edge and the sink's rim 1.6 cm before the grasp, with the food already
    between the fingers.
    """


@dataclass
class PickPlaceDrawerToCounter(HSRPickPlaceTask, HSRTaskAtDrawer):
    """
    Pick the object from the open drawer and place it on the counter.
    """

    counter_height: float = HSRTaskAtDrawer.counter_height
    """
    As for every task at the drawer, see :attr:`HSRTaskAtDrawer.counter_height`.
    """

    seed: int = 9
    """
    The seed RoboCasa samples the task with: a rolling pin lying across the drawer
    diagonally.
    """

    grasp_pitch: float = math.pi / 3
    """
    Steep, so that the arm reaching into the drawer passes above the drawer's front.
    """

    reach: float = 0.58
    """
    The base stands behind the fully opened drawer's handle, and with the hand tilted
    down at :attr:`grasp_pitch` the arm reaches 0.46 m to 0.58 m ahead at the drawer's
    floor.
    """

    placing_reach: float = 0.52
    """
    Held at :attr:`grasp_pitch`, the object has to be lowered onto the counter, and the
    arm reaches that low only up to 0.54 m in front of the base.
    """

    lift: float = 0.2
    """
    Above the drawer's front and the counter's top: carried lower, the long object
    swings into the drawer's front as the HSR drives off.
    """

    tool_frame_tolerance: float = 0.025
    """
    Rolled to close across the object, the wrist moves the tool frame up to 2 cm
    sideways out of the plane the arm swings in, which the arm cannot make up for.
    """

    @property
    def stages(self) -> list[Callable[[Context], Plan]]:
        """
        Pull the drawer fully open, which brings the object out from under the
        counter's top and the drawer's front further from it, then take the object
        from where it came to rest and put it on the counter.
        """
        return [self.build_opening_plan, self.build_plan]

    def build_opening_plan(self, context: Context) -> Plan:
        """
        :param context: The context the actions are built in.
        :return: The plan that pulls the drawer fully open.
        """
        return self.handle_plan(context, OpenAction)

    @property
    def counter(self) -> Body:
        """
        The counter above the drawer.
        """
        return self.task.referenced_fixtures[FixtureReference.COUNTER]

    def top_pieces(self, counter: Body) -> list[VolumetricBoundingBox]:
        """
        :param counter: A counter of the kitchen.
        :return: The pieces of the counter's top, in the kitchen's frame.
        """
        return [
            piece
            for piece in counter.collision.as_bounding_box_collection_in_frame(
                self.world.root
            ).bounding_boxes
            if math.isclose(piece.max_z, self.counter_height, abs_tol=0.01)
        ]

    @property
    def counters(self) -> list[Body]:
        """
        Every counter of the kitchen, any of which the object may be placed on.
        """
        return [
            annotation.root
            for annotation in self.world.get_semantic_annotations_by_type(CounterTop)
        ]

    @property
    def long_axis(self) -> np.ndarray:
        """
        The horizontal direction the object is longest along, pointing towards the
        counter.
        """
        food = self.food.root
        extent = food.collision.as_bounding_box_collection_in_frame(food).bounding_box()
        rotation = food.global_pose.to_np()[:3, :3]
        axis = rotation[:2, 0] if extent.depth >= extent.width else rotation[:2, 1]
        axis = axis / np.linalg.norm(axis)
        return axis if axis[1] >= 0.0 else -axis

    def grasp(self) -> GraspCandidate:
        """
        :return: The grasp that takes the object at :attr:`grasp_point`, approaching
            it towards :attr:`grasp_heading` and from above at :attr:`grasp_pitch`, the
            fingers closing across the object's long axis.
        """
        approach = np.array(
            [
                math.cos(self.grasp_heading) * math.cos(self.grasp_pitch),
                math.sin(self.grasp_heading) * math.cos(self.grasp_pitch),
                -math.sin(self.grasp_pitch),
            ]
        )
        closing = np.cross(approach, [*self.long_axis, 0.0])
        closing /= np.linalg.norm(closing)
        world_T_grasp = np.eye(4)
        world_T_grasp[:3, 0] = approach
        world_T_grasp[:3, 1] = closing
        world_T_grasp[:3, 2] = np.cross(approach, closing)
        world_T_grasp[:3, 3] = self.grasp_point
        food_T_grasp = self.world.transform(
            HomogeneousTransformationMatrix(
                world_T_grasp, reference_frame=self.world.root
            ).pose,
            self.food.root,
        )
        return GraspCandidate(self.food, food_T_grasp)

    @property
    def receptacle(self) -> Body:
        """
        The counter :attr:`receptacle_floor` is a piece of.
        """
        floor = self.receptacle_floor
        return [
            counter for counter in self.counters if floor in self.top_pieces(counter)
        ][0]

    @property
    def receptacle_floor(self) -> VolumetricBoundingBox:
        """
        The widest piece of a counter's top that does not stand above the open
        drawer, which keeps the HSR from reaching across it; of equally wide ones, the
        one nearest the drawer.
        """
        drawer = self.extent(self.drawer.root)
        beside_the_drawer = [
            piece
            for counter in self.counters
            for piece in self.top_pieces(counter)
            if piece.max_x <= drawer.min_x or piece.min_x >= drawer.max_x
        ]
        return max(
            beside_the_drawer,
            key=lambda piece: (
                round(piece.width * piece.depth, 3),
                -abs(piece.center.to_np()[0] - drawer.center.to_np()[0]),
            ),
        )

    @property
    def bodies_touching_handled_bodies(self) -> list[Body]:
        """
        The wrist's links and the forearm, which the drawer's front comes towards as
        the drawer opens.
        """
        return self.arm_links(list(ArmLink))

    @property
    def handled_bodies(self) -> list[Body]:
        """
        The object, the counter it is placed on, and the drawer it is taken out of
        with the counter above it.
        """
        return (
            super().handled_bodies
            + self.colliding_bodies_below(self.drawer.root)
            + self.colliding_bodies_below(self.counter)
        )


if __name__ == "__main__":
    experiment = PickPlaceCounterToStove()
    experiment.build_world()
    experiment.run()
