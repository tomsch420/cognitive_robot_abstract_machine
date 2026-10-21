"""
What Tiago's RoboCasa tasks share: a RoboCasa kitchen lowered to where Tiago reaches,
the robot standing in front of what it is to take hold of, and the whole thing carried
out against MuJoCo physics with nothing attached to the hand.

The kitchen, the objects and where they stand are RoboCasa's. Tiago replaces RoboCasa's
own robot, and every object it moves is held by the grip of its fingers alone.
"""

from __future__ import annotations

from enum import StrEnum

import math
from abc import ABC, abstractmethod
from contextlib import nullcontext
from dataclasses import dataclass, field
from datetime import timedelta
from typing import Any, Callable

import mujoco
import numpy as np
from krrood.ormatic.utils import classproperty

from coraplex.datastructures.dataclasses import Context, MotionToleranceConfig
from coraplex.execution_environment import PhysicallySimulatedRobot
from coraplex.datastructures.enums import ExecutionType
from coraplex.execution_environment import ExecutionEnvironment
from coraplex.plans.factories import sequential
from coraplex.plans.failures import PlanFailure
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.motions.gripper import MoveToolCenterPointMotion
from coraplex.robot_plans.motions.robot_body import MoveJointsMotion
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from experiments.hsr_robocasa.robocasa_task import RoboCasaLayout
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType
from experiments.tiago_robocasa.exceptions import (
    NoClearStandingPose,
    NoReachableStandingPose,
    NoWayToTheStand,
)
from experiments.scene_recording import SceneRecorder, SceneRecording, ViewerCamera
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
    RoboCasaTask,
)
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import (
    StaticJointState,
    TorsoState,
)
from semantic_digital_twin.robots.tiago import TiagoMujoco
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.spatial_types.spatial_types import Point2, Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from random_events.interval import closed
from random_events.product_algebra import SimpleEvent
from semantic_digital_twin.datastructures.variables import SpatialVariables
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    PlanarGraphOfBoundingBoxes,
)
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)
from semantic_digital_twin.world_description.world_entity import Body


class KitchenBodyPrefix(StrEnum):
    """
    How RoboCasa begins the names of the kitchen's bodies this experiment looks for.
    """

    FLOOR = "floor"
    """
    The floor and the slab under it.
    """


# %% looking for somewhere to stand


@dataclass
class StandCandidate:
    """
    One stand to try: how far it steps back and along from the stand that puts the
    working hand right in front of the target, and which way round Tiago is.
    """

    back: float
    """
    How much further back from the target than the task's reach it stands, in metres.
    """

    along: float
    """
    How far it steps to the robot's left along the counter, in metres.
    """

    heading: float
    """
    Which way it faces, as a yaw in the kitchen's frame.
    """


@dataclass
class StandSearch:
    """
    What came of looking for a stand the arm reaches a target from.
    """

    target: np.ndarray
    """
    What was to be reached.
    """

    stand: Pose | None
    """
    Where Tiago stands to reach it, or nothing if no stand tried does.
    """

    tried: int
    """
    How many stands were tried.
    """

    clear: int
    """
    How many of them kept Tiago clear of the kitchen.
    """


# %% one of RoboCasa's tasks, performed by Tiago


@dataclass
class TiagoRoboCasaTask(ABC):
    """
    One of RoboCasa's tasks, performed by Tiago.

    Each task is a subclass named after the RoboCasa task it performs.
    """

    layout: Any = None
    """
    The RoboCasa kitchen layout; the first one unless a subclass says otherwise.
    """

    style: Any = None
    """
    The RoboCasa kitchen style.
    """

    seed: int = 0
    """
    The seed RoboCasa samples the task's objects and their places with.
    """

    counter_height: float | None = None
    """
    How high the kitchen's counters are made to stand, in metres, or nothing to leave
    RoboCasa's kitchen at the size it ships.

    Tiago reaches RoboCasa's own counters, so the kitchen is not shrunk for it the way
    it has to be for a robot that cannot.
    """

    reach: float = 0.6
    """
    How far in front of its base's centre Tiago takes hold of what it reaches for, in
    metres.
    """

    grasp_pitch: float = math.pi / 4
    """
    How steeply the hand comes down onto what it takes hold of, in radians below the
    horizontal.
    """

    stand_step: float = 0.05
    """
    How far the stand is moved at a time while looking for a clear one, in metres.
    """

    stand_search: float = 0.4
    """
    How far back of, and along from, the ideal stand a clear one may be looked for, in
    metres.
    """

    tool_frame_tolerance: float = 0.015
    """
    How far, in metres, the tool frame may stop from a goal of the task's motions.
    """

    base_follows_the_arm: bool = False
    """
    Whether the base is driven along with the arm while the plan runs, instead of
    staying where the task stood it.

    The stand and the arm's way to it are worked out for the arm alone either way; what
    this adds is the few centimetres of base travel a goal just outside the arm's
    working envelope needs.
    """

    collision_avoidance: bool = True
    """
    Whether Giskard keeps Tiago clear of the kitchen and of itself.
    """

    step_size: float = 2.5e-4
    """
    How much simulated time one physics step covers, in seconds.

    A quarter of the millisecond the rest of the kitchen is happy with: Tiago's wrist
    joints are held by servos stiff enough that a longer step lets them oscillate
    apart, and the arm runs away to tens of thousands of radians.
    """

    settling_time: timedelta = timedelta(seconds=1)
    """
    How long the scene is left to settle before Tiago starts, so nothing the kitchen was
    composed with is still falling when the task begins.
    """

    resting_time: timedelta = timedelta(seconds=3)
    """
    How long the physics runs on after the plan, so what was let go lands and comes to
    rest before anything is read off it.
    """

    integrator: Any = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    """
    How MuJoCo advances the physics.

    The semi-implicit one, rather than the explicit default: Tiago's joints are held by
    stiff servos, and stepping those explicitly lets a joint run away to hundreds of
    thousands of radians before the simulation gives up.
    """

    solver: Any = mujoco.mjtSolver.mjSOL_CG
    """
    Which constraint solver MuJoCo uses.

    The conjugate-gradient one, rather than MuJoCo's default Newton: Newton factorizes a
    Hessian that this kitchen, with the robot's hand among its fixtures, makes rank-
    deficient, and the simulation stops dead when it does.
    """

    friction_cone: Any = mujoco.mjtCone.mjCONE_ELLIPTIC
    """
    How MuJoCo bounds a contact's friction.

    The elliptic cone is the exact one; the default pyramid only approximates it and
    lets a squeezed object creep out of the hand.
    """

    noslip_iterations: int = 10
    """
    How many passes MuJoCo spends removing the slip its soft contacts leave, so a held
    object stays where the fingers hold it.
    """

    impedance_ratio: float = 10.0
    """
    How much stiffer MuJoCo makes friction than the push along a contact's normal.
    """

    headless: bool = True
    """
    Whether the simulation runs without MuJoCo's own viewer.
    """

    recording: SceneRecording | None = None
    """
    Where to write a video of the run, or nothing to run without recording one.
    """

    task: RoboCasaTask = field(init=False)
    """
    The task as RoboCasa composed it, built by :meth:`build_world`.
    """

    world: World = field(init=False)
    """
    The kitchen with the task's objects and Tiago, built by :meth:`build_world`.
    """

    @classproperty
    def task_name(cls) -> str:
        """
        The name of the RoboCasa task, which the subclass carries.
        """
        return cls.__name__

    @property
    def task_parameters(self) -> dict[str, Any]:
        """
        :return: What RoboCasa is told about the task beyond its name.
        """
        return {}

    @property
    @abstractmethod
    def first_target(self) -> np.ndarray:
        """
        :return: Where in the kitchen the hand first has to reach, which is where Tiago
            is stood.
        """

    @abstractmethod
    def build_plan(self, context: Context) -> Plan:
        """
        :param context: The context to plan in.
        :return: The plan carrying the task out.
        """

    @property
    @abstractmethod
    def task_is_done(self) -> bool:
        """
        :return: Whether the scene now stands as the task asks it to.
        """

    @property
    @abstractmethod
    def outcome(self) -> str:
        """
        :return: What became of what the task moves, as a line to be read in a report.
        """

    @property
    def robot(self) -> TiagoMujoco:
        """
        :return: Tiago, in the built world.
        """
        return self.world.get_semantic_annotations_by_type(TiagoMujoco)[0]

    @property
    def quarter_turns(self) -> tuple[float, ...]:
        """
        :return: The ways a kitchen is worked from, as yaws in its frame.

        A RoboCasa kitchen runs its counters along both of the room's axes and sets its
        islands free of the walls, so which way the robot has to face to work at what
        the task sampled is not known before the kitchen is composed.
        """
        return (math.pi / 2, 0.0, math.pi, -math.pi / 2)

    @property
    def headings_to_try(self) -> tuple[float, ...]:
        """
        :return: Which ways Tiago is turned while a stand is looked for.
        """
        return self.quarter_turns

    heading: float = field(init=False, default=math.pi / 2)
    """
    Which way Tiago faces where it now stands.
    """
    works_with_the_left_arm: bool = False
    """
    Which of Tiago's two arms does the work.

    The other one is parked at its side, and in a kitchen that leaves it pointing at
    whatever the robot's other shoulder faces, so the arm that works is the one that
    keeps the idle arm out of the wall.
    """

    @property
    def arm(self):
        """
        :return: The arm Tiago works with.
        """
        torso = self.robot.mobile_base.torso
        return torso.left_arm if self.works_with_the_left_arm else torso.right_arm

    def extent(self, body: Body) -> VolumetricBoundingBox:
        """
        :param body: A body of the kitchen.
        :return: Its bounding box in the kitchen's frame.
        """
        return body.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_box()

    def box_around(self, bodies: list[Body]) -> tuple[np.ndarray, np.ndarray]:
        """
        :param bodies: Bodies of the kitchen.
        :return: The lowest and highest corner of the box around all of their pieces,
            in the kitchen's frame.
        """
        pieces = [
            piece
            for body in bodies
            for piece in body.collision.as_bounding_box_collection_in_frame(
                self.world.root
            ).bounding_boxes
        ]
        return self.corners_around(pieces)

    @staticmethod
    def corners_around(pieces: list) -> tuple[np.ndarray, np.ndarray]:
        """
        :param pieces: Bounding boxes in one frame.
        :return: The lowest and highest corner of the box around all of them.
        """
        lowest = np.array([[p.min_x, p.min_y, p.min_z] for p in pieces]).min(axis=0)
        highest = np.array([[p.max_x, p.max_y, p.max_z] for p in pieces]).max(axis=0)
        return lowest, highest

    def colliding_bodies_below(self, root: Body) -> list[Body]:
        """
        :param root: A body of the kitchen.
        :return: It and everything under it that has collision geometry.
        """
        return [
            body
            for body in self.world.compute_descendent_child_kinematic_structure_entities(
                root
            )
            + [root]
            if isinstance(body, Body) and body.has_collision()
        ]

    def build_world(self) -> World:
        """
        Compose the RoboCasa task scene, at RoboCasa's own scale unless the task asks
        for a counter height of its own, and stand Tiago in front of
        :attr:`first_target`.

        :return: The world.
        """
        layout = self.layout if self.layout is not None else LayoutType.LAYOUT001
        style = self.style if self.style is not None else StyleType.STYLE001
        self.task = RoboCasaDatasetLoader().load_task(
            self.task_name,
            layout,
            style,
            seed=self.seed,
            task_parameters=self.task_parameters,
            fixture_updates=self.fixture_updates(layout),
        )
        self.world = self.task.world
        self.prepare_for_the_kitchen()
        robot = RobotSpecification(TiagoMujoco).spawn(self.world)
        robot.mobile_base.full_body_controlled = False
        with self.world.modify_world():
            self.world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=robot.bodies_with_collision,
                    body_group_b=self.floors,
                )
            )
            self.let_the_arms_work_among(self.handled_bodies)
        self.raise_the_torso()
        self.park_the_arms()
        self.stand_where_nothing_touches()
        return self.world

    def prepare_for_the_kitchen(self) -> None:
        """
        Set up what the task needs of the kitchen once it is composed, before Tiago is
        put into it. Nothing, unless the task needs something.
        """

    def work_in(self, kitchen: TiagoRoboCasaTask) -> None:
        """
        Take on the kitchen another task has composed, to be carried out in it as one
        step of that task.

        :param kitchen: The task that composed the kitchen.
        """
        self.task = kitchen.task
        self.world = kitchen.world

    @property
    def floors(self) -> list[Body]:
        """
        :return: The floor and the slab under it, which Tiago's base stands on and so
            never keeps clear of. RoboCasa names them after the room they lie in, which
            differs from layout to layout.
        """
        return [
            body
            for body in self.world.bodies
            if body.name.name.startswith(KitchenBodyPrefix.FLOOR)
        ]

    @property
    def handled_bodies(self) -> list[Body]:
        """
        :return: The bodies Tiago's arms work right among, which they may touch as the
            hand does. Nothing, unless the task names them.
        """
        return []

    def let_the_arms_work_among(self, handled: list[Body]) -> None:
        """
        Allow both of Tiago's arms to touch what it works among.

        Whole arms rather than their wrists alone: an arm that holds a handle while the
        fixture slides or swings towards it has every link of its last stretch against
        the moving part. Both arms rather than the working one alone: working a fixture
        brings Tiago up against it, closer than the clearance the idle arm at its other
        side is otherwise held to.

        :param handled: What the arms work among.
        """
        if not handled:
            return
        self.world.collision_manager.add_ignore_collision_rule(
            AllowCollisionBetweenGroups(
                body_group_a=[
                    body
                    for arm in self.robot.all_arms
                    for body in arm.bodies
                    if body.has_collision()
                ],
                body_group_b=handled,
            )
        )

    kitchen_range: float = 8.0
    """
    How wide a square around the kitchen's origin the base's way is planned in, in
    metres: wide enough for the whole room, since a task may send Tiago from one wall
    to another.
    """
    base_clearance: float = 0.05
    """
    How far the planned way keeps the base off the kitchen, in metres.
    """
    backing_range: float = 1.0
    """
    How far Tiago backs out of a stand at most to reach the free space the way is
    planned through, in metres.
    """
    backing_step: float = 0.05
    """
    How far apart the points tried along the way out of a stand lie, in metres.
    """

    @property
    def footprint_radius(self) -> float:
        """
        :return: How far any part of Tiago reaches out over the floor from the centre of
            its base, in metres, with its arms as they stand.
        """
        vertices = np.concatenate(
            [
                shape.mesh_in_frame(self.robot.root).vertices
                for body in self.robot.bodies_with_collision
                for shape in body.collision.shapes
            ]
        )
        return float(np.max(np.linalg.norm(vertices[:, :2], axis=1)))

    floor_clearance: float = 0.05
    """
    How far above the floor the base's way is planned, in metres: the floor is a body of
    the kitchen like any other, and a way planned from the ground up has the robot
    standing in it everywhere.
    """

    def floor_plan(self) -> PlanarGraphOfBoundingBoxes:
        """
        :return: The kitchen's free space for the base's centre: what is in the way
            between just above the floor and the top of Tiago, so that the floor
            underneath does not count as an obstacle, grown by how far Tiago reaches out
            from its base and the base's clearance.
        """
        _, top = self.box_around(self.robot.bodies_with_collision)
        search_space = BoundingBoxCollection.from_simple_event(
            VolumetricBoundingBox,
            reference_frame=self.world.root,
            simple_event=SimpleEvent.from_data(
                {
                    SpatialVariables.x.value: closed(
                        -self.kitchen_range, self.kitchen_range
                    ),
                    SpatialVariables.y.value: closed(
                        -self.kitchen_range, self.kitchen_range
                    ),
                    SpatialVariables.z.value: closed(
                        self.floor_clearance, float(top[2])
                    ),
                }
            ),
        )
        return PlanarGraphOfBoundingBoxes.navigation_map_from_world(
            world=self.world,
            search_space=search_space,
            bloat_obstacles=self.footprint_radius + self.base_clearance,
        )

    def drive_to(self, stand: Pose) -> list[NavigateAction]:
        """
        Drive the base to a stand along a way planned through the kitchen's free space,
        rather than straight at it: a kitchen has an island and a table between its
        counters, and a straight line walks into them. Tiago backs straight out of where
        it stands into that free space, and drives straight into the stand from it,
        since a stand in front of a fixture lies closer to it than the way keeps the
        base.

        :param stand: Where Tiago is to end up.
        :return: Driving there, one leg per corner of the planned way.
        :raises NoWayToTheStand: If the free space holds no way there.
        """
        navigation = self.floor_plan()
        here = self.stands_at
        way_out = self.way_into_the_free_space(navigation, here)
        way_in = self.way_into_the_free_space(navigation, stand)
        if way_out is None or way_in is None:
            raise NoWayToTheStand(stand=stand)
        corners = navigation.path_from_to(way_out, way_in)
        if corners is None:
            raise NoWayToTheStand(stand=stand)
        backing_out = Pose.from_xyz_rpy(
            x=float(way_out.to_np()[0]),
            y=float(way_out.to_np()[1]),
            yaw=self.heading_of(here),
            reference_frame=self.world.root,
        )
        stand_position = Point2(
            stand.position.x, stand.position.y, reference_frame=self.world.root
        )
        legs = self.legs_along(corners + [stand_position], stand)
        if not np.allclose(way_out.to_np()[:2], here.to_np()[:2, 3]):
            legs = [backing_out] + legs
        return [NavigateAction(leg) for leg in legs]

    def heading_of(self, pose: Pose) -> float:
        """
        :param pose: A pose of the base in the kitchen's frame.
        :return: The way it faces over the floor, in radians.
        """
        rotation = pose.to_np()[:3, :3]
        return float(np.arctan2(rotation[1, 0], rotation[0, 0]))

    def way_into_the_free_space(
        self, navigation: PlanarGraphOfBoundingBoxes, stand: Pose
    ) -> Point2 | None:
        """
        :param navigation: The kitchen's free space for the base's centre.
        :param stand: A pose of the base.
        :return: The nearest point straight behind the stand, the stand's own place
            included, that lies in the free space, or nothing within
            :attr:`backing_range`.
        """
        heading = self.heading_of(stand)
        backwards = -np.array([np.cos(heading), np.sin(heading)])
        for distance in np.arange(0.0, self.backing_range, self.backing_step):
            x, y = stand.to_np()[:2, 3] + distance * backwards
            point = Point2(float(x), float(y), reference_frame=self.world.root)
            if any(node.contains(point) for node in navigation.graph.nodes()):
                return point
        return None

    def legs_along(self, corners: list[Point2], stand: Pose) -> list[Pose]:
        """
        :param corners: The corners of the planned way, from where Tiago stands to
            where it is going.
        :param stand: Where it is going, which carries the way it is to face there.
        :return: Two poses per leg of the way, turning on the spot at its start to face
            the way it runs and then driving along it, and ``stand`` itself last. A base
            that turns while it drives sweeps an arc off the planned way.
        """
        positions = [corner.to_np()[:2].flatten() for corner in corners]
        legs = []
        for start, end in zip(positions, positions[1:]):
            step = end - start
            if np.allclose(step, 0.0):
                continue
            yaw = float(np.arctan2(step[1], step[0]))
            legs += [
                Pose.from_xyz_rpy(
                    x=float(place[0]),
                    y=float(place[1]),
                    yaw=yaw,
                    reference_frame=self.world.root,
                )
                for place in (start, end)
            ]
        return legs + [stand]

    @property
    def stands_at(self) -> Pose:
        """
        :return: Where Tiago stands at the moment.
        """
        return self.robot.root.parent_connection.origin.pose

    def stand_that_reaches(
        self,
        arrives: Callable[[], bool],
        target: np.ndarray | None = None,
        reach: float | None = None,
        headings: tuple[float, ...] | None = None,
    ) -> StandSearch:
        """
        Move Tiago among the clear stands until the arm can do what the task needs, and
        leave it standing there.

        Standing clear of the kitchen is not enough: the stand also has to leave what the
        arm is for within its working envelope, which depends on where that is and so
        cannot be read off the stand alone. Each candidate is tried in a kinematic world,
        which costs nothing and leaves the scene as it was.

        :param arrives: Whether the arm gets where the task needs it from where Tiago
            now stands, asked afresh at each stand.
        :param target: What the stand is worked out from, or nothing for the task's own
            first target.
        :param reach: How far in front of the base that target is to be, or nothing for
            the task's own.
        :param headings: Which ways to try facing, or nothing for the task's own.
        :return: What the search found.
        """
        target = self.first_target if target is None else target
        reach = self.reach if reach is None else reach
        self.world.collision_manager.update_collision_matrix()
        drive = self.robot.root.parent_connection
        stands = self.stands_to_try(headings)
        clear = 0
        for candidate in stands:
            self.heading = candidate.heading
            stand = self.stand_of(candidate, target, reach)
            drive.origin = stand.homogeneous_matrix
            if not self.keeps_clear_of_the_kitchen():
                continue
            clear += 1
            if arrives():
                return StandSearch(
                    target=target, stand=stand, tried=len(stands), clear=clear
                )
        return StandSearch(target=target, stand=None, tried=len(stands), clear=clear)

    def stand_where_the_hand_arrives(
        self,
        arrives: Callable[[], bool],
        target: np.ndarray | None = None,
        reach: float | None = None,
    ) -> Pose:
        """
        :return: The stand :meth:`stand_that_reaches` found, which Tiago is left at.
        :raises NoClearStandingPose: If Tiago touches the kitchen at every stand.
        :raises NoReachableStandingPose: If it stands clear somewhere but the arm
            arrives from none of those stands.
        """
        search = self.stand_that_reaches(arrives, target, reach)
        if search.stand is not None:
            return search.stand
        if search.clear == 0:
            raise NoClearStandingPose(target=search.target, tried=search.tried)
        raise NoReachableStandingPose(
            target=search.target, tried=search.tried, clear=search.clear
        )

    def raise_the_torso(self) -> None:
        """
        Stand the torso at its full height before the simulation is built.

        The torso is set rather than commanded: its joint carries the whole upper body,
        and a servo asked to lift that saturates its force without moving, so the robot
        starts where it needs to be. With the torso down, Tiago's hand does not reach a
        RoboCasa counter at all.
        """
        self.robot.mobile_base.torso.get_joint_state_by_type(TorsoState.HIGH).apply_to(
            self.world
        )

    def keeps_clear_of_the_kitchen(self) -> bool:
        """
        :return: Whether Tiago stands clear of the kitchen: its base by the whole
            distance the collision avoidance holds it to, the rest of it by not
            violating that distance.

        The base is held to the wider margin because it is what drifts: it drives to
        the stand open loop, and a wheel two millimetres inside a cabinet door ends a
        task whose object is already in hand. What the arm does on the way is the
        avoidance's business while the plan runs.
        """
        collision_manager = self.world.collision_manager
        rolls = set(self.robot.mobile_base.bodies_with_collision)
        robot_bodies = set(self.robot.bodies_with_collision)
        for contact in collision_manager.compute_collisions().contacts:
            touches_the_robot = (contact.body_a in robot_bodies) != (
                contact.body_b in robot_bodies
            )
            if not touches_the_robot:
                continue
            if contact.body_a in rolls or contact.body_b in rolls:
                needed = collision_manager.get_buffer_zone_distance(
                    contact.body_a, contact.body_b
                )
            else:
                needed = collision_manager.get_violated_distance(
                    contact.body_a, contact.body_b
                )
            if contact.distance <= needed:
                return False
        return True

    def park_the_arms(self) -> None:
        """
        Fold both arms at Tiago's sides before anything is worked out from where its
        bodies are.

        Tiago is spawned with its arms straight out, which reaches nearly a metre to
        either side: a stand looked for with the arms like that has the robot in the
        wall or in the counter almost wherever it is put, and the plan parks the arms
        before it does anything anyway.
        """
        for arm in self.robot.all_arms:
            arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(self.world)

    def stand_where_nothing_touches(self) -> None:
        """
        Put Tiago where it reaches :attr:`first_target`, stepping it back and along the
        counter until it touches nothing.

        Tiago is wider than the robot a RoboCasa kitchen is laid out around and carries
        a second arm at its side, so the stand that puts its working hand in front of
        the target often leaves a wheel in a cabinet or an elbow in a wall.

        :raises NoClearStandingPose: If it touches the kitchen at every stand tried.
        """
        self.world.collision_manager.update_collision_matrix()
        drive = self.robot.root.parent_connection
        stands = self.stands_to_try()
        for candidate in stands:
            self.heading = candidate.heading
            drive.origin = self.stand_of(
                candidate, self.first_target, self.reach
            ).homogeneous_matrix
            if self.keeps_clear_of_the_kitchen():
                return
        raise NoClearStandingPose(target=self.first_target, tried=len(stands))

    def stands_to_try(
        self, headings: tuple[float, ...] | None = None
    ) -> list[StandCandidate]:
        """
        :param headings: Which ways to try facing, or nothing for the task's own.
        :return: The stands to try, nearest and facing the way the kitchen is usually
            worked from first.
        """
        steps = np.arange(0.0, self.stand_search + self.stand_step / 2, self.stand_step)
        slides = [0.0] + [side * step for step in steps[1:] for side in (1.0, -1.0)]
        return [
            StandCandidate(back=float(back), along=float(along), heading=heading)
            for heading in (self.headings_to_try if headings is None else headings)
            for back in steps
            for along in slides
        ]

    def stand_of(
        self, candidate: StandCandidate, target: np.ndarray, reach: float
    ) -> Pose:
        """
        :param candidate: The stand to work out.
        :param target: What it is to reach.
        :param reach: How far in front of the base the target is to be.
        :return: Where Tiago stands.
        """
        left = np.array(
            [-math.sin(candidate.heading), math.cos(candidate.heading), 0.0]
        )
        return self.standing_pose(
            target + candidate.along * left, reach + candidate.back, candidate.heading
        )

    def fixture_updates(self, layout: Any) -> dict[str, Any]:
        """
        :param layout: The kitchen layout being composed.
        :return: What to change about the kitchen's fixtures, which is nothing unless
            the counters are to stand at a height of their own.
        """
        if self.counter_height is None:
            return {}
        return RoboCasaLayout.from_layout_type(layout).lowered_fixtures(
            self.counter_height
        )

    def standing_pose(
        self, target: np.ndarray, reach: float, heading: float = math.pi / 2
    ) -> Pose:
        """
        :param target: Where in the kitchen the hand has to reach.
        :param reach: How far in front of the base's centre ``target`` is to be.
        :param heading: The direction Tiago faces, as a yaw in the kitchen's frame.
        :return: Where Tiago stands to reach it, with the arm it works with rather than
            its base in line with the target.
        """
        facing = np.array([math.cos(heading), math.sin(heading)])
        left = np.array([-math.sin(heading), math.cos(heading)])
        x, y = target[:2] - reach * facing - self.shoulder_offset * left
        return Pose.from_xyz_rpy(
            x=float(x), y=float(y), yaw=heading, reference_frame=self.world.root
        )

    @property
    def shoulder_offset(self) -> float:
        """
        :return: How far the working arm's shoulder sits to the robot's left of its
            base's centre, in metres, which is negative for the right arm.
        """
        base = self.robot.root.global_transform.to_np()
        shoulder = self.arm.bodies[1].global_transform.to_np()
        return float((np.linalg.inv(base) @ shoulder)[1, 3])

    def can_reach(self, tool_frame_goal: Pose) -> bool:
        """
        :param tool_frame_goal: Where the tool frame is to be.
        :return: Whether the arm gets there in a kinematically simulated world.
        """
        try:
            self.arm_motions_to(tool_frame_goal)
        except PlanFailure:
            return False
        return True

    def arm_motions_to(self, tool_frame_goal: Pose) -> list[MoveJointsMotion]:
        """
        Bring the arm to a tool frame goal in joint space.

        Where the arm is to go is worked out in a kinematically simulated world, which
        the goal is reachable in, and the joints it ends up in are what the physical run
        is asked for. A tool frame goal given to the physical run directly stalls: the
        arm is still on its way when the controller is satisfied, and the hand ends up
        short of what it is reaching for.

        :param tool_frame_goal: Where the tool frame is to be.
        :return: The motion bringing the arm there.
        """
        context = Context(world=self.world, robot=self.robot, evaluate_conditions=False)
        joints = [
            connection
            for connection in self.arm.active_connections
            if isinstance(connection, ActiveConnection1DOF)
        ]
        with self.world.reset_state_context():
            self.straighten_the_arm()
            with ExecutionEnvironment(
                ExecutionType.KINEMATICALLY_SIMULATED,
                collision_avoidance=self.collision_avoidance,
            ):
                sequential(
                    [
                        MoveToolCenterPointMotion(
                            tool_frame_goal, self.arm, allow_gripper_collision=True
                        )
                    ],
                    context=context,
                ).plan.perform()
            positions = [float(joint.position) for joint in joints]
        return [
            MoveJointsMotion(
                names=[joint.name.name for joint in joints], positions=positions
            )
        ]

    def straighten_the_arm(self) -> None:
        """
        Put the working arm straight out before a goal is solved for.

        The solver walks the arm from wherever it stands, and out of the folded park
        pose it gets nowhere: of the stands that leave an object on the counter within
        reach, it finds none, while from the arm straight out it finds them. What the
        fold costs is the solve, not the motion -- the physical run starts from the park
        pose and gets there.
        """
        for connection in self.arm.active_connections:
            if isinstance(connection, ActiveConnection1DOF):
                connection.position = 0.0

    def prepare_contacts(self) -> None:
        """
        Set up how the task's objects make contact in MuJoCo, before the simulation is
        built.

        Leaves them as RoboCasa gives them.
        """

    camera_offset: float = 35.0
    """
    How far the recording camera is turned from looking the way Tiago faces, in
    degrees, so that it looks over Tiago's shoulder rather than at its back.
    """

    def camera_on(self, target: np.ndarray, distance: float = 3.5) -> ViewerCamera:
        """
        :param target: What the recording looks at.
        :param distance: How far the camera stands from it, in metres.
        :return: A camera looking down at that point from behind Tiago and to its
            side, from above the kitchen's walls, which leaves a narrow kitchen's far
            wall behind it rather than in front of it.
        """
        return ViewerCamera(
            look_at=(float(target[0]), float(target[1]), float(target[2])),
            distance=distance,
            azimuth=math.degrees(self.heading) + self.camera_offset,
            elevation=-50.0,
        )

    @property
    def stages(self) -> list[Callable[[Context], Plan]]:
        """
        :return: What builds the plans the task is carried out in, one after the other,
            each from where the one before left the scene. A task that is one plan from
            start to finish leaves this as it is.
        """
        return [self.build_plan]

    def planned_with_the_arm_alone(
        self, build: Callable[[Context], Plan], context: Context
    ) -> Plan:
        """
        Work out one stage's plan with the base held where the task stood it, and free
        the base to come along afterwards if the task has it.

        Solving the arm moves whatever the controller is given, so a base freed while
        the plan is worked out ends up in the joint positions the plan is made of.

        :param build: What builds the plan.
        :param context: The context to plan in.
        :return: The plan.
        """
        self.robot.mobile_base.full_body_controlled = False
        plan = build(context)
        self.robot.mobile_base.full_body_controlled = self.base_follows_the_arm
        return plan

    def run(self) -> None:
        """
        Let the scene settle, then carry the task's stages out against MuJoCo physics
        one after the other, recording them if a recording was asked for.
        """
        context = Context(
            world=self.world,
            robot=self.robot,
            evaluate_conditions=False,
            motion_tolerances=MotionToleranceConfig(
                default_tcp_position_threshold=self.tool_frame_tolerance
            ),
        )
        self.prepare_contacts()
        simulation = MujocoSim(
            world=self.world,
            headless=self.headless,
            step_size=self.step_size,
            cone=self.friction_cone,
            impratio=self.impedance_ratio,
            noslip_iterations=self.noslip_iterations,
            solver=self.solver,
            integrator=self.integrator,
        )
        simulation.start_stepped_simulation()
        try:
            simulation.step_simulation(self.settling_time)
            with self._recorder(simulation):
                for build in self.stages:
                    plan = self.planned_with_the_arm_alone(build, context)
                    with PhysicallySimulatedRobot(
                        simulation=simulation,
                        collision_avoidance=self.collision_avoidance,
                    ):
                        plan.perform()
                simulation.step_simulation(self.resting_time)
        finally:
            simulation.stop_simulation()

    def _recorder(self, simulation: MujocoSim):
        """
        :param simulation: The running simulation frames are taken from.
        :return: What writes the video, or a context that writes nothing.
        """
        if self.recording is None:
            return nullcontext()
        return SceneRecorder(
            simulation=simulation,
            camera=self.recording_camera,
            recording=self.recording,
        )

    @property
    def recording_camera(self) -> ViewerCamera:
        """
        :return: What the recording is taken through: a camera on
            :attr:`first_target`.
        """
        return self.camera_on(self.first_target)
