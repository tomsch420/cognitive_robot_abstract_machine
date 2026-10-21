"""
What the Toyota HSR's RoboCasa tasks share: a RoboCasa kitchen lowered to the HSR's
height, the HSR standing in front of what it reaches for and moving its arm there, and
the task carried out against MuJoCo physics once the scene has settled.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import StrEnum
from typing_extensions import Any, Callable

import numpy as np
import yaml
from krrood.ormatic.utils import classproperty
from coraplex.datastructures.dataclasses import Context, MotionToleranceConfig
from coraplex.execution_environment import PhysicallySimulatedRobot
from coraplex.plans.plan import Plan
from coraplex.robot_plans.motions.robot_body import MoveJointsMotion
from segmind.datastructures.events import DetectionEvent
from segmind.event_segmentation import Segmind
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
    RoboCasaTask,
)
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB, HSRBJoint
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

from experiments.hsr_kitchen_mujoco.demo import (
    FRICTION_CONE,
    IMPEDANCE_RATIO,
    NO_SLIP_ITERATIONS,
    SETTLING_DURATION,
    STEP_SIZE,
    hand_offset,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import (
        LayoutType,
        StyleType,
        get_layout_path,
    )

# %% the scene


class KitchenBody(StrEnum):
    """
    The bodies of the RoboCasa kitchen the experiment refers to, by their names.
    """

    FLOOR = "floor_room_main"
    """
    The floor, which the HSR's base stands on and so never keeps clear of.
    """

    FLOOR_BACKING = "floor_backing_room_main"
    """
    The slab under the floor, which the HSR's base comes as close to as to the floor.
    """


class ArmLink(StrEnum):
    """
    The links of the HSR's arm that a task may bring as close to what it handles as
    the hand itself, by the names the HSR's description gives them.
    """

    ARM_ROLL = "arm_roll_link"
    """
    The link the arm rolls, which carries the wrist.
    """

    WRIST_FLEX = "wrist_flex_link"
    """
    The link the wrist flexes.
    """

    WRIST_ROLL = "wrist_roll_link"
    """
    The link the wrist rolls, which carries the hand.
    """


@dataclass
class HSRBArmPlane:
    """
    The vertical plane the HSR's arm swings its hand in while its arm and wrist roll
    joints stand at zero, and the joint positions that put the hand at a pose in it.
    In that plane, the hand's tilt depends on the sum of arm and wrist flex alone, and
    the wrist's place on arm flex and lift alone.
    """

    search_steps: int = 600
    """
    How many positions each of the two searches tries within a joint's limits.
    """

    world: World = field(init=False)
    """
    A world holding nothing but an HSR, whose joints the searches set, so that no
    simulated robot moves.
    """

    robot: HSRB = field(init=False)
    """
    The HSR of :attr:`world`.
    """

    def __post_init__(self):
        self.world = World()
        with self.world.modify_world():
            self.world.add_kinematic_structure_entity(Body(name=PrefixedName("floor")))
        self.robot = RobotSpecification(HSRB).spawn(self.world)

    def joint_positions_for(self, base_T_tool: np.ndarray) -> dict[HSRBJoint, float]:
        """
        :param base_T_tool: Where the tool frame is to be, relative to the HSR's root;
            it has to lie in the arm's plane.
        :return: The positions of the arm's joints that put the tool frame there, as
            near as the searches get, the wrist rolled to the goal's turn about the
            hand's approach.
        """
        end_effector = self.robot.all_arms[0].end_effector
        wanted_approach = base_T_tool[:3, :3] @ end_effector.approach_axis.to_np()[:3]
        self._set({HSRBJoint.ARM_ROLL: 0.0, HSRBJoint.WRIST_ROLL: 0.0})
        tilt = max(
            self._tilts(),
            key=lambda candidate: self._approach_at(candidate) @ wanted_approach,
        )
        self._set_tilt(tilt)
        wrist_T_tool = self._base_T(end_effector.tool_frame)[:3, 3] - self._wrist()
        wanted_wrist = base_T_tool[:3, 3] - wrist_T_tool
        lift_limits = self._limits(HSRBJoint.ARM_LIFT)
        wrist_limits = self._limits(HSRBJoint.WRIST_FLEX)
        candidates = []
        for flex in np.linspace(*self._limits(HSRBJoint.ARM_FLEX), self.search_steps):
            if not wrist_limits[0] <= tilt - flex <= wrist_limits[1]:
                continue
            self._set({HSRBJoint.ARM_LIFT: 0.0, HSRBJoint.ARM_FLEX: flex})
            wrist = self._wrist()
            lift = wanted_wrist[2] - wrist[2]
            if lift_limits[0] <= lift <= lift_limits[1]:
                candidates.append((abs(wanted_wrist[0] - wrist[0]), flex, lift))
        _, flex, lift = min(candidates)
        positions = {
            HSRBJoint.ARM_LIFT: float(lift),
            HSRBJoint.ARM_FLEX: float(flex),
            HSRBJoint.ARM_ROLL: 0.0,
            HSRBJoint.WRIST_FLEX: float(tilt - flex),
            HSRBJoint.WRIST_ROLL: 0.0,
        }
        positions[HSRBJoint.WRIST_ROLL] = self._wrist_roll_for(positions, base_T_tool)
        return positions

    def _wrist_roll_for(
        self, positions: dict[HSRBJoint, float], base_T_tool: np.ndarray
    ) -> float:
        """
        :param positions: The arm's joint positions, the wrist not rolled.
        :param base_T_tool: Where the tool frame is to be, relative to the HSR's root.
        :return: The wrist roll that turns the tool frame about the hand's approach as
            far as ``base_T_tool`` is turned.
        """
        tool_frame = self.robot.all_arms[0].end_effector.tool_frame
        self._set(positions)
        unrolled = self._base_T(tool_frame)[:3, :3]
        turn = unrolled.T @ base_T_tool[:3, :3]
        angle = math.atan2(turn[1, 0], turn[0, 0])

        def orientation_error(roll: float) -> float:
            self._set({HSRBJoint.WRIST_ROLL: roll})
            return np.linalg.norm(
                self._base_T(tool_frame)[:3, :3] - base_T_tool[:3, :3]
            )

        return min([angle, -angle], key=orientation_error)

    def _tilts(self) -> np.ndarray:
        """
        :return: The sums of arm and wrist flex the joints' limits allow.
        """
        flex = self._limits(HSRBJoint.ARM_FLEX)
        wrist = self._limits(HSRBJoint.WRIST_FLEX)
        return np.linspace(flex[0] + wrist[0], flex[1] + wrist[1], self.search_steps)

    def _set_tilt(self, tilt: float) -> None:
        """
        Tilt the hand by ``tilt``, the sum of arm and wrist flex, with the arm flexed as
        little as the wrist's limits allow.
        """
        flex = float(np.clip(tilt, *self._limits(HSRBJoint.ARM_FLEX)))
        self._set({HSRBJoint.ARM_FLEX: flex, HSRBJoint.WRIST_FLEX: tilt - flex})

    def _approach_at(self, tilt: float) -> np.ndarray:
        """
        :return: The direction the hand approaches along when tilted by ``tilt``,
            relative to the HSR's root.
        """
        self._set_tilt(tilt)
        end_effector = self.robot.all_arms[0].end_effector
        return (
            self._base_T(end_effector.tool_frame)[:3, :3]
            @ end_effector.approach_axis.to_np()[:3]
        )

    def _wrist(self) -> np.ndarray:
        """
        :return: Where the wrist flex joint is, relative to the HSR's root.
        """
        wrist = self.world.get_connection_by_name(HSRBJoint.WRIST_FLEX).child
        return self._base_T(wrist)[:3, 3]

    def _base_T(self, entity: KinematicStructureEntity) -> np.ndarray:
        """
        :return: The pose of ``entity`` relative to the HSR's root.
        """
        return self.world.compute_forward_kinematics_np(self.robot.root, entity)

    def _limits(self, joint: HSRBJoint) -> tuple[float, float]:
        """
        :return: The lower and upper position limit of ``joint``.
        """
        limits = self.world.get_connection_by_name(joint).dof.limits
        return limits.lower.position, limits.upper.position

    def _set(self, positions: dict[HSRBJoint, float]) -> None:
        """
        Set the joints of :attr:`world`'s HSR to ``positions``.
        """
        for joint, position in positions.items():
            self.world.get_connection_by_name(joint).position = position


class FixtureReference(StrEnum):
    """
    The names RoboCasa's tasks refer to their fixtures by.
    """

    DRAWER = "drawer"
    """
    The drawer a drawer task works.
    """

    COUNTER = "counter"
    """
    The counter above the drawer.
    """

    FIXTURE = "fxtr"
    """
    The fixture a door task works.
    """


class LayoutKey(StrEnum):
    """
    The keys of a RoboCasa layout this experiment reads or changes.
    """

    SIZE = "size"
    """
    A fixture's width, depth and height.
    """

    POSITION = "pos"
    """
    Where a fixture stands, along the wall, out from it, and up.
    """

    STACK_HEIGHT = "stack_height"
    """
    How high the fixtures below a stacked fixture reach.
    """

    MAIN_GROUP = "main_group"
    """
    The fixtures along the kitchen's main wall.
    """

    BOTTOM_ROW = "bottom_row"
    """
    The counters and appliances standing on the floor along the main wall.
    """

    TYPE = "type"
    """
    What kind of fixture an entry is.
    """

    NAME = "name"
    """
    The name of a fixture.
    """

    ISLAND_GROUP = "island_group"
    """
    The fixtures of a kitchen island standing free in the room.
    """


class FixtureType(StrEnum):
    """
    The kinds of fixture this experiment looks for in a RoboCasa layout.
    """

    COUNTER = "counter"
    """
    A counter.
    """


class RowOffTheFloor(StrEnum):
    """
    The rows of a RoboCasa layout whose fixtures hang on a wall or make up the room,
    rather than stand on the floor.
    """

    WALLS = "walls"
    """
    The walls themselves.
    """

    FLOOR = "floor"
    """
    The floor itself.
    """

    WALL_ACCESSORIES = "wall_accessories"
    """
    Outlets, switches and the like on the walls.
    """

    TOP_ROW = "top_row"
    """
    The wall cabinets and the appliances hanging between them.
    """

    WALL_STACK = "wall_stack"
    """
    Fixtures stacked on a wall.
    """

    WALL_STACKS = "wall_stacks"
    """
    Fixtures stacked on a wall, as some layouts spell it.
    """


@dataclass
class RoboCasaLayout:
    """
    A RoboCasa kitchen layout, as RoboCasa's layout files describe it.
    """

    configuration: dict[str, Any]
    """
    The layout, as read from its file.
    """

    @classmethod
    def from_layout_type(cls, layout: LayoutType) -> RoboCasaLayout:
        """
        :param layout: One of RoboCasa's layouts.
        :return: That layout, read from RoboCasa's file of it.
        """
        with open(get_layout_path(layout)) as layout_file:
            return cls(yaml.safe_load(layout_file))

    @property
    def counter_height(self) -> float:
        """
        How high the counters along the main wall stand, in metres.
        """
        return next(
            fixture[LayoutKey.SIZE][2]
            for fixture in self.configuration[LayoutKey.MAIN_GROUP][
                LayoutKey.BOTTOM_ROW
            ]
            if fixture[LayoutKey.TYPE] == FixtureType.COUNTER
        )

    def lowered_fixtures(self, counter_height: float) -> dict[str, dict[str, Any]]:
        """
        :param counter_height: How high the counters are to stand, in metres.
        :return: The updates, by fixture name, that scale every height of the fixtures
            standing on the floor by the same factor, so that the counters stand
            ``counter_height`` high; what hangs on the walls keeps its height, and
            where a group stands is left as it is.
        """
        factor = counter_height / self.counter_height
        return {
            fixture[LayoutKey.NAME]: update
            for group in self.configuration.values()
            for row_name, row in group.items()
            if row_name not in RowOffTheFloor and isinstance(row, list)
            for fixture in row
            if isinstance(fixture, dict) and (update := self._lowered(fixture, factor))
        }

    @staticmethod
    def _lowered(fixture: dict[str, Any], factor: float) -> dict[str, Any]:
        """
        :param fixture: A fixture of the layout.
        :param factor: What to multiply each of its heights with.
        :return: The fixture's size, position and stack height with their heights
            scaled, for those it states.
        """
        update = {}
        for key in (LayoutKey.SIZE, LayoutKey.POSITION):
            value = fixture.get(key)
            if (
                isinstance(value, list)
                and len(value) == 3
                and isinstance(value[2], (int, float))
            ):
                update[key] = [value[0], value[1], value[2] * factor]
        if LayoutKey.STACK_HEIGHT in fixture:
            update[LayoutKey.STACK_HEIGHT] = fixture[LayoutKey.STACK_HEIGHT] * factor
        return update


@dataclass
class HSRRoboCasaTask(ABC):
    """
    One of RoboCasa's tasks, performed by the HSR. Each task is a subclass named after
    the RoboCasa task it performs.
    """

    layout: LayoutType = LayoutType.LAYOUT001
    """
    The RoboCasa kitchen layout.
    """

    style: StyleType = StyleType.STYLE001
    """
    The RoboCasa kitchen style.
    """

    seed: int = 0
    """
    The seed RoboCasa samples the task's objects and their places with.
    """

    counter_height: float = 0.75
    """
    How high the kitchen's counters stand, in metres, lowered from RoboCasa's 0.92 m
    together with everything else standing on the floor.
    """

    reach: float = 0.6
    """
    How far in front of its base's centre the HSR takes hold of what it reaches for, in
    metres, so that both the grasp and the pre-grasp before it lie within the 0.43 m to
    0.64 m its arm reaches with the hand tilted down at :attr:`grasp_pitch`.
    """

    grasp_pitch: float = math.pi / 4
    """
    How steeply the hand comes down onto what it takes hold of, in radians below the
    horizontal.
    """

    tool_frame_tolerance: float = 0.015
    """
    How far, in metres, the tool frame may stop from a goal of the task's motions: the
    base stops within about a centimetre of where it is sent, and the arm, swinging in
    its plane, cannot make up for that sideways.
    """

    collision_avoidance: bool = True
    """
    Whether Giskard keeps the HSR clear of the kitchen and of itself.
    """

    headless: bool = False
    """
    Whether the simulation runs without MuJoCo's own viewer.
    """

    task: RoboCasaTask = field(init=False)
    """
    The task as RoboCasa composed it, built by :meth:`build_world`.
    """

    world: World = field(init=False)
    """
    The kitchen with the task's objects and the HSR, built by :meth:`build_world`.
    """

    detected_events: list[DetectionEvent] = field(init=False, default_factory=list)
    """
    What SegMind detected during the last run, in order.
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
        The arguments the RoboCasa task is composed with, by name.
        """
        return {}

    @property
    @abstractmethod
    def handled_bodies(self) -> list[Body]:
        """
        The bodies the HSR's hand works next to, which its wrist may touch as the hand
        may.
        """

    @property
    @abstractmethod
    def first_target(self) -> np.ndarray:
        """
        Where in the kitchen the hand first reaches, which the HSR is put in front of.
        """

    @abstractmethod
    def build_plan(self, context: Context) -> Plan:
        """
        :param context: The context the actions are built in.
        :return: The plan that performs the task, or its last stage.
        """

    @abstractmethod
    def build_segmind(self) -> Segmind:
        """
        :return: SegMind watching what the task's success is read from.
        """

    @property
    def stages(self) -> list[Callable[[Context], Plan]]:
        """
        What builds the plans the task is performed in, one after the other, each from
        where the previous one left the scene: :meth:`build_plan` alone.
        """
        return [self.build_plan]

    def build_world(self) -> World:
        """
        Compose the RoboCasa task scene in the lowered kitchen and put the HSR in front
        of :attr:`first_target`.

        :return: The world.
        """
        self.task = RoboCasaDatasetLoader().load_task(
            self.task_name,
            self.layout,
            self.style,
            seed=self.seed,
            task_parameters=self.task_parameters,
            fixture_updates=RoboCasaLayout.from_layout_type(
                self.layout
            ).lowered_fixtures(self.counter_height),
        )
        self.world = self.task.world
        robot = RobotSpecification(HSRB).spawn(self.world)
        robot.mobile_base.full_body_controlled = False
        with self.world.modify_world():
            self.world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=robot.bodies_with_collision,
                    body_group_b=[
                        self.world.get_body_by_name(floor)
                        for floor in (KitchenBody.FLOOR, KitchenBody.FLOOR_BACKING)
                    ],
                )
            )
            self.world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=self.bodies_touching_handled_bodies,
                    body_group_b=self.handled_bodies,
                )
            )
        self.robot.root.parent_connection.origin = self.standing_pose(
            self.first_target, self.reach
        ).homogeneous_matrix
        return self.world

    @property
    def bodies_touching_handled_bodies(self) -> list[Body]:
        """
        The HSR's bodies that may touch :attr:`handled_bodies`: the wrist's links,
        which taking an object from where it lies and setting it into its receptacle
        brings as close to them as the hand.
        """
        return self.arm_links([ArmLink.WRIST_FLEX, ArmLink.WRIST_ROLL])

    def arm_links(self, links: list[ArmLink]) -> list[Body]:
        """
        :param links: Links of the HSR's arm.
        :return: The bodies of those links.
        """
        return [
            self.world.get_body_in_branch_by_name(self.robot.root, link)
            for link in links
        ]

    @property
    def robot(self) -> HSRB:
        """
        The HSR.
        """
        return self.world.get_semantic_annotations_by_type(HSRB)[0]

    def colliding_bodies_below(self, root: Body) -> list[Body]:
        """
        :return: The bodies of the object rooted in ``root`` that carry its collision
            geometry: the root itself, or the bodies below it RoboCasa puts it in.
        """
        return [
            body
            for body in [root]
            + list(
                self.world.compute_descendent_child_kinematic_structure_entities(root)
            )
            if isinstance(body, Body) and body.has_collision()
        ]

    def extent(self, body: Body) -> VolumetricBoundingBox:
        """
        :return: The axis-aligned box the collision geometry of ``body`` occupies in the
            kitchen's frame.
        """
        return body.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_box()

    def standing_pose(
        self, target: np.ndarray, reach: float, heading: float = math.pi / 2
    ) -> Pose:
        """
        :param target: Where in the kitchen the hand has to reach.
        :param reach: How far in front of the base's centre ``target`` is to be.
        :param heading: The direction the HSR faces, as a yaw in the kitchen's frame;
            by default towards the counter.
        :return: Where the HSR stands to reach it: facing ``heading``, ``reach`` back
            from ``target``, with its hand rather than its base in line with it.
        """
        facing = np.array([math.cos(heading), math.sin(heading)])
        left = np.array([-math.sin(heading), math.cos(heading)])
        x, y = target[:2] - reach * facing - hand_offset(self.world) * left
        return Pose.from_xyz_rpy(
            x=float(x), y=float(y), yaw=heading, reference_frame=self.world.root
        )

    def arm_motions_to(
        self, tool_frame_goal: Pose, standing_pose: Pose
    ) -> list[MoveJointsMotion]:
        """
        :param tool_frame_goal: Where the tool frame is to be.
        :param standing_pose: Where the HSR stands, see :meth:`standing_pose`.
        :return: The motions that bring the arm there in joint space: the arm lift
            first, so that the hand leaves its park pose above the counter rather than
            in front of the cabinets below it, then every joint of the arm.
        """
        world_T_base = self.world.transform(standing_pose, self.world.root).to_np()
        world_T_goal = self.world.transform(tool_frame_goal, self.world.root).to_np()
        joints = HSRBArmPlane().joint_positions_for(
            np.linalg.inv(world_T_base) @ world_T_goal
        )
        return [
            MoveJointsMotion(
                names=[HSRBJoint.ARM_LIFT], positions=[joints[HSRBJoint.ARM_LIFT]]
            ),
            MoveJointsMotion(names=list(joints), positions=list(joints.values())),
        ]

    def prepare_contacts(self) -> None:
        """
        Set up how the task's objects make contact in MuJoCo, before the simulation is
        built. Leaves them as RoboCasa gives them.
        """

    def run(self) -> None:
        """
        Let the scene settle, then build each of the :attr:`stages`' plans from where
        everything came to rest and carry it out against MuJoCo physics, watched by
        :meth:`build_segmind`'s SegMind, whose events are kept in
        :attr:`detected_events`.
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
            step_size=STEP_SIZE,
            cone=FRICTION_CONE,
            impratio=IMPEDANCE_RATIO,
            noslip_iterations=NO_SLIP_ITERATIONS,
        )
        simulation.start_stepped_simulation()
        try:
            simulation.step_simulation(SETTLING_DURATION)
            segmind = self.build_segmind()
            with segmind:
                for build_stage in self.stages:
                    plan = build_stage(context)
                    with PhysicallySimulatedRobot(
                        simulation=simulation,
                        collision_avoidance=self.collision_avoidance,
                    ):
                        plan.perform()
                simulation.step_simulation(SETTLING_DURATION)
            self.detected_events = segmind.event_logger.get_events()
        finally:
            simulation.stop_simulation()
