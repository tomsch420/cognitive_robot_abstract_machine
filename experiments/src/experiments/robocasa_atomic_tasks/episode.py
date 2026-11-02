"""
What every one of RoboCasa's atomic tasks shares when a PR2 performs it: the kitchen and
objects RoboCasa composes, the PR2 standing where RoboCasa starts its own robot, the
motions simulated kinematically with collision avoidance, and RoboCasa's own success
check as the judge.

The PR2's motions are simulated kinematically, or physically in a MuJoCo simulation of
the kitchen built from the world, where the robot's servos move its joints, objects are
held by the friction of its fingers and fall where they are let go of, and the fixtures'
joints move only where the robot pushes them.

RoboCasa's own simulation is never stepped. It only judges: after the PR2 has acted, the
poses of the objects and the positions of the fixture joints are copied into it, its
fixtures update their state from those joints, and its gripper is taken to be where the
PR2's is. What RoboCasa learns by contact alone, a button pressed by the gripper, is
told to it while the PR2's fingertips rest on the button.

Every task is a subclass of :class:`RoboCasaAtomicTask` named after the RoboCasa task it
performs, in the module named after the RoboCasa file that defines that task.
"""

from __future__ import annotations

import logging
import math
import time
from abc import ABC, abstractmethod
from datetime import timedelta
from types import FunctionType
from contextlib import contextmanager
from dataclasses import dataclass, field
from enum import StrEnum
from functools import cached_property
from pathlib import Path

import numpy as np
from typing_extensions import Any, ClassVar, Dict, Iterator, List, Self, Sequence

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import ExecutionType
import mujoco

from coraplex.plans.executables import GiskardExecutable
from coraplex.execution_environment import (
    ExecutionEnvironment,
    PhysicallySimulatedRobot,
)
from coraplex.plans.factories import sequential
from coraplex.plans.failures import EmptyUnderspecified, PlanFailure
from giskardpy.qp.exceptions import InfeasibleException
from coraplex.robot_plans.actions.core.robot_body import (
    MoveTorsoAction,
    ParkArmsAction,
)
from experiments.robocasa_goals.simulation_mirror import RoboCasaSimulationMirror
from experiments.robocasa_task_execution.exceptions import NoClearStandingPose
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
    RoboCasaTask,
)
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import (
    StaticJointState,
    TorsoState,
)
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Handle,
    KnobWithHandle,
    Lid,
)
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    OmniDrive,
    PrismaticConnection,
)
from semantic_digital_twin.world_description.world_entity import Body

from experiments.robocasa_atomic_tasks.exceptions import (
    MotionWasInfeasible,
    NoPlacementSpot,
    ObjectNotPickedUp,
)
from experiments.robocasa_atomic_tasks.recording import HeadCameraVideo
from experiments.robocasa_atomic_tasks.skills import (
    grasps_approaching_along,
    grasps_from_above,
    part_grasps,
    StandAndOperateMechanismAction,
    StandAndPickUpAction,
    StandAndPlaceAction,
    StandAndPressAction,
    StandAction,
    WithdrawAction,
)

logger = logging.getLogger(__name__)


# %% motions the controller finds no solution for


@contextmanager
def infeasible_motions_fail() -> Iterator[None]:
    """
    Within the context, a motion the controller finds no solution for fails as a plan
    failure, :class:`MotionWasInfeasible`, so that coraplex tries its next candidate
    as it does for a motion that stalls.

    coraplex turns a stall and a collision violation into plan failures where a motion
    crosses into the plan, but lets the solver's infeasibility through, which ends the
    whole plan from inside the trial of a single candidate.
    """
    execute = GiskardExecutable.execute

    def execute_failing_where_infeasible(self) -> None:
        try:
            execute(self)
        except InfeasibleException as infeasibility:
            raise MotionWasInfeasible(infeasibility) from infeasibility

    GiskardExecutable.execute = execute_failing_where_infeasible
    try:
        yield
    finally:
        GiskardExecutable.execute = execute


# %% RoboCasa's robot, as the judge sees it


class RoboCasaRobotFrame(StrEnum):
    """
    The names of the frames of RoboCasa's own robot that its success checks read.
    """

    BASE = "mobilebase0_base"
    """
    The body of the mobile base, whose pose a navigation task is judged by.
    """

    GRIPPER = "right"
    """
    The gripper whose end effector site the checks of a gripper being far read.
    """


ROBOCASA_ROBOT_BODY_PREFIXES = ("robot0_", "mobilebase0_", "gripper0_")
"""
How the names of the bodies of RoboCasa's own robot begin.
"""


# %% one episode of a task


@dataclass
class RoboCasaEpisode:
    """
    One episode of one of RoboCasa's atomic tasks, with a PR2 standing where RoboCasa
    starts its own robot.
    """

    environment: Any
    """
    The reset RoboCasa environment the episode was composed in. It holds RoboCasa's
    ground truth and judges the episode.
    """

    task: RoboCasaTask
    """
    The task, bound to the world the PR2 performs it in.
    """

    robot: PR2
    """
    The robot performing the task.
    """

    simulation: MujocoSim | None = field(default=None, init=False, repr=False)
    """
    The physical simulation of the world the PR2 acts in, while one runs.
    """

    mirror: RoboCasaSimulationMirror = field(init=False, repr=False)
    """
    Keeps RoboCasa's simulation in the state of the world the PR2 acts in.
    """

    handling_margin: ClassVar[float] = 0.4
    """
    How far from an object picked up, or from where it is put down, the fixtures lie
    that the arm may come close to meanwhile, in metres: as far as the open door of a
    fridge beside the counter.
    """

    small_fixture_footprint: ClassVar[float] = 0.5
    """
    The area, in square metres, below which a fixture's footprint counts as small
    enough for it to stand on a counter.
    """

    stand_arm_margin: ClassVar[float] = 0.05
    """
    How far beyond the distance collision avoidance never lets them come closer than
    the arms have to keep from the kitchen where the robot is to stand, in metres.
    """

    pressed_geoms: set[str] = field(default_factory=set, init=False, repr=False)
    """
    The names of RoboCasa's geoms the PR2's fingertips rest on right now.
    """

    def __post_init__(self) -> None:
        self.mirror = RoboCasaSimulationMirror(
            environment=self.environment, world=self.world
        )

    @property
    def world(self) -> World:
        """
        The world the PR2 performs the task in.
        """
        return self.task.world

    @property
    def arm(self) -> Arm:
        """
        The arm the PR2 performs the task with; RoboCasa's robot acts with its right
        arm too.
        """
        return self.robot.right_arm

    def context(self, sampling_seed: int = 42) -> Context:
        """
        :param sampling_seed: The seed the stands are sampled with.
        :return: A context of the episode's world and robot.
        """
        return Context(self.world, self.robot, sampling_seed=sampling_seed)

    @property
    def physically_simulated(self) -> bool:
        """
        Whether the PR2 acts in a physical simulation of the world.
        """
        return self.simulation is not None

    @contextmanager
    def executed(
        self, collision_avoidance: bool = True, physics: bool = False
    ) -> Iterator[None]:
        """
        Perform the steps within the context kinematically simulated, or physically
        simulated in MuJoCo, which is started on entering and stopped on leaving.

        :param collision_avoidance: Whether the PR2 avoids colliding with the kitchen
            and with itself while moving.
        :param physics: Whether the steps are simulated physically.
        """
        with infeasible_motions_fail():
            if not physics:
                with ExecutionEnvironment(
                    ExecutionType.KINEMATICALLY_SIMULATED,
                    collision_avoidance=collision_avoidance,
                ):
                    yield
                return
            began = time.time()
            self.simulation = self.start_physical_simulation()
            logger.info("Physical simulation started in %.0f s", time.time() - began)
            try:
                with PhysicallySimulatedRobot(
                    simulation=self.simulation, collision_avoidance=collision_avoidance
                ):
                    yield
            finally:
                self.simulation.stop_simulation()
                self.simulation = None

    def start_physical_simulation(
        self,
        step_size: timedelta = timedelta(milliseconds=2),
        impedance_ratio: float = 10.0,
    ) -> MujocoSim:
        """
        Start simulating the world physically in MuJoCo, stepped by whatever performs
        in it.

        Friction is resolved in elliptic cones, and the impedance ratio raised above
        MuJoCo's default of one, as MuJoCo's documentation recommends where objects are
        held by friction.

        :param step_size: How much simulated time one physics step advances.
        :param impedance_ratio: How much harder the contacts resist sliding than
            pressing in; MuJoCo's ``impratio``.
        :return: The started simulation.
        """
        simulation = MujocoSim(
            world=self.world,
            headless=True,
            step_size=step_size.total_seconds(),
            integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
            cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
            impratio=impedance_ratio,
        )
        simulation.start_stepped_simulation()
        return simulation

    def let_physics_run(self, duration: timedelta = timedelta(seconds=1)) -> None:
        """
        Let the physical simulation run on with the robot holding still, for what it
        let go of to come to rest; nothing happens in a kinematic simulation.

        :param duration: How much simulated time passes.
        """
        if self.physically_simulated:
            self.simulation.step_simulation(duration)

    # %% composing

    @classmethod
    def compose(
        cls,
        task_name: str,
        layout: LayoutType,
        style: StyleType,
        seed: int | None = None,
        task_parameters: Dict[str, Any] | None = None,
    ) -> Self:
        """
        Compose a new episode of a task.

        :param task_name: The name of the RoboCasa task environment.
        :param layout: The kitchen layout.
        :param style: The visual style of the kitchen.
        :param seed: The seed RoboCasa samples the objects and their placements with.
        :param task_parameters: Arguments of the RoboCasa task itself, by name.
        :return: The episode, with the PR2 standing clear of the kitchen at or behind
            RoboCasa's start pose.
        """
        loader = RoboCasaDatasetLoader()
        environment = loader.build_task_environment(
            task_name, layout, style, seed=seed, task_parameters=task_parameters
        )
        task = loader.load_task_from_environment(environment)
        world = task.world
        pr2_world = URDFParser.from_file(PR2.get_ros_file_path()).parse()
        with world.modify_world():
            drive = OmniDrive.create_with_dofs(
                parent=world.root, child=pr2_world.root, world=world
            )
            world.merge_world(pr2_world, drive)
            drive.has_hardware_interface = True
        drive.origin = task.robot_base_pose
        robot = PR2.from_world(world)
        # The base takes part in every motion of the arm, so that the arm reaches the
        # back of a counter the base cannot stand any nearer to.
        robot.mobile_base.full_body_controlled = True
        episode = cls(environment=environment, task=task, robot=robot)
        episode.stand_clear_of_the_kitchen()
        return episode

    def stand_clear_of_the_kitchen(
        self, step: float = 0.05, maximum_distance: float = 1.0
    ) -> None:
        """
        Park the arms of the PR2 and step it back from RoboCasa's start pose, and
        sideways where stepping back alone does not do, until it keeps clear of the
        kitchen by the distances collision avoidance keeps it at, while letting its
        base rest on the floor.

        Standing nearer than those distances, collision avoidance pushes the robot
        away from the kitchen from its very first motion, against whatever the motion
        asks for.

        :param step: How far the PR2 steps at a time.
        :param maximum_distance: How far back and to either side of the start pose the
            PR2 stands at most.
        :raises NoClearStandingPose: If the PR2 comes too close to the kitchen wherever
            it stands up to ``maximum_distance`` behind and beside the start pose.
        """
        world = self.world
        with world.modify_world():
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=list(self.robot.bodies_with_collision),
                    body_group_b=[
                        floor.root
                        for floor in world.get_semantic_annotations_by_type(Floor)
                    ],
                )
            )
        world.collision_manager.update_collision_matrix()
        for arm in self.robot.all_arms:
            arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)
        start_pose = self.task.robot_base_pose.to_np()
        drive = self.robot.root.parent_connection
        offsets = sorted(
            (
                (back, side)
                for back in np.arange(0.0, maximum_distance + step / 2, step)
                for side in np.arange(-maximum_distance / 2, maximum_distance / 2 + step / 2, step)
            ),
            key=lambda offset: (abs(offset[0]) + 2 * abs(offset[1]), abs(offset[1])),
        )
        for back, side in offsets:
            start_T_standing_pose = np.eye(4)
            start_T_standing_pose[0, 3] = -back
            start_T_standing_pose[1, 3] = side
            drive.origin = start_pose @ start_T_standing_pose
            if self.robot_keeps_clear():
                return
        raise NoClearStandingPose(
            start_pose=self.task.robot_base_pose, maximum_distance=maximum_distance
        )

    def robot_keeps_clear(
        self, base_margin: float = 0.03, arm_margin: float | None = None
    ) -> bool:
        """
        :param base_margin: How far beyond the distance collision avoidance never lets
            it come closer than the robot's base has to keep, in metres.
        :param arm_margin: How far beyond that distance the rest of the robot has to
            keep, in metres; ``None`` for the distance collision avoidance keeps it at.
        :return: Whether the robot keeps those distances from everything else and
            from its own other bodies.

        Standing nearer than the distance collision avoidance keeps it at, an arm is
        pushed away from its very first motion, against whatever the motion asks for.
        The base may come closer, since a stand that close to a counter is often the
        only one from which the arm reaches its back.
        """
        collision_manager = self.world.collision_manager
        own = set(self.robot.bodies_with_collision)
        base = self.world.get_body_in_branch_by_name(self.robot.root, "base_link")

        def required_distance(body_a: Body, body_b: Body) -> float:
            violated = collision_manager.get_violated_distance(body_a, body_b)
            if base in (body_a, body_b):
                return violated + base_margin
            if arm_margin is not None:
                return violated + arm_margin
            return collision_manager.get_buffer_zone_distance(body_a, body_b)

        return not any(
            contact.distance < required_distance(contact.body_a, contact.body_b)
            for contact in collision_manager.compute_collisions().contacts
            if contact.body_a in own or contact.body_b in own
        )

    def bodies_too_close(self, margin: float = 0.005) -> List[tuple[Body, Body]]:
        """
        :param margin: How far beyond the distance collision avoidance never lets them
            come closer than two bodies count as too close, in metres.
        :return: The pairs of a body of the robot and another body, of the kitchen or
            of the robot, that are closer than collision avoidance lets any motion
            bring them, the robot's first.
        """
        collision_manager = self.world.collision_manager
        own = set(self.robot.bodies_with_collision)
        pairs = []
        for contact in collision_manager.compute_collisions().contacts:
            body_a, body_b = contact.body_a, contact.body_b
            if body_a not in own:
                if body_b not in own:
                    continue
                body_a, body_b = body_b, body_a
            violated = collision_manager.get_violated_distance(body_a, body_b)
            if contact.distance < violated + margin:
                pairs.append((body_a, body_b))
        return pairs

    def get_clear(self, base_step: float = 0.1) -> bool:
        """
        Free the robot where it is closer to something than collision avoidance lets
        any motion bring it: park the arms, and step the base away from what it is too
        close to, letting just the bodies too close touch meanwhile.

        A motion that went wrong in a physical simulation may leave the robot that
        close, such as a forearm that a door swung back onto, or a base that drifted
        against an appliance; every motion from there violates collision avoidance
        from its very first tick.

        :param base_step: How far the base steps away, in metres.
        :return: Whether the robot was too close to anything.
        """
        pairs = self.bodies_too_close()
        if not pairs:
            return False
        logger.warning(
            "Getting clear of %s",
            ", ".join(sorted({f"{a.name.name}-{b.name.name}" for a, b in pairs})),
        )
        steps = [ParkArmsAction(self.robot.all_arms)]
        base = self.world.get_body_in_branch_by_name(self.robot.root, "base_link")
        base_against = [other for body, other in pairs if body is base]
        if base_against:
            world_T_base = self.robot.root.global_pose.to_np()
            away = world_T_base[:2, 3] - np.mean(
                [self.bounding_box_center(other)[:2] for other in base_against], axis=0
            )
            away /= max(np.linalg.norm(away), 1e-9)
            world_T_stand = world_T_base.copy()
            world_T_stand[:2, 3] += base_step * away
            steps.append(
                StandAction(
                    HomogeneousTransformationMatrix(
                        world_T_stand, reference_frame=self.world.root
                    ).pose
                )
            )
        rule = AllowCollisionBetweenGroups(
            body_group_a=list({body for body, _ in pairs}),
            body_group_b=list({other for _, other in pairs}),
        )
        collision_manager = self.world.collision_manager
        with self.world.modify_world():
            collision_manager.add_ignore_collision_rule(rule)
        collision_manager.update_collision_matrix()
        try:
            sequential(steps, self.context()).perform()
        except PlanFailure as failure:
            logger.warning(
                "Getting clear failed after %s: %s",
                type(failure).__name__,
                str(failure).splitlines()[0][:200],
            )
        finally:
            with self.world.modify_world():
                collision_manager.ignore_collision_rules.remove(rule)
            collision_manager.update_collision_matrix()
        return True

    # %% finding what RoboCasa refers to

    def object_body(self, name: str) -> Body:
        """
        :param name: The name RoboCasa gives one of the task's objects, such as
            ``"obj"``.
        :return: The object's root body.
        """
        return self.world.get_body_by_name(self.environment.objects[name].root_body)

    def graspable(self, name: str) -> HasGraspCandidates:
        """
        :param name: The name RoboCasa gives one of the task's objects.
        :return: The annotation of the object that says where it may be grasped.
        """
        body = self.object_body(name)
        return next(
            annotation
            for annotation in self.world.get_semantic_annotations_by_type(
                HasGraspCandidates
            )
            if annotation.root is body
        )

    def loose_part(self, body_name: str) -> HasGraspCandidates:
        """
        :param body_name: The name RoboCasa gives the body of a loose part of a
            fixture, such as the lid of a blender.
        :return: The part's annotation that says where it may be grasped, made a lid
            if it has none yet.
        """
        body = self.world.get_body_by_name(body_name)
        for annotation in self.world.get_semantic_annotations_by_type(
            HasGraspCandidates
        ):
            if annotation.root is body:
                return annotation
        lid = Lid(root=body)
        with self.world.modify_world():
            self.world.add_semantic_annotation(lid)
        return lid

    def pick_up_graspable(
        self, graspable: HasGraspCandidates, attempts: int = 3
    ) -> None:
        """
        Pick an object up, standing wherever it can be reached from, and park the arms
        again with it in the gripper.

        In a physical simulation the fingers hold the object by friction alone, and a
        pick-up whose motions all succeed may leave it behind; the object is then picked
        up again from where it lies.

        :param graspable: The annotation of the object.
        :param attempts: How often the object is picked up at most in a physical
            simulation.
        :raises ObjectNotPickedUp: If the gripper holds the object after none of the
            attempts.
        """
        for attempt in range(attempts if self.physically_simulated else 1):
            with self.gripper_may_touch(
                self.fixture_bodies_near(
                    *self.bounding_box(graspable.root), margin=self.handling_margin
                )
            ):
                self._pick_up(graspable)
            if not self.physically_simulated or self.holds(graspable):
                return
            logger.warning("%s was left behind by the pick-up", graspable.root.name)
        raise ObjectNotPickedUp(body=graspable.root)

    def holds(self, graspable: HasGraspCandidates, reach: float = 0.12) -> bool:
        """
        :param graspable: The annotation of an object.
        :param reach: How far from the tool frame of the acting arm the object's
            bounding box may lie at most, in metres.
        :return: Whether the acting arm's gripper holds the object: whether it hangs
            off its tool frame or, held by friction, lies between its fingers.
        """
        if graspable.root in self.world.get_kinematic_structure_entities_of_branch(
            self.robot.root
        ):
            return True
        tool = self.arm.end_effector.tool_frame.global_pose.to_np()[:3, 3]
        lower, upper = self.bounding_box(graspable.root)
        return float(np.linalg.norm(tool - np.clip(tool, lower, upper))) <= reach

    def _pick_up(self, graspable: HasGraspCandidates) -> None:
        """
        Pick an object up, see :meth:`pick_up_graspable`.
        """
        self.raise_torso_for(graspable.root.global_pose.to_np()[:3, 3])
        stands = self.stands_facing(graspable.root.global_pose.to_np()[:3, 3])
        grasps = grasps_from_above(graspable, self.kitchen_direction) + (
            grasps_approaching_along(graspable.grasp_candidates(), self.kitchen_direction)
            or graspable.grasp_candidates()
        )
        self.perform_reaching(
            ParkArmsAction(self.robot.all_arms),
            lambda through_free_space: StandAndPickUpAction.from_stands(
                graspable,
                self.arm,
                stands,
                grasps=grasps,
                through_free_space=through_free_space,
            ),
            ParkArmsAction(self.robot.all_arms),
            retries=2,
        )

    def fixture_body(self, fixture: Any) -> Body:
        """
        :param fixture: One of RoboCasa's fixtures.
        :return: The fixture's root body.
        """
        return self.world.get_body_by_name(fixture.root_body)

    def joint(self, name: str) -> ActiveConnection1DOF:
        """
        :param name: The name RoboCasa gives one of its fixtures' joints.
        :return: The joint, as it is in the world.
        """
        return next(
            connection
            for connection in self.world.connections
            if isinstance(connection, ActiveConnection1DOF)
            and connection.name.name == name
        )

    def part_moved_by(self, joint_name: str) -> Handle:
        """
        :param joint_name: The name RoboCasa gives one of its fixtures' joints.
        :return: The part hanging from the joint, annotated as what it is taken hold of
            by: its handle where it has one, otherwise the part itself.
        """
        part = self.joint(joint_name).child
        if not part.has_collision():
            part = next(
                body
                for body in self.world.compute_descendent_child_kinematic_structure_entities(
                    part
                )
                if isinstance(body, Body) and body.has_collision()
            )
        handles = [
            handle
            for handle in self.world.get_semantic_annotations_by_type(Handle)
            if handle.root.has_collision()
            and (
                handle.root is part
                or self.joint_of(handle.root) is self.joint(joint_name)
            )
        ]
        if handles:
            return handles[0]
        handle = Handle(root=part)
        with self.world.modify_world():
            self.world.add_semantic_annotation(handle)
        return handle

    @staticmethod
    def joint_of(body: Body) -> ActiveConnection1DOF | None:
        """
        :param body: A body of the kitchen.
        :return: The nearest joint of one degree of freedom above the body, if any.
        """
        try:
            return body.get_first_parent_connection_of_type(ActiveConnection1DOF)
        except Exception:
            return None

    def mirrored(self, joint_name: str) -> bool:
        """
        :param joint_name: The name of one of RoboCasa's joints.
        :return: Whether the world holds the joint mirrored, opening towards positive
            positions where RoboCasa's opens towards negative ones.
        """
        model = self.environment.sim.model
        lower, upper = model.jnt_range[model.joint_name2id(joint_name)]
        return lower < upper == 0.0

    def world_position_of(self, joint_name: str, robocasa_position: float) -> float:
        """
        :param joint_name: The name of one of RoboCasa's joints.
        :param robocasa_position: A position of the joint as RoboCasa states it.
        :return: The same position as the world states it.
        """
        if self.mirrored(joint_name):
            return -robocasa_position
        return robocasa_position

    def world_position_at_share(self, joint_name: str, share: float) -> float:
        """
        :param joint_name: The name of one of RoboCasa's joints.
        :param share: How far along its travel the joint stands, as RoboCasa normalizes
            it for a door: 0 closed, 1 fully open.
        :return: The position of the joint at that share, as the world states it.
        """
        model = self.environment.sim.model
        lower, upper = model.jnt_range[model.joint_name2id(joint_name)]
        if lower < 0:
            robocasa_position = upper - share * (upper - lower)
        else:
            robocasa_position = lower + share * (upper - lower)
        return self.world_position_of(joint_name, robocasa_position)

    def world_position_along_range(self, joint_name: str, fraction: float) -> float:
        """
        :param joint_name: The name of one of RoboCasa's joints.
        :param fraction: How far along the joint's range from its lower to its upper
            limit, as RoboCasa states them, a position lies; RoboCasa normalizes a knob
            or a faucet handle this way.
        :return: The position, as the world states it.
        """
        model = self.environment.sim.model
        lower, upper = model.jnt_range[model.joint_name2id(joint_name)]
        return self.world_position_of(joint_name, lower + fraction * (upper - lower))

    # %% acting

    def perform(self, *steps: Any, retries: int = 0) -> None:
        """
        Perform steps one after another in the active execution environment.

        :param steps: The actions, or matches of actions left underspecified.
        :param retries: How often the steps are performed again from the start if a
            motion of theirs fails. A candidate that succeeded in coraplex's trial in a
            copy of the world now and then fails when performed for real; candidates
            that all failed their trials fail again.
        Before every attempt, the robot is freed where it is too close to something, see
        :meth:`get_clear`; candidates that all failed their trials from there are tried
        again once it is free.

        :raises PlanFailure: If the steps fail more often than ``retries``.
        """
        for attempt in range(retries + 1):
            self.get_clear()
            try:
                sequential(list(steps), self.context()).perform()
                return
            except PlanFailure as failure:
                if attempt == retries or (
                    isinstance(failure, EmptyUnderspecified)
                    and not self.bodies_too_close()
                ):
                    raise
                logger.warning(
                    "Performing again after %s: %s",
                    type(failure).__name__,
                    str(failure).splitlines()[0][:200],
                )

    def perform_reaching(self, *steps: Any, retries: int = 2) -> None:
        """
        Perform steps one after another, some of which walk the robot to a stand and
        act from there; if they fail, perform them again with the gripper led to where
        each action begins along a path through free space.

        Collision avoidance alone stops the gripper in front of whatever stands in the
        straight way to its goal; a path through free space leads it around, but also
        holds the arm to a way the shoulder may not follow, so it is tried second.

        :param steps: The actions, or matches of actions left underspecified, or
            functions that make one from whether it reaches through free space.
        :param retries: How often the steps are performed again, each way, see
            :meth:`perform`.
        :raises PlanFailure: If the steps fail both ways.
        """
        for through_free_space in (False, True):
            try:
                self.perform(
                    *(
                        step(through_free_space)
                        if isinstance(step, FunctionType)
                        else step
                        for step in steps
                    ),
                    retries=retries,
                )
                return
            except PlanFailure as failure:
                if through_free_space:
                    raise
                logger.warning(
                    "Reaching through free space after %s: %s",
                    type(failure).__name__,
                    str(failure).splitlines()[0][:200],
                )

    def raise_torso_for(self, point: np.ndarray, height: float = 0.85) -> None:
        """
        Raise the torso fully before picking up or putting down an object at a
        counter's height or above, so that the arm comes down onto it from above rather
        than along the counter; lower points are left to the torso as it stands. A
        part of a fixture is reached level, which the torso as it stands does better.

        :param point: The point to be reached, in the world's root frame.
        :param height: From how high up a point counts as at a counter's height.
        """
        if point[2] < height:
            return
        self.perform(MoveTorsoAction(TorsoState.HIGH))

    def park(self) -> None:
        """
        Park both arms.
        """
        self.perform(ParkArmsAction(self.robot.all_arms))

    def pick_up(self, name: str) -> None:
        """
        Pick one of the task's objects up, standing wherever it can be reached from,
        and park the arms again with it in the gripper.

        :param name: The name RoboCasa gives the object.
        """
        self.pick_up_graspable(self.graspable(name))

    def operate(
        self,
        joint_name: str,
        world_position: float,
        threshold: float = 0.01,
        attempts: int = 4,
        shortfall: float = 0.05,
        minimum_following: float = 0.4,
    ) -> None:
        """
        Move one of the fixtures' joints to a position by the part hanging from it,
        standing wherever the part can be reached from, and park the arms afterwards.

        In a physical simulation the motion moves the joint as far as the fingers drag
        the part, which they may slip off on the way or trail behind, while the
        controller takes the joint to follow them all the way. Where the joint has
        fallen short, the part is taken hold of again from where it stands and moved
        on, aiming as far beyond the position as the joint fell behind last time.

        :param joint_name: The name RoboCasa gives the joint.
        :param world_position: The position the joint is moved to, as the world states
            it, kept within the joint's limits.
        :param threshold: How close the joint has to come to the position.
        :param attempts: How often the part is taken hold of at most in a physical
            simulation.
        :param shortfall: How far short of the position, as a share of the joint's
            travel, the joint may stop in a physical simulation before it is moved on.
        :param minimum_following: The smallest share of the way it is told to go that
            the joint is taken to go when the next attempt aims beyond the position,
            which keeps an attempt that barely moved the joint from aiming far past it.
        """
        joint = self.joint(joint_name)
        handle = self.part_moved_by(joint_name)
        goal = self.within_limits(joint, world_position)
        limits = joint.dof.limits
        travel = limits.upper.position - limits.lower.position
        # how much of the way it is told to go the joint goes, as the last attempt
        # found; the next is told to go that much further
        followed = 1.0
        for attempt in range(attempts if self.physically_simulated else 1):
            start = joint.position
            aim = self.within_limits(
                joint, start + (goal - start) / max(followed, minimum_following)
            )
            logger.info(
                "Moving %s from %.3f to %.3f, aiming at %.3f",
                joint_name,
                start,
                goal,
                aim,
            )
            with self.gripper_may_touch(self.fixtures_around(handle.root)):
                self._operate(handle, joint, aim, threshold)
            if abs(joint.position - goal) <= shortfall * travel:
                return
            logger.warning(
                "%s stopped at %.3f, short of %.3f", joint_name, joint.position, goal
            )
            if abs(aim - start) > 1e-6:
                followed = float(
                    np.clip((joint.position - start) / (aim - start), 0.0, 1.0)
                )

    def _operate(
        self,
        handle: Handle,
        joint: ActiveConnection1DOF,
        goal: float,
        threshold: float,
    ) -> None:
        """
        Move a joint to a position by a part hanging from it, see :meth:`operate`.
        """
        stands = self.stands_facing(self.bounding_box_center(handle.root))
        grasps = grasps_approaching_along(part_grasps(handle), self.kitchen_direction)
        knob = self.knob_turned_by(joint)
        turned_by_the_wrist = knob is not None
        if turned_by_the_wrist:
            grasps = knob.turning_grasp_candidates(self.kitchen_direction) + grasps
        self.perform_reaching(
            ParkArmsAction(self.robot.all_arms),
            lambda through_free_space: StandAndOperateMechanismAction.from_stands(
                handle=handle,
                arm=self.arm,
                stands=stands,
                grasps=grasps,
                joint=joint,
                goal_position=goal,
                threshold=threshold,
                through_free_space=through_free_space,
                hold_base=turned_by_the_wrist,
            ),
            ParkArmsAction(self.robot.all_arms),
            retries=2,
        )
        logger.info("%s moved to %.4f", joint.name.name, joint.position)

    @staticmethod
    def within_limits(joint: ActiveConnection1DOF, position: float) -> float:
        """
        :param joint: A joint of the world.
        :param position: A position of it.
        :return: The position, moved within the joint's limits.
        """
        limits = joint.dof.limits
        return float(
            np.clip(position, limits.lower.position, limits.upper.position)
        )

    def press(self, geom_name: str, approach: np.ndarray | None = None) -> None:
        """
        Press one of RoboCasa's geoms with the fingertips, such as a button, tell
        RoboCasa's fixtures about the contact while the fingertips rest on it, and
        withdraw and park the arms afterwards.

        :param geom_name: The name RoboCasa gives the geom.
        :param approach: The direction the fingertips come in along; by default
            levelled from the robot towards the geom.
        """
        self.mirror.synchronize()
        if approach is None:
            approach = self.approach_towards(self.geom_surface_point(geom_name))
        point = self.geom_surface_point(geom_name, approach)
        body_name = self.environment.sim.model.body_id2name(
            self.environment.sim.model.geom_bodyid[
                self.environment.sim.model.geom_name2id(geom_name)
            ]
        )
        with self.gripper_may_touch(
            self.fixtures_around(self.world.get_body_by_name(body_name))
        ):
            stands = self.stands_facing(point)
            self.perform_reaching(
                ParkArmsAction(self.robot.all_arms),
                lambda through_free_space: StandAndPressAction.from_stands(
                    point=point,
                    approach=approach,
                    arm=self.arm,
                    stands=stands,
                    through_free_space=through_free_space,
                ),
            )
            touching = self.fingertips_touch(point)
            logger.info("Fingertips touch %s: %s", geom_name, touching)
            if touching:
                with self.geoms_pressed(geom_name):
                    self.update_fixture_states()
            self.withdraw(point, approach)

    def push(
        self,
        joint_name: str,
        world_position: float,
        approach: np.ndarray | None = None,
    ) -> None:
        """
        Press the part hanging from one of the fixtures' joints with the fingertips and
        push it on until the joint stands at a position, such as a switch or a lever,
        then withdraw and park the arms.

        :param joint_name: The name RoboCasa gives the joint.
        :param world_position: The position the joint is pushed to, as the world
            states it, kept within the joint's limits.
        :param approach: The direction the fingertips come in along; by default along
            the way the joint pushes the part, or levelled from the robot towards the
            part where the joint turns it.
        """
        joint = self.joint(joint_name)
        part = self.part_moved_by(joint_name).root
        center = self.bounding_box_center(part)
        if approach is None:
            approach = self.push_direction(joint, world_position)
            toward = self.approach_towards(center)
            if approach is None:
                approach = toward
            elif abs(approach[2]) > 0.7:
                # A part pushed down is pressed from above and in front at once: the
                # PR2's wrist does not bend far enough to point the fingertips
                # straight down at the height of a counter's appliances.
                approach = approach + toward
                approach = approach / np.linalg.norm(approach)
        point = self.box_entry_point(part, center, approach)
        goal = self.within_limits(joint, world_position)
        logger.info("Pushing %s from %.4f to %.4f", joint_name, joint.position, goal)
        with self.gripper_may_touch(self.fixtures_around(part)):
            self._push(point, approach, joint, goal)

    def _push(
        self,
        point: np.ndarray,
        approach: np.ndarray,
        joint: ActiveConnection1DOF,
        goal: float,
    ) -> None:
        """
        Press a point and push the part there on to a joint position, see
        :meth:`push`.
        """
        stands = self.stands_facing(point)
        travel = joint.dof.limits.upper.position - joint.dof.limits.lower.position
        # The fingertips withdraw from where the part took them, the way they came in.
        part_T_world = np.linalg.inv(joint.child.global_pose.to_np())
        part_point = part_T_world[:3, :3] @ point + part_T_world[:3, 3]
        part_approach = part_T_world[:3, :3] @ approach
        self.perform_reaching(
            ParkArmsAction(self.robot.all_arms),
            lambda through_free_space: StandAndPressAction.from_stands(
                point=point,
                approach=approach,
                arm=self.arm,
                stands=stands,
                pushed_joint=joint,
                goal_position=goal,
                threshold=0.02 * travel,
                through_free_space=through_free_space,
            ),
        )
        logger.info("%s pushed to %.4f", joint.name.name, joint.position)
        world_T_part = joint.child.global_pose.to_np()
        pushed_point = world_T_part[:3, :3] @ part_point + world_T_part[:3, 3]
        pushed_approach = world_T_part[:3, :3] @ part_approach
        if not self.physically_simulated:
            self.snap_to_end_stop(joint, goal)
        self.update_fixture_states()
        self.withdraw(pushed_point, pushed_approach)

    def pushable(self, approach: np.ndarray, facing: float = 0.3) -> bool:
        """
        :param approach: The direction fingertips would push a part in.
        :param facing: How far at least, as the cosine of the angle between them, the
            direction has to point along the one the robot faces the kitchen in.
        :return: Whether fingertips coming from the robot reach the side of the part
            they would push: pushing it into the kitchen, or down.
        """
        return float(approach @ self.kitchen_direction) > facing or approach[2] < -0.5

    def push_shut(
        self,
        joint_name: str,
        world_position: float,
        attempts: int = 3,
        shortfall: float = 0.004,
    ) -> None:
        """
        Push a door, lid, drawer or rack shut with the fingertips, pushing again where
        it stopped short, and take hold of it by its handle where no push gets it
        shut.

        Held by friction alone, the fingers slip off a part they drag shut, while
        fingertips pushing it need no grip. They press the part, together with its
        handle, on the side it moves away from, through the middle of the handle,
        which lies furthest from the hinge, along the way the handle moves on closing.
        A door swung wide open turns that side away from the robot, out of the reach
        of its fingertips, and a drop-down door opened wide holds it low above the
        floor; such a part is first taken hold of by its handle and moved most of the
        way, see :meth:`operate`, then pushed shut, and taken hold of again if that
        fails too.

        :param joint_name: The name RoboCasa gives the joint.
        :param world_position: Its closed position, as the world states it.
        :param attempts: How often the part is pushed at most.
        :param shortfall: How far short of the position, as a share of the joint's
            travel, the joint may stop: within RoboCasa's tolerance for a closed door,
            half a percent.
        """
        joint = self.joint(joint_name)
        handle = self.part_moved_by(joint_name)
        goal = self.within_limits(joint, world_position)
        limits = joint.dof.limits
        travel = limits.upper.position - limits.lower.position

        def shut() -> bool:
            return abs(joint.position - goal) <= shortfall * travel

        def pushed_shut() -> bool:
            for attempt in range(attempts):
                if shut():
                    return True
                center = self.bounding_box_center(handle.root)
                approach = self.push_direction(joint, goal, center)
                if abs(approach[2]) > 0.7:
                    # pressed from above and in front at once, see push
                    approach = approach + self.approach_towards(center)
                    approach = approach / np.linalg.norm(approach)
                # only what moves with the part is pushed, not a button on it with a
                # joint of its own, such as the one that releases a kettle's lid
                boxes = [
                    self.bounding_box(joint.child, rigidly_attached=True),
                    self.bounding_box(handle.root, rigidly_attached=True),
                ]
                point = self.entry_point(
                    np.min([lower for lower, _ in boxes], axis=0),
                    np.max([upper for _, upper in boxes], axis=0),
                    center,
                    approach,
                )
                logger.info(
                    "Pushing %s shut from %.4f to %.4f",
                    joint_name,
                    joint.position,
                    goal,
                )
                try:
                    with self.gripper_may_touch(self.fixtures_around(handle.root)):
                        self._push(point, approach, joint, goal)
                except PlanFailure as failure:
                    logger.warning(
                        "Pushing %s shut failed after %s: %s",
                        joint_name,
                        type(failure).__name__,
                        str(failure).splitlines()[0][:200],
                    )
                    return False
            return shut()

        # Where pushing fails, the part is taken hold of and moved most of the way,
        # which turns the side to push towards the robot, and pushed from there.
        if self.pushable(
            self.push_direction(joint, goal, self.bounding_box_center(handle.root))
        ) and pushed_shut():
            return
        self.operate(joint_name, world_position)
        if pushed_shut():
            return
        self.operate(joint_name, world_position, shortfall=shortfall)

    @staticmethod
    def snap_to_end_stop(
        joint: ActiveConnection1DOF, end_stop: float, share: float = 0.1
    ) -> None:
        """
        Let a switch or lever pushed close to the end of its travel snap there, as a
        real one's detent does; only in a kinematic simulation, as a physical one
        pushes it against its end stop.

        The motion controller stops a pushed joint short of its limit, by a few tenths
        of a millimetre, which is most of the travel of a kettle's switch.

        :param joint: The joint pushed.
        :param end_stop: The end of its travel it was pushed towards.
        :param share: How close to the end, as a share of the travel, the joint has to
            come to snap there.
        """
        limits = joint.dof.limits
        travel = limits.upper.position - limits.lower.position
        if abs(joint.position - end_stop) <= share * travel:
            joint.position = end_stop

    def withdraw(self, point: np.ndarray, approach: np.ndarray) -> None:
        """
        Draw the gripper back from a point it pressed, park the arms and let the
        fixtures update their state.

        :param point: The point pressed.
        :param approach: The direction the fingertips came in along.
        """
        self.perform(
            WithdrawAction(point, approach, self.arm),
            ParkArmsAction(self.robot.all_arms),
        )
        self.update_fixture_states()

    def knob_turned_by(self, joint: ActiveConnection1DOF) -> KnobWithHandle | None:
        """
        :param joint: A joint of the kitchen.
        :return: The knob the joint turns, if it is one turned by a bar on its face.
        """
        return next(
            (
                knob
                for knob in self.world.get_semantic_annotations_by_type(KnobWithHandle)
                if knob.turning_joint is joint
            ),
            None,
        )

    @staticmethod
    def push_direction(
        joint: ActiveConnection1DOF,
        world_position: float,
        point: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """
        :param joint: A joint of the world.
        :param world_position: The position it is to be pushed to.
        :param point: A point on the part the joint moves, in the world's root frame.
        :return: The direction the joint moves ``point`` in on the way there, or its
            part if it slides it; ``None`` if it turns the part and no point is given.
        """
        if point is None and not isinstance(joint, PrismaticConnection):
            return None
        start = joint.position
        world_T_part = joint.child.global_pose.to_np()
        before = world_T_part[:3, 3] if point is None else np.asarray(point, dtype=float)
        part_point = np.linalg.inv(world_T_part) @ np.append(before, 1.0)
        joint.position = start + 1e-3 * (1.0 if world_position >= start else -1.0)
        after = (joint.child.global_pose.to_np() @ part_point)[:3]
        joint.position = start
        return (after - before) / np.linalg.norm(after - before)

    def bounding_box_center(self, body: Body) -> np.ndarray:
        """
        :param body: A body with collision geometry.
        :return: The centre of its bounding box in the world's root frame.
        """
        box = body.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_box()
        return np.array(
            [
                (box.min_x + box.max_x) / 2,
                (box.min_y + box.max_y) / 2,
                (box.min_z + box.max_z) / 2,
            ]
        )

    def box_entry_point(
        self, body: Body, center: np.ndarray, approach: np.ndarray
    ) -> np.ndarray:
        """
        :param body: A body with collision geometry.
        :param center: A point inside its bounding box.
        :param approach: A direction.
        :return: Where a ray along ``approach`` through ``center`` enters the body's
            bounding box in the world's root frame.
        """
        box = body.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_box()
        return self.entry_point(
            np.array([box.min_x, box.min_y, box.min_z]),
            np.array([box.max_x, box.max_y, box.max_z]),
            center,
            approach,
        )

    @staticmethod
    def entry_point(
        lower: np.ndarray, upper: np.ndarray, point: np.ndarray, approach: np.ndarray
    ) -> np.ndarray:
        """
        :param lower: The lower corner of a box in the world's root frame.
        :param upper: Its upper corner.
        :param point: A point inside the box.
        :param approach: A direction.
        :return: Where a ray along ``approach`` through ``point`` enters the box.
        """
        with np.errstate(divide="ignore"):
            steps = np.where(
                approach > 1e-9,
                (point - lower) / approach,
                np.where(approach < -1e-9, (point - upper) / approach, np.inf),
            )
        return point - approach * float(np.min(steps))

    # %% letting the gripper touch what it acts on

    def fixture_bodies_near(
        self, lower: np.ndarray, upper: np.ndarray, margin: float = 0.15
    ) -> List[Body]:
        """
        :param lower: The lower corner of a box in the world's root frame.
        :param upper: Its upper corner.
        :param margin: How far beyond the box a fixture's body may begin.
        :return: The bodies of the kitchen's fixtures whose bounding boxes come within
            ``margin`` of the box, such as the counter an object stands on and the
            cabinet it stands in.
        """
        prefixes = [
            fixture.naming_prefix for fixture in self.environment.fixtures.values()
        ]
        near = []
        for body in self.world.bodies_with_collision:
            if not any(body.name.name.startswith(prefix) for prefix in prefixes):
                continue
            body_lower, body_upper = self.bounding_box(body)
            if np.all(body_lower <= upper + margin) and np.all(
                body_upper >= lower - margin
            ):
                near.append(body)
        return near

    def bodies_of_fixture(self, fixture: Any) -> List[Body]:
        """
        :param fixture: One of RoboCasa's fixtures.
        :return: Every body of the fixture with collision geometry.
        """
        return [
            body
            for body in self.world.bodies_with_collision
            if body.name.name.startswith(fixture.naming_prefix)
        ]

    def fixture_bodies_of(self, body: Body) -> List[Body]:
        """
        :param body: A body of one of the kitchen's fixtures, such as a knob.
        :return: Every body of that fixture with collision geometry; just ``body`` if
            it belongs to no fixture RoboCasa names.
        """
        prefixes = [
            fixture.naming_prefix
            for fixture in self.environment.fixtures.values()
            if body.name.name.startswith(fixture.naming_prefix)
        ]
        if not prefixes:
            return [body]
        prefix = max(prefixes, key=len)
        return [
            other
            for other in self.world.bodies_with_collision
            if other.name.name.startswith(prefix)
        ]

    def fixtures_around(self, part: Body, margin: float = 0.3) -> List[Body]:
        """
        :param part: A part of one of the kitchen's fixtures, such as a knob.
        :param margin: How far from the part the bodies of other fixtures may begin.
        :return: The bodies of the part's fixture and of the fixtures near the part,
            such as the wall cabinet above a faucet, that an arm reaching the part
            comes close to.
        """
        near = self.fixture_bodies_near(*self.bounding_box(part), margin=margin)
        return list(dict.fromkeys(self.fixture_bodies_of(part) + near))

    @contextmanager
    def gripper_may_touch(
        self, bodies: Sequence[Body], arm_buffer_zone_distance: float = 0.01
    ) -> Iterator[None]:
        """
        Let the acting arm's gripper touch some bodies, and the rest of the arm come
        nearly as close to them as it likes, while the context is entered; everything
        else is avoided as before.

        Collision avoidance keeps the gripper a hand's breadth and the arm a few
        centimetres from everything, so without this a gripper reaching for a knob is
        held back by the fixture the knob sits on, and a forearm by the door it opens.

        :param bodies: The bodies, such as those of the fixture acted on.
        :param arm_buffer_zone_distance: How close to the bodies collision avoidance
            lets the arm come before pushing it away, in metres.
        """
        gripper_rule = AllowCollisionBetweenGroups(
            body_group_a=list(self.arm.end_effector.bodies_with_collision),
            body_group_b=list(bodies),
        )
        arm_rule = AvoidCollisionBetweenGroups(
            body_group_a=list(self.arm.bodies_with_collision),
            body_group_b=list(bodies),
            buffer_zone_distance=arm_buffer_zone_distance,
            violated_distance=0.0,
        )
        collision_manager = self.world.collision_manager
        with self.world.modify_world():
            collision_manager.add_default_rule(arm_rule)
            collision_manager.add_ignore_collision_rule(gripper_rule)
        collision_manager.update_collision_matrix()
        try:
            yield
        finally:
            with self.world.modify_world():
                collision_manager.ignore_collision_rules.remove(gripper_rule)
                collision_manager.default_rules.remove(arm_rule)
            collision_manager.update_collision_matrix()

    # %% where to stand

    @property
    def kitchen_direction(self) -> np.ndarray:
        """
        The horizontal direction RoboCasa's start pose faces the kitchen in, the way
        the robot reaches into the fixture it starts in front of.
        """
        facing = self.task.robot_base_pose.to_np()[:3, 0]
        facing[2] = 0.0
        return facing / np.linalg.norm(facing)

    def stands_facing(
        self,
        point: np.ndarray,
        distances: Sequence[float] = (0.75, 0.65, 0.85, 0.55, 0.95, 1.05),
        offsets_to_the_left: Sequence[float] = (0.2, 0.1, 0.3, 0.0, 0.4, -0.1, -0.2),
        turns: Sequence[float] = (0.0, 0.4, -0.4),
        number: int = 12,
    ) -> List[Pose]:
        """
        Where the PR2 may stand to reach a point with its right arm: in front of the
        point, facing the kitchen the way RoboCasa starts its robot facing it, a little
        to the left so that the point lies ahead of the right shoulder, and clear of
        the kitchen.

        :param point: The point to reach, in the world's root frame.
        :param distances: How far in front of the point the base may stand, in metres.
        :param offsets_to_the_left: How far to the robot's left of the point the base
            may stand, in metres.
        :param turns: By how much, in radians, the direction the stand lies in may be
            turned about the point away from the direction opposite to the one
            RoboCasa's start pose faces.
        :param number: How many stands are returned at most.
        :return: The stands, those deviating least from the first distance, offset
            and turn first.
        """
        start_facing = self.task.robot_base_pose.to_np()[:2, 0]
        base_angle = math.atan2(-start_facing[1], -start_facing[0])
        drive = self.robot.root.parent_connection
        original = drive.origin.to_np()
        base_height = original[2, 3]
        candidates = sorted(
            (
                (turn, distance, offset)
                for turn in turns
                for distance in distances
                for offset in offsets_to_the_left
            ),
            key=lambda candidate: (
                turns.index(candidate[0])
                + distances.index(candidate[1])
                + offsets_to_the_left.index(candidate[2])
            ),
        )
        stands = []
        try:
            for turn, distance, offset in candidates:
                angle = base_angle + turn
                front = np.array([math.cos(angle), math.sin(angle)])
                left_of_the_robot = np.array([front[1], -front[0]])
                position = np.asarray(point[:2]) + distance * front + offset * left_of_the_robot
                world_T_stand = HomogeneousTransformationMatrix.from_xyz_rpy(
                    position[0],
                    position[1],
                    base_height,
                    yaw=angle + math.pi,
                    reference_frame=self.world.root,
                )
                drive.origin = world_T_stand
                if self.robot_keeps_clear(arm_margin=self.stand_arm_margin):
                    stands.append(world_T_stand.pose)
                if len(stands) >= number:
                    break
        finally:
            drive.origin = HomogeneousTransformationMatrix(
                original, reference_frame=self.world.root
            )
        logger.info("%d stands facing %s", len(stands), np.round(point, 3))
        return stands

    # %% where things are

    def approach_towards(self, point: np.ndarray) -> np.ndarray:
        """
        :param point: A point in the kitchen.
        :return: The levelled direction from the robot's base towards the point.
        """
        base = self.robot.root.global_pose.to_np()[:3, 3]
        direction = np.array(point[:2]) - base[:2]
        return np.append(direction / np.linalg.norm(direction), 0.0)

    def geom_surface_point(
        self, geom_name: str, approach: np.ndarray | None = None
    ) -> np.ndarray:
        """
        :param geom_name: The name RoboCasa gives one of its geoms.
        :param approach: The direction the geom is come at from; ``None`` for the
            geom's centre.
        :return: Where a ray along ``approach`` through the geom's centre enters its
            bounding box, in the world's root frame.
        """
        model, data = self.environment.sim.model, self.environment.sim.data
        geom = model.geom_name2id(geom_name)
        center = np.array(data.geom_xpos[geom])
        if approach is None:
            return center
        rotation = np.array(data.geom_xmat[geom]).reshape(3, 3)
        half_extents = np.array(model.geom_size[geom])
        if model.geom_type[geom] in (2, 3, 5):  # sphere, capsule, cylinder
            half_extents = np.array(
                [half_extents[0], half_extents[0], half_extents[1] or half_extents[0]]
            )
        local_approach = rotation.T @ approach
        with np.errstate(divide="ignore"):
            steps = np.where(
                np.abs(local_approach) > 1e-9,
                half_extents / np.abs(local_approach),
                np.inf,
            )
        return center - approach * float(np.min(steps))

    def fingertips_touch(self, point: np.ndarray, tolerance: float = 0.02) -> bool:
        """
        :param point: A point in the kitchen.
        :param tolerance: How far from the point the fingertips may stop.
        :return: Whether the tool frame of the acting arm stands on the point.
        """
        tool = self.arm.end_effector.tool_frame.global_pose.to_np()[:3, 3]
        return float(np.linalg.norm(tool - point)) <= tolerance

    # %% placing

    def place(
        self,
        graspable: HasGraspCandidates,
        spots: List[np.ndarray],
        retreat_distance: float = 0.1,
    ) -> None:
        """
        Put an object the acting arm holds down on one of a list of spots, standing
        wherever it can be reached from, park the arms, and let the object come to
        rest on what is below it.

        :param graspable: The annotation of the object.
        :param spots: Where the middle of the object's underside may come to lie, best
            first, in the world's root frame.
        :param retreat_distance: How far above its spot the object is carried before
            it is lowered onto it, in metres; less where something overhangs the spot.
        :raises NoPlacementSpot: If no spot is given.
        """
        if not spots:
            raise NoPlacementSpot(body=graspable.root)
        poses = [self.pose_with_underside_at(graspable.root, spot) for spot in spots]
        self.raise_torso_for(np.asarray(spots[0]))
        spread = np.array(spots)
        with self.gripper_may_touch(
            self.fixture_bodies_near(
                spread.min(axis=0), spread.max(axis=0), margin=self.handling_margin
            )
        ):
            stands = self.stands_facing(np.asarray(spots[0]))
            self.perform_reaching(
                lambda through_free_space: StandAndPlaceAction.from_stands(
                    graspable,
                    self.arm,
                    poses,
                    stands,
                    retreat_distance=retreat_distance,
                    through_free_space=through_free_space,
                ),
                ParkArmsAction(self.robot.all_arms),
                retries=2,
            )
        if self.physically_simulated:
            self.let_physics_run()
        else:
            self.settle(graspable.root)

    def pick_and_place(self, name: str, spots: List[np.ndarray]) -> None:
        """
        Pick one of the task's objects up and put it down on one of a list of spots.

        :param name: The name RoboCasa gives the object.
        :param spots: See :meth:`place`.
        """
        self.pick_up(name)
        self.place(self.graspable(name), spots)

    def bounding_box(
        self, body: Body, rigidly_attached: bool = False
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        :param body: A body with collision geometry, with everything below it.
        :param rigidly_attached: Whether only what is fixed to the body is bounded,
            leaving out what hangs from a joint below it.
        :return: The lower and upper corner of the box bounding them in the world's
            root frame.
        """
        below = self.world.compute_descendent_child_kinematic_structure_entities(body)
        if rigidly_attached:
            below = [
                part
                for part in below
                if self.joint_of(part) is self.joint_of(body)
            ]
        corners = []
        for part in [body, *below]:
            if not isinstance(part, Body) or not part.has_collision():
                continue
            box = part.collision.as_bounding_box_collection_in_frame(
                self.world.root
            ).bounding_box()
            corners.append([box.min_x, box.min_y, box.min_z, box.max_x, box.max_y, box.max_z])
        corners = np.array(corners)
        return corners[:, :3].min(axis=0), corners[:, 3:].max(axis=0)

    def pose_with_underside_at(
        self, body: Body, spot: np.ndarray, clearance: float = 0.01
    ) -> Pose:
        """
        :param body: An object.
        :param spot: Where the middle of its underside is to lie.
        :param clearance: How far above the spot the underside is held when the object
            is let go of, in metres.
        :return: The pose of the object, oriented as it is now, that puts the middle of
            its underside ``clearance`` above the spot.
        """
        lower, upper = self.bounding_box(body)
        world_T_body = body.global_pose.to_np()
        underside_middle = np.array(
            [(lower[0] + upper[0]) / 2, (lower[1] + upper[1]) / 2, lower[2]]
        )
        offset = world_T_body[:3, 3] - underside_middle
        world_T_target = world_T_body.copy()
        world_T_target[:3, 3] = np.asarray(spot) + offset + np.array([0, 0, clearance])
        return HomogeneousTransformationMatrix(
            world_T_target, reference_frame=self.world.root
        ).pose

    def settle(
        self,
        body: Body,
        step: float = 0.003,
        maximum_drop: float = 0.4,
        maximum_judge_gap: float = 0.03,
    ) -> None:
        """
        Let a released object come to rest on what is below it, as a kinematic
        simulation has no gravity to do it: lower it until it touches something, so
        that RoboCasa finds the two in contact. An object touching something already
        stays where it is.

        :param body: The object.
        :param step: How far the object moves at a time, in metres.
        :param maximum_drop: How far the object is lowered at most, in metres.
        :param maximum_judge_gap: How much further the object is lowered at most for
            RoboCasa's own model to see it touch what it rests on, in metres.
        """
        connection = body.parent_connection
        if body in self.world.get_kinematic_structure_entities_of_branch(
            self.robot.root
        ):
            return
        detector = self.world.collision_manager.collision_detector
        own = {body} | {
            part
            for part in self.world.compute_descendent_child_kinematic_structure_entities(body)
            if isinstance(part, Body)
        }
        own_with_collision = [part for part in own if part.has_collision()]
        robot_bodies = set(self.robot.bodies)

        def others_near() -> List[Body]:
            lower, upper = self.bounding_box(body)
            near = []
            for other in self.world.bodies_with_collision:
                if other in own or other in robot_bodies:
                    continue
                other_lower, other_upper = self.bounding_box(other)
                if (
                    np.all(other_lower[:2] <= upper[:2] + 0.01)
                    and np.all(other_upper[:2] >= lower[:2] - 0.01)
                    and other_upper[2] >= lower[2] - maximum_drop
                    and other_lower[2] <= upper[2]
                ):
                    near.append(other)
            return near

        def touches(others: List[Body]) -> bool:
            detector.sync_world_state()
            return any(
                detector.check_collision_between_bodies(part, other, 0.0) is not None
                for part in own_with_collision
                for other in others
            )

        def shift(height: float) -> None:
            origin = connection.origin.to_np()
            origin[2, 3] += height
            connection.origin = HomogeneousTransformationMatrix(
                origin, reference_frame=connection.parent
            )

        others = others_near()
        dropped = 0.0
        while not touches(others) and dropped < maximum_drop:
            shift(-step)
            dropped += step
        # RoboCasa's own model of the kitchen collides by geometry of its own, which
        # may leave a gap where the world's touches.
        further = 0.0
        while not self.judge_sees_contact(body) and further < maximum_judge_gap:
            shift(-step)
            further += step
        logger.info(
            "Settled %s: lowered %.3f m, and %.3f m more for RoboCasa to see contact",
            body.name,
            dropped,
            further,
        )

    def spots_in_region(
        self,
        corners: Sequence[np.ndarray],
        body: Body,
        spacing: float = 0.04,
        margin: float = 0.02,
        ignored: Sequence[Body] = (),
        height_tolerance: float = 0.05,
        dropped_in: bool = False,
    ) -> List[np.ndarray]:
        """
        :param corners: A box as RoboCasa states the inside of a fixture: a corner and
            the three corners next to it along its width, its depth and its height.
        :param body: The object to be put down inside it.
        :param spacing: How far apart the spots lie, in metres.
        :param margin: How far the object stays from the sides of the box, in metres.
        :param ignored: Bodies the object may overlap, see :meth:`spot_is_free`.
        :param height_tolerance: How far the object may stick out of the top of the
            box, in metres; RoboCasa counts an object 5 cm out of a fixture as inside.
        :param dropped_in: Whether the spots lie at the top of the box rather than on
            its floor, for an object let go of above a deep fixture such as a sink,
            which it then drops to the floor of.
        :return: Spots on the floor of the box where the object fits, horizontally
            and in height, the ones nearest the middle of the box's front first.
        """
        p0, px, py, pz = (np.asarray(corner, dtype=float) for corner in corners)
        lower, upper = self.bounding_box(body)
        half_footprint = np.max(upper[:2] - lower[:2]) / 2 + margin
        height = upper[2] - lower[2]
        u, v = px - p0, py - p0
        width, depth = np.linalg.norm(u), np.linalg.norm(v)
        if (
            pz[2] - p0[2] + height_tolerance < height
            or width < 2 * half_footprint
            or depth < 2 * half_footprint
        ):
            return []
        u, v = u / width, v / depth
        floor = p0 + (pz - p0 if dropped_in else 0.0)
        spots = []
        for a in np.arange(half_footprint, width - half_footprint + 1e-9, spacing):
            for b in np.arange(half_footprint, depth - half_footprint + 1e-9, spacing):
                spots.append(floor + a * u + b * v)
        if not spots:
            return []
        front_middle = floor + u * width / 2
        spots.sort(key=lambda spot: np.linalg.norm((spot - front_middle)[:2]))
        return [
            spot for spot in spots if self.spot_is_free(spot, body, ignored=ignored)
        ]

    def spots_inside(
        self, fixture: Any, region_names: Sequence[str] | None = None, **kwargs
    ) -> List[np.ndarray]:
        """
        :param fixture: One of RoboCasa's fixtures.
        :param region_names: Which of the fixture's inside regions to use; all if not
            given.
        :param kwargs: Passed on to :meth:`spots_in_region`, with the object as
            ``body``.
        :return: Spots inside the fixture, region after region, lowest region first.
        """
        regions = fixture.get_int_sites(relative=False)
        names = [
            name
            for name in sorted(regions, key=lambda name: regions[name][0][2])
            if region_names is None or name in region_names
        ]
        ignored = self.bodies_of_fixture(fixture)
        return [
            spot
            for name in names
            for spot in self.spots_in_region(regions[name], ignored=ignored, **kwargs)
        ]

    def spots_on_counter(
        self, counter: Any, body: Body, near: np.ndarray, spacing: float = 0.05
    ) -> List[np.ndarray]:
        """
        :param counter: One of RoboCasa's counters.
        :param body: The object to be put down on it.
        :param near: The point the spots are to lie near.
        :param spacing: How far apart the spots lie, in metres.
        :return: Spots on the counter's top where the object fits clear of everything
            else standing there, nearest ``near`` first.
        """
        lower, upper = self.bounding_box(body)
        half_footprint = np.max(upper[:2] - lower[:2]) / 2 + 0.03
        rotation = np.array(counter.rot) if np.ndim(counter.rot) else counter.rot
        yaw = float(rotation) if np.ndim(rotation) == 0 else float(rotation[2])
        world_R_counter = np.array(
            [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
        )
        spots = []
        for region in counter.get_reset_regions(self.environment).values():
            center = np.asarray(counter.pos, dtype=float) + np.append(
                world_R_counter @ np.asarray(region["offset"][:2]), region["offset"][2]
            )
            half_size = np.asarray(region["size"]) / 2 - half_footprint
            if np.any(half_size <= 0):
                continue
            for a in np.arange(-half_size[0], half_size[0] + 1e-9, spacing):
                for b in np.arange(-half_size[1], half_size[1] + 1e-9, spacing):
                    spots.append(center + np.append(world_R_counter @ [a, b], 0.0))
        spots.sort(key=lambda spot: np.linalg.norm((spot - near)[:2]))
        free = []
        counter_bodies = self.bodies_of_fixture(counter)
        for spot in spots:
            if self.spot_is_free(spot, body, ignored=counter_bodies):
                free.append(spot)
            if len(free) >= 12:
                break
        return free

    def spots_on_receptacle(
        self, receptacle: Body, body: Body, share: float = 0.25
    ) -> List[np.ndarray]:
        """
        :param receptacle: An object other objects are put on or in, such as a plate,
            a bowl or a pan.
        :param body: The object to be put down on it.
        :param share: How far from its middle the spots may lie, as a share of the
            receptacle's horizontal radius.
        :return: Spots above the receptacle's rim, its middle first, from which the
            object comes to rest in it.
        """
        lower, upper = self.bounding_box(receptacle)
        middle = np.array([(lower[0] + upper[0]) / 2, (lower[1] + upper[1]) / 2, upper[2]])
        radius = share * np.min(upper[:2] - lower[:2]) / 2
        spots = [middle]
        for angle in np.linspace(0, 2 * np.pi, 6, endpoint=False):
            spots.append(middle + radius * np.array([np.cos(angle), np.sin(angle), 0.0]))
        return spots

    def spot_is_free(
        self,
        spot: np.ndarray,
        body: Body,
        clearance: float = 0.01,
        ignored: Sequence[Body] = (),
    ) -> bool:
        """
        :param spot: Where the middle of the object's underside is to lie.
        :param body: The object.
        :param clearance: How far above the spot the object is held when tested.
        :param ignored: Bodies the object may overlap, such as those of the fixture it
            is put into, whose bounding boxes enclose their inside.
        :return: Whether the object's bounding box at the spot overlaps that of no other
            body but those ignored.
        """
        lower, upper = self.bounding_box(body)
        size = upper - lower
        spot_lower = np.asarray(spot) + np.array([-size[0] / 2, -size[1] / 2, clearance])
        spot_upper = spot_lower + size
        skipped = (
            {body}
            | set(self.world.compute_descendent_child_kinematic_structure_entities(body))
            | set(ignored)
        )
        for other in self.possible_obstructions:
            if other in skipped:
                continue
            other_lower, other_upper = self.bounding_box(other)
            if np.all(other_lower < spot_upper) and np.all(other_upper > spot_lower):
                return False
        return True

    @cached_property
    def possible_obstructions(self) -> List[Body]:
        """
        The bodies that may stand where an object is to be put down: those of the
        task's objects, and of the fixtures small enough to stand on a counter, such as
        a toaster.

        Larger fixtures are left out, since a box bounding a counter, a cabinet or a
        wall covers much of the space it leaves free, and RoboCasa's regions to put
        objects in keep clear of them anyway.
        """
        object_bodies = []
        for robocasa_object in self.environment.objects.values():
            root = self.world.get_body_by_name(robocasa_object.root_body)
            object_bodies.extend(
                part
                for part in [root, *self.world.compute_descendent_child_kinematic_structure_entities(root)]
                if isinstance(part, Body) and part.has_collision()
            )
        small_fixture_bodies = []
        for fixture in self.environment.fixtures.values():
            bodies = self.bodies_of_fixture(fixture)
            if not bodies:
                continue
            corners = np.array([np.concatenate(self.bounding_box(b)) for b in bodies])
            lower, upper = corners[:, :3].min(axis=0), corners[:, 3:].max(axis=0)
            if np.prod(upper[:2] - lower[:2]) < self.small_fixture_footprint:
                small_fixture_bodies.extend(bodies)
        return object_bodies + small_fixture_bodies

    # %% judging

    @contextmanager
    def geoms_pressed(self, *geom_names: str) -> Iterator[None]:
        """
        Have RoboCasa's contact checks find its gripper touching the geoms while the
        context is entered.

        :param geom_names: The names of the geoms.
        """
        environment = self.environment
        gripper = environment.robots[0].gripper[RoboCasaRobotFrame.GRIPPER]
        check_contact = environment.check_contact

        def check_contact_with_presses(geoms_1, geoms_2=None):
            if geoms_1 is gripper and isinstance(geoms_2, str):
                if geoms_2 in self.pressed_geoms:
                    return True
            return check_contact(geoms_1, geoms_2)

        self.pressed_geoms.update(geom_names)
        environment.check_contact = check_contact_with_presses
        try:
            yield
        finally:
            environment.check_contact = check_contact
            self.pressed_geoms.difference_update(geom_names)

    def update_fixture_states(self) -> None:
        """
        Bring RoboCasa's simulation into the state of the world and have every fixture
        update its state from it, as RoboCasa does after each of its steps.
        """
        self.mirror.synchronize()
        for fixture in self.environment.fixtures.values():
            fixture.update_state(self.environment)

    def take_robocasas_robot_to_the_pr2(self) -> None:
        """
        Put the frames of RoboCasa's robot that its checks read where the PR2's
        counterparts are: the end effector site of its gripper on the PR2's tool frame,
        and its base on the PR2's base.

        Only the computed frames are moved, not the robot, so this holds until
        RoboCasa's simulation is next brought up to date.
        """
        model, data = self.environment.sim.model, self.environment.sim.data
        robot = self.environment.robots[0]
        site = robot.eef_site_id[RoboCasaRobotFrame.GRIPPER]
        data.site_xpos[site] = self.arm.end_effector.tool_frame.global_pose.to_np()[
            :3, 3
        ]
        base = model.body_name2id(RoboCasaRobotFrame.BASE)
        world_T_base = self.robot.root.global_pose.to_np()
        data.body_xpos[base] = world_T_base[:3, 3]
        data.body_xmat[base] = world_T_base[:3, :3].reshape(-1)

    def judge_sees_contact(self, body: Body) -> bool:
        """
        :param body: The root body of an object, or of a loose part of a fixture.
        :return: Whether RoboCasa's simulation, brought into the state of the world,
            has the body touch anything but RoboCasa's own robot.
        """
        self.mirror.synchronize()
        model, data = self.environment.sim.model, self.environment.sim.data
        own = model.body_name2id(body.name.name)
        robot_bodies = {
            model.body_name2id(name)
            for name in model.body_names
            if (name or "").startswith(ROBOCASA_ROBOT_BODY_PREFIXES)
        }
        for contact in data.contact[: data.ncon]:
            first = model.geom_bodyid[contact.geom1]
            second = model.geom_bodyid[contact.geom2]
            if own in (first, second) and not robot_bodies & {first, second}:
                return True
        return False

    def log_judgement(self) -> None:
        """
        Log what RoboCasa's simulation, as the judge sees it, has every object of the
        task touch, and how far the gripper stands from it.
        """
        model, data = self.environment.sim.model, self.environment.sim.data
        robot = self.environment.robots[0]
        gripper = data.site_xpos[robot.eef_site_id[RoboCasaRobotFrame.GRIPPER]]
        touching: Dict[str, set] = {}
        for contact in data.contact[: data.ncon]:
            first = model.body_id2name(model.geom_bodyid[contact.geom1])
            second = model.body_id2name(model.geom_bodyid[contact.geom2])
            touching.setdefault(first, set()).add(second)
            touching.setdefault(second, set()).add(first)
        for name, robocasa_object in self.environment.objects.items():
            position = data.body_xpos[self.environment.obj_body_id[name]]
            logger.info(
                "Judged %s at %s, %.2f m from the gripper, touching %s",
                name,
                np.round(position, 3).tolist(),
                float(np.linalg.norm(gripper - position)),
                sorted(touching.get(robocasa_object.root_body, set()))[:6],
            )

    def succeeded(self) -> bool:
        """
        :return: Whether RoboCasa's own check finds the task accomplished in the world
            as it stands.
        """
        self.update_fixture_states()
        self.take_robocasas_robot_to_the_pr2()
        return bool(self.environment._check_success())


# %% one of RoboCasa's tasks, performed by the PR2


@dataclass
class RoboCasaAtomicTask(ABC):
    """
    One of RoboCasa's atomic tasks, performed by a PR2 in a kinematic simulation with
    collision avoidance.

    A subclass is named after the RoboCasa task it performs and says, in
    :meth:`solve`, what the PR2 does.
    """

    layout: LayoutType = LayoutType.LAYOUT001
    """
    The kitchen layout.
    """

    style: StyleType = StyleType.STYLE001
    """
    The visual style of the kitchen.
    """

    seed: int | None = 0
    """
    The seed RoboCasa samples the objects and their placements with.
    """

    collision_avoidance: bool = True
    """
    Whether the PR2 avoids colliding with the kitchen and with itself while moving.
    """

    physics: bool = False
    """
    Whether the PR2 acts in a physical simulation of the kitchen in MuJoCo rather than
    a kinematic one.
    """

    video: Path | None = None
    """
    Where a video of the episode seen from the PR2's head camera is written, if
    anywhere.
    """

    episode: RoboCasaEpisode | None = field(default=None, init=False, repr=False)
    """
    The episode composed for the task, once it is.
    """

    task_parameters: ClassVar[Dict[str, Any]] = {}
    """
    Arguments of the RoboCasa task itself, by name.
    """

    @classmethod
    def robocasa_task_name(cls) -> str:
        """
        :return: The name of the RoboCasa task environment performed, the name of the
            class.
        """
        return cls.__name__

    def compose(self) -> RoboCasaEpisode:
        """
        :return: A new episode of the task.
        """
        self.episode = RoboCasaEpisode.compose(
            self.robocasa_task_name(),
            self.layout,
            self.style,
            seed=self.seed,
            task_parameters=self.task_parameters or None,
        )
        return self.episode

    @abstractmethod
    def solve(self, episode: RoboCasaEpisode) -> None:
        """
        Have the PR2 perform the task in the active execution environment.

        :param episode: The episode the task is performed in.
        """

    def run(self) -> bool:
        """
        Compose an episode, have the PR2 perform the task kinematically and judge it.

        :return: Whether RoboCasa's own check finds the task accomplished.
        """
        episode = self.compose()
        logger.info("%s: %s", self.robocasa_task_name(), episode.task.instruction)
        video = None
        if self.video is not None:
            video = HeadCameraVideo(
                episode.world, episode.robot, episode.arm.end_effector.tool_frame
            )
        try:
            with episode.executed(self.collision_avoidance, self.physics):
                self.solve(episode)
                episode.let_physics_run()
            if video is not None:
                video.stop()
            succeeded = episode.succeeded()
            if not succeeded:
                episode.log_judgement()
            return succeeded
        finally:
            # a failed episode is shown as far as it got
            if video is not None:
                video.render(self.video)


# %% kinds of tasks


@dataclass
class JointGoalTask(RoboCasaAtomicTask, ABC):
    """
    A task accomplished by moving joints of the kitchen's fixtures to positions by the
    parts hanging from them, such as opening a door by its handle or turning a knob.
    """

    @abstractmethod
    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        """
        :param episode: The episode the task is performed in.
        :return: The joints to move, by the names RoboCasa gives them, each with the
            position to move it to as the world states it, in the order they are
            moved.
        """

    def solve(self, episode: RoboCasaEpisode) -> None:
        for joint_name, position in self.joint_goals(episode):
            episode.operate(joint_name, position)


@dataclass
class DoorShareTask(JointGoalTask, ABC):
    """
    A task accomplished by moving the doors, drawers or racks of a fixture to a share of
    their travel.
    """

    share: ClassVar[float] = 1.0
    """
    The share of their travel the joints are moved to: 0 closed, 1 fully open.
    """

    @abstractmethod
    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        """
        :param episode: The episode the task is performed in.
        :return: The names RoboCasa gives the joints to move.
        """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        return [
            (name, episode.world_position_at_share(name, self.share))
            for name in self.joint_names(episode)
        ]

    def solve(self, episode: RoboCasaEpisode) -> None:
        # In a physical simulation what is shut is pushed shut, see
        # RoboCasaEpisode.push_shut.
        for joint_name, position in self.joint_goals(episode):
            shut = episode.world_position_at_share(joint_name, 0.0)
            if episode.physically_simulated and abs(position - shut) < 1e-6:
                episode.push_shut(joint_name, position)
            else:
                episode.operate(joint_name, position)


@dataclass
class PushTask(RoboCasaAtomicTask, ABC):
    """
    A task accomplished by pushing a part of a fixture with the fingertips until the
    joint it hangs from stands at a position, such as a switch or a lever.
    """

    @abstractmethod
    def pushed_joint(self, episode: RoboCasaEpisode) -> tuple[str, float]:
        """
        :param episode: The episode the task is performed in.
        :return: The name RoboCasa gives the joint pushed, and the position it is
            pushed to as the world states it.
        """

    def solve(self, episode: RoboCasaEpisode) -> None:
        episode.push(*self.pushed_joint(episode))


@dataclass
class PressTask(RoboCasaAtomicTask, ABC):
    """
    A task accomplished by pressing a button RoboCasa senses by contact alone.
    """

    @abstractmethod
    def button_geom(self, episode: RoboCasaEpisode) -> str:
        """
        :param episode: The episode the task is performed in.
        :return: The name RoboCasa gives the geom of the button.
        """

    def solve(self, episode: RoboCasaEpisode) -> None:
        episode.press(self.button_geom(episode))


def rack_joint_name(episode: RoboCasaEpisode, fixture: Any, rack_level: int) -> str:
    """
    :param episode: The episode the task is performed in.
    :param fixture: An oven or a toaster oven.
    :param rack_level: The level of the rack RoboCasa refers to, 0 at the bottom.
    :return: The name of the joint the rack or tray at that level slides on, or of the
        bottom one's if the fixture has no rack at that level, as RoboCasa falls back
        to it.
    """
    joint_names = set(episode.environment.sim.model.joint_names)
    for level in (rack_level, 0):
        for kind in ("rack", "tray"):
            name = f"{fixture.naming_prefix}{kind}{level}_joint"
            if name in joint_names:
                return name
    raise ValueError(f"{fixture.name} has no rack or tray at level {rack_level}.")


# %% running tasks from the command line


def main(module_name: str) -> None:
    """
    Run one of the tasks of a module as the command line names it, and report whether
    RoboCasa finds it accomplished.

    The module is imported under its own name rather than used as the ``__main__`` it
    runs as, since dataclasses only recognise the class variables of the tasks in a
    module they can look up by the tasks' module name.

    :param module_name: The name of the module of the tasks, holding them in
        ``TASKS``.
    """
    import argparse
    import importlib

    task_classes = importlib.import_module(module_name).TASKS
    by_name = {task_class.__name__: task_class for task_class in task_classes}
    parser = argparse.ArgumentParser(
        description="Have a PR2 perform one of RoboCasa's atomic tasks kinematically."
    )
    parser.add_argument("task", choices=sorted(by_name))
    parser.add_argument("--layout", choices=[layout.name for layout in LayoutType])
    parser.add_argument("--style", choices=[style.name for style in StyleType])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--physics",
        action="store_true",
        help="Simulate the PR2 and the kitchen physically in MuJoCo.",
    )
    parser.add_argument(
        "--video",
        type=Path,
        help="Where to write a video of the episode seen from the PR2's head camera.",
    )
    parser.add_argument(
        "--collision-avoidance",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Whether the PR2 avoids colliding with the kitchen and with itself.",
    )
    arguments = parser.parse_args()
    logging.basicConfig(level=logging.INFO, force=True)
    settings = dict(
        seed=arguments.seed,
        collision_avoidance=arguments.collision_avoidance,
        physics=arguments.physics,
        video=arguments.video,
    )
    if arguments.layout:
        settings["layout"] = LayoutType[arguments.layout]
    if arguments.style:
        settings["style"] = StyleType[arguments.style]
    task = by_name[arguments.task](**settings)
    start = time.time()
    succeeded = task.run()
    logger.info(
        "%s %s after %.0f s",
        arguments.task,
        "succeeded" if succeeded else "failed",
        time.time() - start,
    )


@dataclass
class PickAndPlaceTask(RoboCasaAtomicTask, ABC):
    """
    A task accomplished by picking one of the task's objects up and putting it down
    somewhere RoboCasa asks for.
    """

    picked_object: ClassVar[str] = "obj"
    """
    The name RoboCasa gives the object moved.
    """

    @abstractmethod
    def spots(self, episode: RoboCasaEpisode) -> List[np.ndarray]:
        """
        :param episode: The episode the task is performed in, with the object in the
            gripper.
        :return: Where the middle of the object's underside may come to lie, best
            first.
        """

    retreat_distance: ClassVar[float] = 0.1
    """
    How far above its spot the object is carried before it is lowered onto it, in
    metres.
    """

    def solve(self, episode: RoboCasaEpisode) -> None:
        episode.pick_up(self.picked_object)
        graspable = episode.graspable(self.picked_object)
        episode.place(
            graspable, self.spots(episode), retreat_distance=self.retreat_distance
        )
