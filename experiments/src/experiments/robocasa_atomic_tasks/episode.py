"""
One episode of one of RoboCasa's atomic tasks, performed by a robot: the kitchen and
objects RoboCasa composes, the robot standing where RoboCasa starts its own robot, its
motions simulated kinematically with collision avoidance, or physically in a MuJoCo
simulation of the kitchen built from the world, and RoboCasa's own success check as the
judge.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass, field
from datetime import timedelta

import mujoco
import numpy as np
from typing_extensions import TYPE_CHECKING, Any, Iterator, Self, Sequence

from coraplex.plans.context_extensions import RobotAccess, StatementGrounding
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    PhysicallySimulatedPlanExecutor,
    PlanExecutor,
    PlanOutcome,
)
from coraplex.plans.failures import EmptyUnderspecified
from coraplex.robot_plans.actions.core.robot_body import (
    MoveTorsoAction,
    ParkArmsAction,
)
from coraplex.robot_plans.plan_transformations import (
    LetContactCarryTheBodies,
    MoveStraightAroundObjects,
)
from cramph.composites import Sequence as SequenceOfSteps
from cramph.context import StatechartContext
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from experiments.robocasa_atomic_tasks.exceptions import (
    JointNotPushedToItsEnd,
    NoPlacementSpot,
    ObjectNotPickedUp,
)
from experiments.robocasa_atomic_tasks.fixtures import (
    CollisionAllowances,
    KitchenFixtures,
)
from experiments.robocasa_atomic_tasks.geometry import BodyBounds, Ray
from experiments.robocasa_atomic_tasks.grasps import KitchenGrasps
from experiments.robocasa_atomic_tasks.joints import FixtureJoints
from experiments.robocasa_atomic_tasks.judge import RoboCasaJudge
from experiments.robocasa_atomic_tasks.placement import (
    KinematicSettling,
    PlacementSpots,
)
from experiments.robocasa_atomic_tasks.skills import (
    StandAction,
    StandAndOperateMechanismAction,
    StandAndPickUpAction,
    StandAndPlaceAction,
    FingerApproach,
    FingerWithdrawAction,
    StandAndPressAction,
    StandAndPushWithAFingerAction,
    WithdrawAction,
)
from experiments.robocasa_atomic_tasks.robots import TaskRobot
from experiments.robocasa_atomic_tasks.stands import RobotClearance, StandFinder
from experiments.robocasa_task_execution.exceptions import (
    NoClearStandingPoseBehindTheStart,
)
from krrood.entity_query_language.query.match import Match
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
from semantic_digital_twin.datastructures.definitions import (
    StaticJointState,
    TorsoState,
)
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Handle,
    Lid,
)
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    RevoluteConnection,
)
from semantic_digital_twin.world_description.geometry import CoordinateBounds
from semantic_digital_twin.world_description.world_entity import Body

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

if TYPE_CHECKING:
    from robocasa.environments.kitchen.kitchen import Kitchen
    from robocasa.models.fixtures import Fixture

logger = logging.getLogger(__name__)


@dataclass
class RoboCasaEpisode:
    """
    One episode of one of RoboCasa's atomic tasks, with a robot standing where RoboCasa
    starts its own robot.
    """

    environment: Kitchen
    """
    The reset RoboCasa environment the episode was composed in.

    It holds RoboCasa's ground truth and judges the episode.
    """

    task: RoboCasaTask
    """
    The task, bound to the world the robot performs it in.
    """

    robot: AbstractRobot
    """
    The robot performing the task.
    """

    task_robot: TaskRobot
    """
    What the robot brings into the kitchen: the arm it acts with and where it stands.
    """

    handling_margin: float = 0.4
    """
    How far from an object picked up, or from where it is put down, the fixtures lie
    that the arm may come close to meanwhile, in metres.
    """

    finger_settling_time: float = 1.0
    """
    How much simulated time, in seconds, the fingers are given to close on a part in a
    physical simulation before the joint it hangs from is moved, see :attr:`~experiments
    .robocasa_atomic_tasks.skills.OperateMechanismAction.settling_time`.
    """

    simulation: MujocoSim | None = field(default=None, init=False, repr=False)
    """
    The physical simulation of the world the robot acts in, while one runs.
    """

    collision_avoidance: bool = field(default=True, init=False, repr=False)
    """
    Whether the robot avoids colliding with the kitchen and with itself while moving, as
    :meth:`executed` sets it.
    """

    bounds: BodyBounds = field(init=False, repr=False)
    """
    The bounding boxes of the world's bodies.
    """

    joints: FixtureJoints = field(init=False, repr=False)
    """
    The joints of the kitchen's fixtures.
    """

    fixtures: KitchenFixtures = field(init=False, repr=False)
    """
    The bodies of the kitchen's fixtures.
    """

    allowances: CollisionAllowances = field(init=False, repr=False)
    """
    What collision avoidance lets the robot touch for a while.
    """

    clearance: RobotClearance = field(init=False, repr=False)
    """
    How far the robot keeps from the kitchen.
    """

    stands: StandFinder = field(init=False, repr=False)
    """
    Where the robot may stand.
    """

    grasps: KitchenGrasps = field(init=False, repr=False)
    """
    The grasps the robot tries.
    """

    judge: RoboCasaJudge = field(init=False, repr=False)
    """
    RoboCasa's own simulation, judging the episode.
    """

    placement: PlacementSpots = field(init=False, repr=False)
    """
    Where objects may be put down.
    """

    settling: KinematicSettling = field(init=False, repr=False)
    """
    Lets released objects come to rest in a kinematic simulation.
    """

    def __post_init__(self) -> None:
        world = self.world
        self.bounds = BodyBounds(world)
        self.joints = FixtureJoints(environment=self.environment, world=world)
        self.fixtures = KitchenFixtures(
            environment=self.environment, world=world, bounds=self.bounds
        )
        self.allowances = CollisionAllowances(world)
        self.clearance = RobotClearance(world=world, robot=self.robot)
        self.stands = StandFinder(
            world=world,
            robot=self.robot,
            start_pose=self.task.robot_base_pose,
            clearance=self.clearance,
            candidates=self.task_robot.stand_candidates,
        )
        self.grasps = KitchenGrasps(
            bounds=self.bounds, kitchen_direction=self.stands.kitchen_direction
        )
        self.judge = RoboCasaJudge(
            environment=self.environment, world=world, robot=self.robot, arm=self.arm
        )
        self.placement = PlacementSpots(
            environment=self.environment,
            world=world,
            bounds=self.bounds,
            fixtures=self.fixtures,
        )
        self.settling = KinematicSettling(
            world=world, robot=self.robot, bounds=self.bounds, judge=self.judge
        )

    @property
    def world(self) -> World:
        """
        The world the robot performs the task in.
        """
        return self.task.world

    @property
    def arm(self) -> Arm:
        """
        The arm the robot performs the task with.
        """
        return self.task_robot.acting_arm(self.robot)

    @property
    def physically_simulated(self) -> bool:
        """
        Whether the robot acts in a physical simulation of the world.
        """
        return self.simulation is not None

    # %% composing

    @classmethod
    def compose(
        cls,
        task_name: str,
        layout: LayoutType,
        style: StyleType,
        task_robot: TaskRobot,
        seed: int | None = None,
        task_parameters: dict[str, Any] | None = None,
    ) -> Self:
        """
        Compose a new episode of a task.

        :param task_name: The name of the RoboCasa task environment.
        :param layout: The kitchen layout.
        :param style: The visual style of the kitchen.
        :param task_robot: The robot performing the task.
        :param seed: The seed RoboCasa samples the objects and their placements with.
        :param task_parameters: Arguments of the RoboCasa task itself, by name.
        :return: The episode, with the robot's arms parked and the robot standing clear
            of the kitchen at or behind RoboCasa's start pose.
        :raises NoClearStandingPoseBehindTheStart: If the robot comes too close to the
            kitchen wherever it stands near the start pose.
        """
        loader = RoboCasaDatasetLoader()
        environment = loader.build_task_environment(
            task_name, layout, style, seed=seed, task_parameters=task_parameters
        )
        task = loader.load_task_from_environment(environment)
        robot = task_robot.place_in(task.world, task.robot_base_pose)
        episode = cls(
            environment=environment, task=task, robot=robot, task_robot=task_robot
        )
        episode.stand_clear_of_the_kitchen()
        return episode

    def stand_clear_of_the_kitchen(self, maximum_distance: float = 1.0) -> None:
        """
        Let the robot's base rest on the floor, park its arms and step it back from
        RoboCasa's start pose until it keeps clear of the kitchen, see
        :meth:`StandFinder.stand_clear_of_the_start`.

        :param maximum_distance: How far back and to either side of the start pose the
            robot stands at most.
        :raises NoClearStandingPoseBehindTheStart: If the robot comes too close to the
            kitchen wherever it stands up to ``maximum_distance`` behind and beside the
            start pose.
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
        if not self.stands.stand_clear_of_the_start(maximum_distance=maximum_distance):
            raise NoClearStandingPoseBehindTheStart(
                start_pose=self.task.robot_base_pose, maximum_distance=maximum_distance
            )

    # %% running plans

    def create_executor(self, sampling_seed: int = 42) -> PlanExecutor:
        """
        :param sampling_seed: The seed the stands are sampled with.
        :return: A new executor of a plan of the robot in the episode's world, simulating
            it physically while :attr:`simulation` runs and kinematically otherwise. In
            physics the bodies are carried by the grip of the fingers alone, the hand
            moves in straight lines around what it holds or lets go of, and only what
            the robot does to the kitchen's joints moves them.
        """
        context = StatechartContext.with_extensions(
            self.world,
            [
                RobotAccess(self.robot),
                StatementGrounding(sampling_seed=sampling_seed),
            ],
        )
        if not self.physically_simulated:
            return KinematicallySimulatedPlanExecutor(
                context, collision_avoidance=self.collision_avoidance
            )
        return PhysicallySimulatedPlanExecutor(
            context,
            simulation=self.simulation,
            collision_avoidance=self.collision_avoidance,
            plan_adaptations=[LetContactCarryTheBodies(), MoveStraightAroundObjects()],
            leaves_the_environment_to_the_physics=True,
        )

    def run(self, steps: Sequence[StatechartNode | Match]) -> PlanOutcome:
        """
        :param steps: The steps of a plan, performed one after another.
        :return: How the plan ended.
        """
        executor = self.create_executor()
        statechart = Statechart(context=executor.context)
        statechart.add_node(SequenceOfSteps(nodes=list(steps)))
        executor.compile(statechart)
        return executor.attempt()

    def perform(
        self,
        plan: Callable[[], Sequence[StatechartNode | Match]],
        retries: int = 0,
    ) -> PlanOutcome:
        """
        Perform the steps of a plan one after another, freeing the robot first where it
        is too close to something, see :meth:`get_clear`, and performing them again from
        the start if they fail.

        :param plan: Makes new steps for each attempt.
        :param retries: How often the steps are performed again at most. Candidates that
            all failed their trials are not tried again unless the robot had to be
            freed.
        :return: How the last attempt ended.
        """
        outcome = PlanOutcome()
        for attempt in range(retries + 1):
            self.get_clear()
            outcome = self.run(plan())
            if outcome.succeeded or (
                isinstance(outcome.failure, EmptyUnderspecified)
                and not self.clearance.bodies_too_close()
            ):
                return outcome
            logger.warning(
                "Attempt %d failed after %s: %s",
                attempt + 1,
                type(outcome.failure).__name__,
                str(outcome.failure).splitlines()[0][:200],
            )
        return outcome

    def perform_reaching(
        self,
        plan: Callable[[bool], Sequence[StatechartNode | Match]],
        retries: int = 2,
    ) -> PlanOutcome:
        """
        Perform a plan holding a step that walks the robot to a stand and acts from
        there, first with the gripper left to the step's own motions, then, if that
        fails, with the gripper led to where the step begins along a path through free
        space.

        :param plan: Makes new steps for each attempt from whether the gripper reaches
            through free space.
        :param retries: How often the steps are performed again, each way, see
            :meth:`perform`.
        :return: How the last attempt ended.
        """
        outcome = self.perform(lambda: plan(False), retries=retries)
        if outcome.succeeded:
            return outcome
        logger.warning(
            "Reaching through free space after %s: %s",
            type(outcome.failure).__name__,
            str(outcome.failure).splitlines()[0][:200],
        )
        return self.perform(lambda: plan(True), retries=retries)

    def create_parking_step(self) -> ParkArmsAction:
        """
        :return: A new step that parks both arms.
        """
        return ParkArmsAction(arms=self.robot.all_arms)

    def get_clear(self, base_step: float = 0.1) -> bool:
        """
        Free the robot where it is closer to something than collision avoidance lets
        any motion bring it: park the arms, and step the base away from what it is too
        close to, letting just the bodies too close touch meanwhile.

        :param base_step: How far the base steps away, in metres.
        :return: Whether the robot was too close to anything.
        """
        pairs = self.clearance.bodies_too_close()
        if not pairs:
            return False
        logger.warning(
            "Getting clear of %s",
            ", ".join(
                sorted({f"{p.robot_body.name.name}-{p.other.name.name}" for p in pairs})
            ),
        )
        steps: list[StatechartNode | Match] = [self.create_parking_step()]
        base = self.robot.mobile_base.root
        base_against = [pair.other for pair in pairs if pair.robot_body is base]
        if base_against:
            world_T_base = self.robot.root.global_pose.to_np()
            away = world_T_base[:2, 3] - np.mean(
                [self.bounds.center_of(other)[:2] for other in base_against], axis=0
            )
            away /= np.linalg.norm(away)
            world_T_stand = world_T_base.copy()
            world_T_stand[:2, 3] += base_step * away
            steps.append(
                StandAction(
                    target_location=HomogeneousTransformationMatrix(
                        world_T_stand, reference_frame=self.world.root
                    ).pose
                )
            )
        with self.allowances.bodies_may_touch(
            list({pair.robot_body for pair in pairs}),
            list({pair.other for pair in pairs}),
        ):
            outcome = self.run(steps)
        if not outcome.succeeded:
            logger.warning(
                "Getting clear failed after %s: %s",
                type(outcome.failure).__name__,
                str(outcome.failure).splitlines()[0][:200],
            )
        return True

    # %% simulating

    @contextmanager
    def executed(
        self, collision_avoidance: bool = True, physics: bool = False
    ) -> Iterator[None]:
        """
        Perform the steps within the context kinematically simulated, or physically
        simulated in MuJoCo, which is started on entering and stopped on leaving.

        :param collision_avoidance: Whether the robot avoids colliding with the kitchen
            and with itself while moving.
        :param physics: Whether the steps are simulated physically.
        """
        self.collision_avoidance = collision_avoidance
        if not physics:
            yield
            return
        began = time.time()
        self.simulation = self.start_physical_simulation()
        logger.info("Physical simulation started in %.0f s", time.time() - began)
        try:
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
        Start simulating the world physically in MuJoCo, stepped by whatever performs in
        it, with friction resolved in elliptic cones.

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
        Let the physical simulation run on with the robot holding still, for what it let
        go of to come to rest; nothing happens in a kinematic simulation.

        :param duration: How much simulated time passes.
        """
        if self.physically_simulated:
            self.simulation.step_simulation(duration)

    # %% finding what RoboCasa refers to

    def object_body(self, name: str) -> Body:
        """
        :param name: The name RoboCasa gives one of the task's objects.
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
        annotation = next(
            (
                annotation
                for annotation in self.world.get_semantic_annotations_by_type(
                    HasGraspCandidates
                )
                if annotation.root is body
            ),
            None,
        )
        if annotation is not None:
            return annotation
        lid = Lid(root=body)
        with self.world.modify_world():
            self.world.add_semantic_annotation(lid)
        return lid

    def fixture_body(self, fixture: Fixture) -> Body:
        """
        :param fixture: One of RoboCasa's fixtures.
        :return: The fixture's root body.
        """
        return self.world.get_body_by_name(fixture.root_body)

    @property
    def tool_position(self) -> np.ndarray:
        """
        Where the tool frame of the acting arm is, in the world's root frame.
        """
        return self.arm.end_effector.tool_frame.global_pose.to_np()[:3, 3]

    # %% picking up and putting down

    def raise_torso_for(self, point: np.ndarray, height: float = 0.85) -> None:
        """
        Raise the torso fully before picking up or putting down an object at a counter's
        height or above; lower points are left to the torso as it stands.

        :param point: The point to be reached, in the world's root frame.
        :param height: From how high up a point counts as at a counter's height.
        """
        if point[2] < height:
            return
        self.perform(
            lambda: [MoveTorsoAction(torso_state=TorsoState.HIGH)]
        ).raise_if_failed()

    def pick_up(self, graspable: HasGraspCandidates, attempts: int = 3) -> None:
        """
        Pick an object up, standing wherever it can be reached from, and park the arms
        again with it in the gripper; in a physical simulation, pick it up again from
        where it lies while the gripper does not hold it.

        :param graspable: The annotation of the object.
        :param attempts: How often the object is picked up at most in a physical
            simulation.
        :raises ObjectNotPickedUp: If the gripper holds the object after none of the
            attempts.
        """
        for attempt in range(attempts if self.physically_simulated else 1):
            with self.allowances.gripper_may_touch(
                self.arm,
                self.fixtures.bodies_near(
                    self.bounds.of_branch(graspable.root), margin=self.handling_margin
                ),
            ):
                self.reach_and_pick_up(graspable)
            if not self.physically_simulated or self.holds(graspable):
                return
            logger.warning("%s was left behind by the pick-up", graspable.root.name)
        raise ObjectNotPickedUp(body=graspable.root)

    def reach_and_pick_up(self, graspable: HasGraspCandidates) -> None:
        """
        Pick an object up from a stand facing it, see :meth:`pick_up`.

        :param graspable: The annotation of the object.
        """
        position = graspable.root.global_pose.to_np()[:3, 3]
        self.raise_torso_for(position)
        stands = self.stands.stands_facing(position)
        grasps = self.grasps.from_above(graspable) + (
            self.grasps.approaching_along_the_kitchen(graspable.grasp_candidates())
            or graspable.grasp_candidates()
        )
        self.perform_reaching(
            lambda through_free_space: [
                self.create_parking_step(),
                StandAndPickUpAction.from_stands(
                    graspable,
                    self.arm,
                    stands,
                    grasps=grasps,
                    through_free_space=through_free_space,
                ),
                self.create_parking_step(),
            ]
        ).raise_if_failed()

    def holds(self, graspable: HasGraspCandidates, reach: float = 0.12) -> bool:
        """
        :param graspable: The annotation of an object.
        :param reach: How far from the tool frame of the acting arm the object's
            bounding box may lie at most, in metres.
        :return: Whether the acting arm's gripper holds the object: whether it hangs
            off the robot or lies between its fingers.
        """
        if graspable.root in self.world.get_kinematic_structure_entities_of_branch(
            self.robot.root
        ):
            return True
        return self.bounds.distance_to(graspable.root, self.tool_position) <= reach

    def place(
        self,
        graspable: HasGraspCandidates,
        spots: list[np.ndarray],
        retreat_distance: float = 0.1,
    ) -> None:
        """
        Put an object the acting arm holds down on one of a list of spots, standing
        wherever it can be reached from, park the arms, and let the object come to rest
        on what is below it.

        :param graspable: The annotation of the object.
        :param spots: Where the middle of the object's underside may come to lie, best
            first, in the world's root frame.
        :param retreat_distance: How far above its spot the object is carried before it
            is lowered onto it, in metres.
        :raises NoPlacementSpot: If no spot is given.
        """
        if not spots:
            raise NoPlacementSpot(body=graspable.root)
        poses = [
            self.placement.pose_with_underside_at(graspable.root, spot)
            for spot in spots
        ]
        self.raise_torso_for(np.asarray(spots[0]))
        spread = np.array(spots)
        with self.allowances.gripper_may_touch(
            self.arm,
            self.fixtures.bodies_near(
                CoordinateBounds(spread.min(axis=0), spread.max(axis=0)),
                margin=self.handling_margin,
            ),
        ):
            stands = self.stands.stands_facing(np.asarray(spots[0]))
            self.perform_reaching(
                lambda through_free_space: [
                    StandAndPlaceAction.from_stands(
                        graspable,
                        self.arm,
                        poses,
                        stands,
                        retreat_distance=retreat_distance,
                        through_free_space=through_free_space,
                    ),
                    self.create_parking_step(),
                ]
            ).raise_if_failed()
        if self.physically_simulated:
            self.let_physics_run()
        else:
            self.settling.settle(graspable.root)

    # %% moving the kitchen's joints

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

        In a physical simulation the joint moves as far as the fingers drag the part.
        Where it has fallen short, the part is taken hold of again from where it stands
        and moved on, aiming as far beyond the position as the joint fell behind last
        time.

        :param joint_name: The name RoboCasa gives the joint.
        :param world_position: The position the joint is moved to, as the world states
            it, kept within the joint's limits.
        :param threshold: How close the joint has to come to the position.
        :param attempts: How often the part is taken hold of at most in a physical
            simulation.
        :param shortfall: How far short of the position, as a share of the joint's
            travel, the joint may stop in a physical simulation before it is moved on.
        :param minimum_following: The smallest share of the way it is told to go that
            the joint is taken to go when the next attempt aims beyond the position.
        """
        joint = self.joints.connection(joint_name)
        handle = self.joints.part_moved_by(joint_name)
        goal = self.joints.within_limits(joint, world_position)
        travel = self.joints.travel_of(joint)
        # the share of the way it was told to go that the joint went last time
        followed = 1.0
        for attempt in range(attempts if self.physically_simulated else 1):
            start = joint.position
            aim = self.joints.within_limits(
                joint, start + (goal - start) / max(followed, minimum_following)
            )
            logger.info(
                "Moving %s from %.3f to %.3f, aiming at %.3f",
                joint_name,
                start,
                goal,
                aim,
            )
            with self.allowances.gripper_may_touch(
                self.arm, self.fixtures.bodies_around(handle.root)
            ):
                self.take_hold_and_move(handle, joint, aim, threshold)
            if abs(joint.position - goal) <= shortfall * travel:
                return
            logger.warning(
                "%s stopped at %.3f, short of %.3f", joint_name, joint.position, goal
            )
            if aim != start:
                followed = float(
                    np.clip((joint.position - start) / (aim - start), 0.0, 1.0)
                )

    def take_hold_and_move(
        self,
        handle: Handle,
        joint: ActiveConnection1DOF,
        goal: float,
        threshold: float,
    ) -> None:
        """
        Take hold of a part from a stand facing it and move a joint it hangs from to a
        position, see :meth:`operate`.

        :param handle: The part.
        :param joint: The joint.
        :param goal: The position the joint is moved to.
        :param threshold: How close the joint has to come to it.
        """
        stands = self.stands.stands_facing(self.bounds.center_of(handle.root))
        grasps = self.grasps.approaching_along_the_kitchen(self.grasps.on_part(handle))
        knob = self.joints.knob_turned_by(joint)
        if knob is not None:
            grasps = (
                knob.turning_grasp_candidates(self.stands.kitchen_direction) + grasps
            )
        self.perform_reaching(
            lambda through_free_space: [
                self.create_parking_step(),
                StandAndOperateMechanismAction.from_stands(
                    handle=handle,
                    arm=self.arm,
                    stands=stands,
                    grasps=grasps,
                    joint=joint,
                    goal_position=goal,
                    threshold=threshold,
                    through_free_space=through_free_space,
                    hold_base=knob is not None and self.physically_simulated,
                    settling_time=(
                        self.finger_settling_time if self.physically_simulated else 0.0
                    ),
                ),
                self.create_parking_step(),
            ]
        ).raise_if_failed()
        logger.info("%s moved to %.4f", joint.name.name, joint.position)

    def press(self, geom_name: str, approach: np.ndarray | None = None) -> None:
        """
        Press one of RoboCasa's geoms with the fingertips, such as a button, tell
        RoboCasa's fixtures about the contact while the fingertips rest on it, and
        withdraw and park the arms afterwards.

        :param geom_name: The name RoboCasa gives the geom.
        :param approach: The direction the fingertips come in along; by default levelled
            from the robot towards the geom.
        """
        self.judge.mirror.synchronize()
        if approach is None:
            approach = self.stands.approach_towards(self.geom_center(geom_name))
        point = self.geom_entry_point(geom_name, approach)
        model = self.environment.sim.model
        body_name = model.body_id2name(model.geom_bodyid[model.geom_name2id(geom_name)])
        with self.allowances.gripper_may_touch(
            self.arm,
            self.fixtures.bodies_around(self.world.get_body_by_name(body_name)),
        ):
            stands = self.stands.stands_facing(point)
            self.perform_reaching(
                lambda through_free_space: [
                    self.create_parking_step(),
                    StandAndPressAction.from_stands(
                        point=point,
                        approach=approach,
                        arm=self.arm,
                        stands=stands,
                        through_free_space=through_free_space,
                    ),
                ]
            ).raise_if_failed()
            touching = self.fingertips_touch(point)
            logger.info("Fingertips touch %s: %s", geom_name, touching)
            if touching:
                with self.judge.pressing(geom_name):
                    self.judge.update_fixture_states()
            self.withdraw(point, approach)

    def push(
        self,
        joint_name: str,
        world_position: float,
        downward_limit: float = 0.7,
    ) -> None:
        """
        Press the part hanging from one of the fixtures' joints with the fingertips and
        push it on until the joint stands at a position, such as a switch or a lever,
        then withdraw and park the arms.

        The fingertips come in along the way the joint pushes the part, or levelled from
        the robot towards the part where the joint turns it; a part pushed down is
        pressed from above and in front at once.

        :param joint_name: The name RoboCasa gives the joint.
        :param world_position: The position the joint is pushed to, as the world states
            it, kept within the joint's limits.
        :param downward_limit: How steeply down, as the vertical component of the push,
            the fingertips may come in along the push itself.
        """
        joint = self.joints.connection(joint_name)
        part = self.joints.part_moved_by(joint_name).root
        center = self.bounds.center_of(part)
        toward = self.stands.approach_towards(center)
        approach = self.joints.push_direction(joint, world_position)
        if approach is None:
            approach = toward
        elif abs(approach[2]) > downward_limit:
            approach = approach + toward
            approach = approach / np.linalg.norm(approach)
        point = Ray(point=center, direction=approach).entry_into(
            self.bounds.of_body(part)
        )
        goal = self.joints.within_limits(joint, world_position)
        logger.info("Pushing %s from %.4f to %.4f", joint_name, joint.position, goal)
        with self.allowances.gripper_may_touch(
            self.arm, self.fixtures.bodies_around(part)
        ):
            self.press_and_push(point, approach, joint, goal).raise_if_failed()

    def push_with_a_finger(
        self,
        joint_name: str,
        towards: float,
        hand_distance: float = 0.1,
        threshold_share: float = 0.02,
        reach_of_the_wrist: float = 0.5,
    ) -> None:
        """
        Open the gripper and push the part hanging from one of the fixtures' joints with
        one fingertip, set against the part's shape farthest from the joint's axis,
        until the joint stands at the end of its travel, such as a lever swung as far as
        it goes; then withdraw and park the arms. In a physical simulation the wrist may
        brush what stands around the part.

        The gripper comes in first from where its hand has the most room, and from the
        next way where the joint did not reach the end.

        :param joint_name: The name RoboCasa gives the joint, one that turns its part.
        :param towards: A position of the joint, as the world states it, on the side of
            the end it is pushed to.
        :param hand_distance: How far behind the fingertip the hand is taken to rest
            when the room around it is compared, in metres.
        :param threshold_share: How close the joint has to come to the end, as a share
            of its travel.
        :param reach_of_the_wrist: How far from the part the bodies the gripper and the
            wrist may touch may begin, in metres.
        :raises JointNotPushedToItsEnd: If every push left the joint short of the end.
        """
        joint = self.joints.connection(joint_name)
        part = self.joints.part_moved_by(joint_name).root
        end = self.joints.end_towards(joint, towards)
        surroundings = self.fixtures.bodies_around(part, margin=reach_of_the_wrist)
        wrist = [
            body
            for body in self.task_robot.wrist_bodies(self.robot)
            if body.collision.shapes
        ]
        point = self.point_for_a_fingertip(joint, part, end)
        push = self.joints.push_direction(joint, end, point=point)

        def room_for_the_hand(finger_approach: FingerApproach) -> float:
            hand = point - finger_approach.approach * hand_distance
            return min(
                self.bounds.distance_to_nearest_shape(body, hand)
                for body in surroundings
                if body is not part and body.collision.shapes
            )

        approaches = sorted(
            FingerApproach.across(push), key=room_for_the_hand, reverse=True
        )
        logger.info(
            "Pushing %s with a finger from %.4f to %.4f at %s",
            joint_name,
            joint.position,
            end,
            np.round(point, 3),
        )
        threshold = threshold_share * self.joints.travel_of(joint)
        for finger_approach in approaches:
            logger.info(
                "Coming in along %s, the fingers closing along %s",
                np.round(finger_approach.approach, 2),
                np.round(finger_approach.closing, 2),
            )
            with (
                self.allowances.gripper_may_touch(self.arm, surroundings),
                (
                    self.allowances.bodies_may_touch(wrist, surroundings)
                    if self.physically_simulated
                    else nullcontext()
                ),
            ):
                outcome = self.press_and_push(
                    self.point_for_a_fingertip(joint, part, end),
                    finger_approach.approach,
                    joint,
                    end,
                    threshold_share=threshold_share,
                    closing=finger_approach.closing,
                )
            if outcome.succeeded and abs(joint.position - end) <= threshold:
                return
            logger.warning(
                "%s stands at %.4f, not at %.4f", joint_name, joint.position, end
            )
        outcome.raise_if_failed()
        raise JointNotPushedToItsEnd(
            joint_name=joint_name, position=joint.position, end=end
        )

    def point_for_a_fingertip(
        self, joint: RevoluteConnection, part: Body, end: float
    ) -> np.ndarray:
        """
        :param joint: A joint of the kitchen that turns its part.
        :param part: The part it turns, with collision geometry.
        :param end: The position it is pushed towards.
        :return: Where a fingertip rests to push the part towards ``end`` with the most
            leverage: on the face it is pushed on of the box bounding the part's shape
            farthest from the joint's axis, level with that box's middle, in the
            world's root frame.
        """
        axis = self.joints.axis_of(joint)
        boxes = [
            box.to_array_bounds()
            for box in part.collision.as_bounding_box_collection_in_frame(
                self.world.root
            ).bounding_boxes
        ]
        farthest = max(
            boxes, key=lambda box: axis.distance_to((box.lower + box.upper) / 2.0)
        )
        center = (farthest.lower + farthest.upper) / 2.0
        push = self.joints.push_direction(joint, end, point=center)
        return Ray(point=center, direction=push).entry_into(farthest)

    def press_and_push(
        self,
        point: np.ndarray,
        approach: np.ndarray,
        joint: ActiveConnection1DOF,
        goal: float,
        threshold_share: float = 0.02,
        closing: np.ndarray | None = None,
    ) -> PlanOutcome:
        """
        Press a point from a stand facing it and push the part there on to a joint
        position, then withdraw the way the fingertips came in, from where the part took
        them, and let the fixtures update their state.

        :param point: The point pressed, in the world's root frame.
        :param approach: The direction the fingertips come in along.
        :param joint: The joint pushed.
        :param goal: The position the joint is pushed to.
        :param threshold_share: How close the joint has to come to ``goal``, as a share
            of its travel.
        :param closing: The direction the fingers of the open gripper spread along,
            square to ``approach``, to push the part with the fingertip that trails it,
            see :class:`StandAndPushWithAFingerAction`; ``None`` to press it with the
            fingertips of the closed gripper, the fingers closing level.
        :return: How the press and push ended; nothing is withdrawn if it failed.
        """
        stands = self.stands.stands_facing(point)
        part_T_world = np.linalg.inv(joint.child.global_pose.to_np())
        part_point = part_T_world[:3, :3] @ point + part_T_world[:3, 3]
        part_approach = part_T_world[:3, :3] @ approach
        part_closing = None if closing is None else part_T_world[:3, :3] @ closing
        threshold = threshold_share * self.joints.travel_of(joint)
        outcome = self.perform_reaching(
            lambda through_free_space: [
                self.create_parking_step(),
                (
                    StandAndPressAction.from_stands(
                        point=point,
                        approach=approach,
                        arm=self.arm,
                        stands=stands,
                        pushed_joint=joint,
                        goal_position=goal,
                        threshold=threshold,
                        through_free_space=through_free_space,
                    )
                    if closing is None
                    else StandAndPushWithAFingerAction.from_stands_beside(
                        point=point,
                        finger_approach=FingerApproach(
                            approach=approach, closing=closing
                        ),
                        arm=self.arm,
                        stands=stands,
                        pushed_joint=joint,
                        goal_position=goal,
                        threshold=threshold,
                        through_free_space=through_free_space,
                    )
                ),
            ]
        )
        if not outcome.succeeded:
            return outcome
        logger.info("%s pushed to %.4f", joint.name.name, joint.position)
        world_T_part = joint.child.global_pose.to_np()
        pushed_point = world_T_part[:3, :3] @ part_point + world_T_part[:3, 3]
        pushed_approach = world_T_part[:3, :3] @ part_approach
        pushed_closing = (
            None if closing is None else world_T_part[:3, :3] @ part_closing
        )
        if not self.physically_simulated:
            self.joints.snap_to_end_stop(joint, goal)
        self.judge.update_fixture_states()
        self.withdraw(pushed_point, pushed_approach, pushed_closing)
        return outcome

    def pushable(
        self, approach: np.ndarray, facing: float = 0.3, downward: float = -0.5
    ) -> bool:
        """
        :param approach: The direction fingertips would push a part in.
        :param facing: How far at least, as the cosine of the angle between them, the
            direction has to point along the one the robot faces the kitchen in.
        :param downward: The vertical component below which the direction pushes the
            part down.
        :return: Whether fingertips coming from the robot reach the side of the part
            they would push: pushing it into the kitchen, or down.
        """
        return (
            float(approach @ self.stands.kitchen_direction) > facing
            or approach[2] < downward
        )

    def push_shut(
        self,
        joint_name: str,
        world_position: float,
        attempts: int = 3,
        shortfall: float = 0.004,
        downward_limit: float = 0.7,
    ) -> None:
        """
        Push a door, lid, drawer or rack shut with the fingertips, pushing again where
        it stopped short, and take hold of it by its handle where no push gets it shut.

        The fingertips press the part, together with its handle, through the middle of
        the handle, along the way the handle moves on closing. A part whose side to push
        faces away from the robot is first taken hold of by its handle and moved most of
        the way, see :meth:`operate`, then pushed shut, and taken hold of again if that
        fails too.

        :param joint_name: The name RoboCasa gives the joint.
        :param world_position: Its closed position, as the world states it.
        :param attempts: How often the part is pushed at most.
        :param shortfall: How far short of the position, as a share of the joint's
            travel, the joint may stop.
        :param downward_limit: See :meth:`push`.
        """
        joint = self.joints.connection(joint_name)
        handle = self.joints.part_moved_by(joint_name)
        goal = self.joints.within_limits(joint, world_position)
        if self.pushable(
            self.joints.push_direction(joint, goal, self.bounds.center_of(handle.root))
        ) and self.pushed_shut(
            joint, handle, goal, attempts, shortfall, downward_limit
        ):
            return
        self.operate(joint_name, world_position)
        if self.pushed_shut(joint, handle, goal, attempts, shortfall, downward_limit):
            return
        self.operate(joint_name, world_position, shortfall=shortfall)

    def pushed_shut(
        self,
        joint: ActiveConnection1DOF,
        handle: Handle,
        goal: float,
        attempts: int,
        shortfall: float,
        downward_limit: float,
    ) -> bool:
        """
        Push a part shut with the fingertips, see :meth:`push_shut`.

        :param joint: The joint the part hangs from.
        :param handle: The part's handle.
        :param goal: The joint's closed position.
        :param attempts: How often the part is pushed at most.
        :param shortfall: How far short of ``goal``, as a share of the joint's travel,
            the joint may stop.
        :param downward_limit: See :meth:`push`.
        :return: Whether the joint ended shut; ``False`` as soon as a push fails.
        """
        tolerance = shortfall * self.joints.travel_of(joint)
        for attempt in range(attempts):
            if abs(joint.position - goal) <= tolerance:
                return True
            center = self.bounds.center_of(handle.root)
            approach = self.joints.push_direction(joint, goal, center)
            if abs(approach[2]) > downward_limit:
                approach = approach + self.stands.approach_towards(center)
                approach = approach / np.linalg.norm(approach)
            # only what moves with the part is pushed, not a button on it with a joint
            # of its own, such as the one that releases a kettle's lid
            boxes = [
                self.bounds.of_branch(joint.child, rigidly_attached=True),
                self.bounds.of_branch(handle.root, rigidly_attached=True),
            ]
            point = Ray(point=center, direction=approach).entry_into(
                CoordinateBounds(
                    np.min([box.lower for box in boxes], axis=0),
                    np.max([box.upper for box in boxes], axis=0),
                )
            )
            logger.info(
                "Pushing %s shut from %.4f to %.4f",
                joint.name.name,
                joint.position,
                goal,
            )
            with self.allowances.gripper_may_touch(
                self.arm, self.fixtures.bodies_around(handle.root)
            ):
                outcome = self.press_and_push(point, approach, joint, goal)
            if not outcome.succeeded:
                logger.warning(
                    "Pushing %s shut failed after %s: %s",
                    joint.name.name,
                    type(outcome.failure).__name__,
                    str(outcome.failure).splitlines()[0][:200],
                )
                return False
        return abs(joint.position - goal) <= tolerance

    def withdraw(
        self,
        point: np.ndarray,
        approach: np.ndarray,
        closing: np.ndarray | None = None,
    ) -> None:
        """
        Draw the gripper back from a point it pressed, park the arms and let the
        fixtures update their state.

        :param point: The point pressed.
        :param approach: The direction the fingertips came in along.
        :param closing: The direction the fingers of the open gripper spread along while
            one fingertip rested beside a part; ``None`` if the gripper was closed.
        """
        withdrawal = (
            WithdrawAction(point=point, approach=approach, arm=self.arm)
            if closing is None
            else FingerWithdrawAction(
                point=point, approach=approach, closing=closing, arm=self.arm
            )
        )
        self.perform(lambda: [withdrawal, self.create_parking_step()]).raise_if_failed()
        self.judge.update_fixture_states()

    # %% RoboCasa's geoms

    def geom_center(self, geom_name: str) -> np.ndarray:
        """
        :param geom_name: The name RoboCasa gives one of its geoms.
        :return: The geom's centre, in the world's root frame.
        """
        model, data = self.environment.sim.model, self.environment.sim.data
        return np.array(data.geom_xpos[model.geom_name2id(geom_name)])

    def geom_entry_point(self, geom_name: str, approach: np.ndarray) -> np.ndarray:
        """
        :param geom_name: The name RoboCasa gives one of its geoms.
        :param approach: The direction the geom is come at from.
        :return: Where a ray along ``approach`` through the geom's centre enters the
            box bounding the geom, in the world's root frame.
        """
        model, data = self.environment.sim.model, self.environment.sim.data
        geom = model.geom_name2id(geom_name)
        rotation = np.array(data.geom_xmat[geom]).reshape(3, 3)
        half_extents = np.array(model.geom_size[geom])
        if model.geom_type[geom] in (
            mujoco.mjtGeom.mjGEOM_SPHERE,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            mujoco.mjtGeom.mjGEOM_CYLINDER,
        ):
            radius, half_length = half_extents[0], half_extents[1] or half_extents[0]
            half_extents = np.array([radius, radius, half_length])
        local_entry = Ray(
            point=np.zeros(3), direction=rotation.T @ approach
        ).entry_into(CoordinateBounds(-half_extents, half_extents))
        return self.geom_center(geom_name) + rotation @ local_entry

    def fingertips_touch(self, point: np.ndarray, tolerance: float = 0.02) -> bool:
        """
        :param point: A point in the kitchen.
        :param tolerance: How far from the point the fingertips may stop.
        :return: Whether the tool frame of the acting arm stands on the point.
        """
        return float(np.linalg.norm(self.tool_position - point)) <= tolerance
