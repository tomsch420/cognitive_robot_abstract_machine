"""
The actions the PR2 performs RoboCasa's atomic tasks with, beyond those coraplex ships:
moving a fixture's mechanism by one of its parts to a given position, pressing and
pushing with the fingertips or with one finger of the open gripper, and standing
somewhere picked from a list of candidates.

Each comes with a counterpart that also walks the robot to a stand, picked among a
list of candidates, as is the grasp, by coraplex trying them, and then may lead the
gripper along a path through the kitchen's free space to where its last straight
approach begins.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from coraplex.datastructures.trajectory import PoseTrajectory
from coraplex.querying.grasp_closeness import GraspCloseness
from coraplex.querying.predicates import GripperHolds
from coraplex.robot_plans.actions.base import SequentialAction
from coraplex.robot_plans.actions.core.container import MovesContainerByItsHandle
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import (
    FollowToolCenterPointPathAction,
)
from coraplex.robot_plans.mixins import MovesToolCenterPoint, OperatesGripper
from cramph.composites import Parallel
from cramph.data_types import ObservationStateValues
from cramph.monitors import CountSimulationTimeSeconds
from cramph.node import CompositeNode, EndedByOwner, NodeArtifacts, StatechartNode
from experiments.robocasa_atomic_tasks.free_space import FreeSpacePathPlanner
from experiments.robocasa_atomic_tasks.geometry import ApproachFrame, BodyBounds
from giskardpy.motion_statechart.graph_node import MotionNodeArtifacts
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose, HoldPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from giskardpy.motion_statechart.weights import DefaultWeights
from krrood.entity_query_language.factories import a, flat_variable, variable
from krrood.entity_query_language.predicate import RenderedFields, SymbolicFunction
from krrood.entity_query_language.query.match import Match
from krrood.entity_query_language.verbalization.fragments.base import (
    VerbalizationFragment,
)
from krrood.entity_query_language.verbalization.vocabulary.parts_of_speech import (
    FunctionVerbalizationTemplates,
)
from krrood.ormatic.utils import classproperty
import krrood.symbolic_math.symbolic_math as sm
from krrood.symbolic_math.symbolic_math import FloatVariable, trinary_logic_and
from semantic_digital_twin.adapters.multi_sim import MujocoSynchronizer
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForEndEffector,
    AvoidCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import Arm, EndEffector
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point3,
    Pose,
)
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.geometry import CoordinateBounds
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

# %% standing somewhere


@dataclass(eq=False, repr=False, kw_only=True)
class StandAction(NavigateAction):
    """
    Takes the robot to where it is to stand, as close as a base driven by its wheels'
    velocities settles in a physical simulation.
    """

    base_pose_threshold: float = 0.03
    """
    How far the base may be from where it is to stand, in metres and in radians.
    """


# %% which grasps are tried from a stand


@dataclass(eq=False)
class GraspsClosestTo(SymbolicFunction):
    """
    The grasps of a list that are among the :attr:`number_of_grasps` closest to a
    standing position, in the order the list gives them, see
    :class:`~coraplex.querying.grasp_closeness.GraspCloseness`.
    """

    grasps: list[GraspCandidate]
    """
    The grasps, best first.
    """

    standing_position: Pose
    """
    Where the robot stands while taking one of the grasps, read when the grasps are
    chosen.
    """

    number_of_grasps: int | None = None
    """
    How many of the closest grasps are kept, all of them if None.
    """

    def __call__(self) -> list[GraspCandidate]:
        if self.number_of_grasps is None:
            return list(self.grasps)
        world = self.standing_position.reference_frame._world
        world_P_standing = world.transform(self.standing_position, world.root).to_np()[
            :, 3
        ]
        closest = sorted(
            self.grasps,
            key=lambda grasp: GraspCloseness.from_grasp(
                grasp,
                grasp.graspable.root.global_transform.to_np(),
                world_P_standing,
            ),
        )[: self.number_of_grasps]
        return [grasp for grasp in self.grasps if grasp in closest]

    @classmethod
    def _verbalization_fragment_(cls, fields: RenderedFields) -> VerbalizationFragment:
        """
        :param fields: The rendered fragment for each field.
        :return: The noun phrase *"the grasps closest to <standing position> of
            <grasps>"*.
        """
        return FunctionVerbalizationTemplates.possessive(
            cls, fields["grasps"], fields["standing_position"]
        )


# %% reaching through free space


@dataclass(eq=False, repr=False, kw_only=True)
class ReachThroughFreeSpaceAction(MovesToolCenterPoint, SequentialAction):
    """
    Leads the tool frame from where it is to a goal along a path through the free space
    around the two, turning it towards the goal's orientation on the way; straight to
    the goal where the straight way is free.
    """

    goal: Pose | None
    """
    The pose the tool frame is led to; ``None`` where :meth:`tool_goal` finds it
    only once the reach begins.
    """

    arm: Arm
    """
    The arm whose tool frame is led.
    """

    clearance: float = 0.05
    """
    How far the path keeps the tool frame from everything else, in metres.
    """

    waypoint_threshold: float = 0.03
    """
    How close the tool frame has to come to each point of the path, in metres.
    """

    surroundings_buffer: float | None = None
    """
    How far the whole robot keeps from everything else while the tool frame is led, in
    metres; ``None`` keeps the distances it usually keeps.
    """

    def tool_goal(self) -> Pose:
        """
        :return: The pose the tool frame is led to.
        """
        return self.goal

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to come close to what it reaches for,
            after the robot is let come within :attr:`surroundings_buffer` of
            everything else.
        """
        allowance = AllowCollisionForEndEffector(end_effector=self.arm.end_effector)
        if self.surroundings_buffer is None:
            return [allowance]
        robot = set(self.robot.bodies)
        return [
            AvoidCollisionBetweenGroups(
                body_group_a=list(self.robot.bodies_with_collision),
                body_group_b=[
                    body
                    for body in self.world.bodies_with_collision
                    if body not in robot
                ],
                buffer_zone_distance=self.surroundings_buffer,
            ),
            allowance,
        ]

    def create_steps(self) -> list[StatechartNode]:
        world = self.world
        goal = self.tool_goal()
        world_T_start = self.arm.end_effector.tool_frame.global_pose.to_np()
        world_T_goal = world.transform(goal, world.root).homogeneous_matrix.to_np()
        path = FreeSpacePathPlanner(
            world=world, robot=self.robot, clearance=self.clearance
        ).path(world_T_start[:3, 3], world_T_goal[:3, 3])
        onto_the_goal = self.tool_center_point_goal(goal, self.arm)
        if path is None or len(path) <= 2:
            return [onto_the_goal]
        lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        turning = Slerp(
            [0.0, 1.0],
            Rotation.from_matrix([world_T_start[:3, :3], world_T_goal[:3, :3]]),
        )
        waypoints = []
        for point, length in zip(path[1:-1], lengths[1:-1]):
            world_T_waypoint = np.eye(4)
            world_T_waypoint[:3, :3] = turning(length / lengths[-1]).as_matrix()
            world_T_waypoint[:3, 3] = point
            waypoints.append(
                HomogeneousTransformationMatrix(
                    world_T_waypoint, reference_frame=world.root
                ).pose
            )
        return [
            FollowToolCenterPointPathAction(
                target_locations=PoseTrajectory(waypoints),
                arm=self.arm,
                position_threshold=self.waypoint_threshold,
            ),
            onto_the_goal,
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class ReachAboveThePlacementAction(ReachThroughFreeSpaceAction):
    """
    Leads the tool frame, holding an object, through free space to above where a place
    puts the object down, where the place's own motions begin.
    """

    goal: Pose | None = None

    place: PlaceAction
    """
    The place whose start is reached.
    """

    def tool_goal(self) -> Pose:
        grasp = self.place._grasp_on_the_held_object()
        return self.place.grasp_pose_sequence(
            grasp.moved_to(self.place.target_location), self.arm.end_effector, grasp
        ).retreat


@dataclass(kw_only=True, eq=False)
class MayReachThroughFreeSpace:
    """
    A step that walks the robot to a stand and acts from there, and may lead the
    gripper there along a path through free space first.
    """

    through_free_space: bool = False
    """
    Whether the gripper is led along a path through free space to where the action's
    own motions begin, rather than left to them alone.
    """

    def reaching(self, reach: Match) -> list[Match]:
        """
        :param reach: The statement of the reach through free space to where the action
            begins, grounded once the reach begins, so that its path sets off from where
            the tool frame then is.
        :return: The reach, if the step reaches through free space; nothing otherwise.
        """
        return [reach] if self.through_free_space else []


# %% moving a mechanism to a position


@dataclass(eq=False, repr=False, kw_only=True)
class MeasuredJointPosition(JointPositionList):
    """
    Differs from :class:`JointPositionList` in observing its goal as reached only once
    the physics of the MuJoCo model the world is synchronized with has moved the joints
    there, while the controller still drives the world's own positions of them.
    Without such a model it observes the world's positions, as the base does.
    """

    _synchronizer: MujocoSynchronizer | None = field(
        init=False, default=None, repr=False
    )
    """
    The synchronization with the MuJoCo model the positions are measured in.
    """

    _measured_positions: dict[ActiveConnection1DOF, FloatVariable] = field(
        init=False, default_factory=dict, repr=False
    )
    """
    The variable holding each joint's position as the physics last measured it.
    """

    def set_up(self) -> None:
        self._synchronizer = next(
            iter(
                MujocoSynchronizer.all_callbacks_of_this_type_from_world(
                    self.context.world
                )
            ),
            None,
        )
        if self._synchronizer is None:
            return
        self._measured_positions = {
            connection: FloatVariable(f"{self.name}/measured {connection.name}")
            for connection, _ in self.goal_state.items()
        }
        for variable in self._measured_positions.values():
            self.context.float_variable_data.register_expression(variable)

    def build(self) -> MotionNodeArtifacts:
        """
        Differs from :meth:`JointPositionList.build` in observing the measured positions.
        """
        artifacts = super().build()
        if self._synchronizer is None:
            return artifacts
        measured_errors = [
            sm.abs(
                self.apply_limits_to_target(target, connection)
                - self._measured_positions[connection]
            )
            for connection, target in self.goal_state.items()
        ]
        artifacts.observation = self.goal_reached_at(sm.max(sm.Vector(measured_errors)))
        return artifacts

    def on_tick(self) -> ObservationStateValues | None:
        for connection, variable in self._measured_positions.items():
            self.context.float_variable_data.set_value(
                variable, self._synchronizer.simulated_position(connection)
            )
        return None


@dataclass(eq=False, repr=False, kw_only=True)
class DriveJoint(EndedByOwner, CompositeNode):
    """
    Move one joint of the environment to a position while keeping the robot's tool
    frame fixed relative to a part the joint moves, as a hand holding or pushing that
    part would.

    Unlike
    :class:`~giskardpy.motion_statechart.goals.grasped_mechanism.ArticulateGraspedMechanism`,
    the joint is named rather than taken to be the first one above the part.
    """

    tip_link: KinematicStructureEntity
    """
    The robot's tool frame.
    """

    environment_link: KinematicStructureEntity
    """
    The part the tool frame stays fixed to.
    """

    joint: ActiveConnection1DOF
    """
    The joint moved.
    """

    goal_position: float
    """
    The position the joint is moved to.
    """

    threshold: float = 0.01
    """
    How close the joint has to come to :attr:`goal_position`.
    """

    mechanism_weight: float = DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE
    """
    Weight of the goal on the joint, see
    :attr:`~giskardpy.motion_statechart.goals.grasped_mechanism.ArticulateGraspedMechanism.mechanism_weight`.
    """

    grasp_weight: float = DefaultWeights.WEIGHT_ABOVE_COLLISION_AVOIDANCE
    """
    Weight of the goal keeping the tool frame fixed to the part, see
    :attr:`~giskardpy.motion_statechart.goals.grasped_mechanism.ArticulateGraspedMechanism.grasp_weight`.
    """

    def expand(self) -> None:
        self._add_children_to_statechart(
            [
                MeasuredJointPosition(
                    name="joint goal",
                    goal_state=JointState.from_mapping(
                        {self.joint: self.goal_position}
                    ),
                    threshold=self.threshold,
                    weight=self.mechanism_weight,
                ),
                CartesianPose(
                    name="hold part",
                    root_link=self.environment_link,
                    tip_link=self.tip_link,
                    goal_pose=Pose(reference_frame=self.tip_link),
                    weight=self.grasp_weight,
                ),
            ]
        )

    def build_artifacts(self) -> NodeArtifacts:
        """
        Observe the goal reached once the joint and the hold on the part both are.
        """
        return NodeArtifacts(
            observation=trinary_logic_and(
                *[node.observation_variable for node in self.nodes]
            )
        )


@dataclass(eq=False, repr=False, kw_only=True)
class HoldsBase(MovesToolCenterPoint, ABC):
    """
    An action that may keep the robot's base where it stands while it moves the tool
    center point, even where the robot otherwise moves its whole body.
    """

    def base_hold(self) -> HoldPose:
        """
        :return: A new task keeping the robot's base at the pose it has when the task
            starts.
        """
        return HoldPose(
            name="hold base", root_link=self.world.root, tip_link=self.robot.root
        )


@dataclass(eq=False, repr=False, kw_only=True)
class DrivesAJointByAPart(HoldsBase, ABC):
    """
    An action that moves a joint of the environment by a part the gripper holds or
    pushes.
    """

    def joint_drive(
        self,
        part: Body,
        arm: Arm,
        joint: ActiveConnection1DOF,
        goal_position: float,
        threshold: float,
        hold_base: bool,
    ) -> StatechartNode:
        """
        :param part: The part the gripper holds or pushes.
        :param arm: The arm whose gripper is on the part.
        :param joint: The joint moved.
        :param goal_position: The position the joint is moved to.
        :param threshold: How close the joint has to come to ``goal_position``.
        :param hold_base: Whether the base stands still while the joint moves, as for a
            knob turned by the wrist alone.
        :return: A new goal moving the joint while the gripper stays on the part.
        """
        drive = DriveJoint(
            tip_link=arm.end_effector.tool_frame,
            environment_link=part,
            joint=joint,
            goal_position=goal_position,
            threshold=threshold,
        )
        if not hold_base:
            return drive
        return Parallel(nodes=[drive, self.base_hold()])


@dataclass(eq=False, repr=False, kw_only=True)
class OperateMechanismAction(DrivesAJointByAPart, MovesContainerByItsHandle):
    """
    Takes hold of a part of a fixture, moves a joint the part hangs from to a position
    once the fingers have closed on it, lets go of the part and backs off it.
    """

    joint: ActiveConnection1DOF | None = None
    """
    The joint moved; the first one above the part if not given.
    """

    goal_position: float = 0.0
    """
    The position the joint is moved to.
    """

    threshold: float = 0.01
    """
    How close the joint has to come to :attr:`goal_position`.
    """

    hold_base: bool = False
    """
    Whether the base stands still while the joint moves, see
    :meth:`DrivesAJointByAPart.joint_drive`.
    """

    settling_time: float = 0.0
    """
    How much simulated time, in seconds, the fingers are given to close on the part
    before the joint moves.
    """

    release_clearance: float = 0.12
    """
    The gap in metres between the part and the gripper once it has let go.
    """

    @property
    def mechanism(self) -> ActiveConnection1DOF:
        """
        Differs from :attr:`MovesContainerByItsHandle.mechanism` in being :attr:`joint`
        where one is given.
        """
        if self.joint is not None:
            return self.joint
        return super().mechanism

    def back_off_pose(self, grasp_pose: Pose, end_effector: EndEffector) -> Pose:
        """
        Differs from :meth:`MovesContainerByItsHandle.back_off_pose` in clearing the
        part where it stands once the joint is at :attr:`goal_position`, stated relative
        to what the joint hangs from.
        """
        world = self.world
        joint = self.mechanism
        back_off = super().back_off_pose(grasp_pose, end_effector)
        with world.reset_state_context():
            joint.position = self.goal_position
            world_T_back_off = world.transform(back_off, world.root)
        return world.transform(world_T_back_off, joint.parent)

    def _create_mechanism_goal(self) -> StatechartNode:
        return self.joint_drive(
            self.grasp.graspable.root,
            self.arm,
            self.mechanism,
            self.goal_position,
            self.threshold,
            self.hold_base,
        )

    def create_steps(self) -> list[StatechartNode]:
        end_effector = self.arm.end_effector
        settling = (
            [CountSimulationTimeSeconds(seconds=self.settling_time)]
            if self.settling_time > 0
            else []
        )
        return [
            self.grasping_of(self.grasp, self.arm),
            *settling,
            self._create_mechanism_goal(),
            self.gripper_goal_type.moving_gripper_to(
                end_effector.default_configuration(GripperState.OPEN)
            ),
            self.tool_center_point_goal(
                self.back_off_pose(self.grasp.grasp_pose, end_effector), self.arm
            ),
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class StandAndOperateMechanismAction(MayReachThroughFreeSpace, SequentialAction):
    """
    Walks to where the robot stands and moves the mechanism by the part.
    """

    navigate: NavigateAction
    """
    The step to where the robot stands.
    """

    operate: OperateMechanismAction
    """
    The step that moves the mechanism.
    """

    @classmethod
    def from_stands(
        cls,
        handle: Handle,
        arm: Arm,
        stands: list[Pose],
        joint: ActiveConnection1DOF,
        goal_position: float,
        grasps: list[GraspCandidate],
        threshold: float = 0.01,
        number_of_grasps: int | None = None,
        approach_clearance: float = 0.2,
        through_free_space: bool = False,
        hold_base: bool = False,
        settling_time: float = 0.0,
    ) -> Match:
        """
        :param handle: The part the mechanism is moved by.
        :param arm: The arm that moves it.
        :param stands: Where the robot may stand, best first.
        :param joint: The joint moved.
        :param goal_position: The position the joint is moved to.
        :param grasps: The grasps the part may be taken hold of by, best first.
        :param threshold: How close the joint has to come to ``goal_position``.
        :param number_of_grasps: How many of the grasps closest to a stand are tried
            from there; all of them if not given.
        :param approach_clearance: How far from the part the gripper starts its last
            straight approach onto it, in metres.
        :param through_free_space: Whether the gripper is led to the part along a
            path through free space.
        :param hold_base: See :attr:`OperateMechanismAction.hold_base`.
        :param settling_time: See :attr:`OperateMechanismAction.settling_time`.
        :return: The step, with the stand and the grasp left open.
        """
        stand = variable(Pose, domain=stands)
        return a(cls)(
            navigate=a(StandAction)(target_location=stand),
            operate=a(OperateMechanismAction)(
                grasp=flat_variable(GraspsClosestTo(grasps, stand, number_of_grasps)),
                arm=arm,
                joint=joint,
                goal_position=goal_position,
                threshold=threshold,
                approach_clearance=approach_clearance,
                hold_base=hold_base,
                settling_time=settling_time,
            ),
            through_free_space=through_free_space,
        )

    def create_steps(self) -> list[StatechartNode]:
        operate = self.operate
        pre_grasp = operate.grasp_pose_sequence(
            operate.grasp.grasp_pose, operate.arm.end_effector, operate.grasp
        ).pre_grasp
        return [
            self.navigate,
            *self.reaching(
                a(ReachThroughFreeSpaceAction)(goal=pre_grasp, arm=operate.arm)
            ),
            operate,
        ]


# %% pressing a button


@dataclass(eq=False, repr=False, kw_only=True)
class PlacesFingertips:
    """
    An action that puts an arm's fingertips on a point, coming in along a direction.
    """

    point: np.ndarray
    """
    The point the fingertips come in to, in the world's root frame.
    """

    approach: np.ndarray
    """
    The direction they come in along, in the world's root frame.
    """

    arm: Arm
    """
    The arm whose fingertips are placed.
    """

    def open_fingertip_boxes(self) -> list[CoordinateBounds]:
        """
        :return: The boxes bounding the collision shapes of the fingertips while the
            gripper is open, in the tool frame.
        """
        end_effector = self.arm.end_effector
        world = self.arm._world
        with world.reset_state_context():
            end_effector.default_configuration(GripperState.OPEN).joint_state.apply_to(
                world
            )
            return [
                box.to_array_bounds()
                for finger in end_effector.fingers
                for box in finger.tip.collision.as_bounding_box_collection_in_frame(
                    end_effector.tool_frame
                ).bounding_boxes
            ]

    def fingertips_goal(
        self, distance_before_the_point: float, degenerate_closing: float = 1e-6
    ) -> Pose:
        """
        :param distance_before_the_point: How far back along the approach the
            fingertips stay from the point, in metres.
        :param degenerate_closing: How short a closing direction across a vertical
            approach counts as none, in which case the fingers close along the world's
            y-axis.
        :return: The tool frame goal that puts the fingertips there, with the fingers
            closing level across the approach.
        """
        approach = self.approach / np.linalg.norm(self.approach)
        closing = np.cross([0.0, 0.0, 1.0], approach)
        if np.linalg.norm(closing) < degenerate_closing:
            closing = np.array([0.0, 1.0, 0.0])
        world_T_press = ApproachFrame(
            position=self.point - approach * distance_before_the_point,
            approach=approach,
            closing=closing,
        ).in_reference_frame(self.arm._world.root)
        return self.arm.end_effector.tool_frame_goal_for_grasp(world_T_press.pose)


@dataclass(eq=False, repr=False, kw_only=True)
class PressAction(
    PlacesFingertips, MovesToolCenterPoint, OperatesGripper, SequentialAction
):
    """
    Brings the gripper into :attr:`gripper_state` and pushes its fingertips against a
    point, coming in along a direction; the gripper stays on the point.
    """

    approach_distance: float = 0.25
    """
    How far before the point the straight push onto it starts, in metres.
    """

    @classproperty
    def gripper_state(cls) -> GripperState:
        """
        The state the gripper is in while it presses: closed.
        """
        return GripperState.CLOSE

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to touch what it presses.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
        return [
            self.gripper_goal_type.moving_gripper_to(
                self.arm.end_effector.default_configuration(self.gripper_state)
            ),
            self.tool_center_point_goal(
                self.fingertips_goal(self.approach_distance), self.arm
            ),
            self.tool_center_point_goal(
                self.fingertips_goal(0.0), self.arm, in_a_straight_line=True
            ),
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class PushAction(DrivesAJointByAPart, SequentialAction):
    """
    Presses a part of a fixture with the fingertips and pushes it on, moving the joint
    it hangs from to a position, the way a switch or a lever is pushed down.
    """

    press: PressAction
    """
    The press that puts the fingertips on the part.
    """

    part: Body
    """
    The part pushed.
    """

    joint: ActiveConnection1DOF
    """
    The joint the push moves.
    """

    goal_position: float
    """
    The position the joint is pushed to.
    """

    threshold: float = 0.0005
    """
    How close the joint has to come to :attr:`goal_position`.
    """

    hold_base: bool = False
    """
    Whether the base stands still while the joint moves, see
    :meth:`DrivesAJointByAPart.joint_drive`.
    """

    def create_steps(self) -> list[StatechartNode]:
        return [
            self.press,
            self.joint_drive(
                self.part,
                self.press.arm,
                self.joint,
                self.goal_position,
                self.threshold,
                hold_base=self.hold_base,
            ),
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class WithdrawAction(PlacesFingertips, HoldsBase, SequentialAction):
    """
    Draws the gripper straight back from a point it pressed, the way it came in.
    """

    distance: float = PressAction.approach_distance
    """
    How far back from the point the gripper withdraws, in metres.
    """

    hold_base: bool = False
    """
    Whether the base stands still while the gripper withdraws, as after a push that held
    it, so that the base does not drive into what it stands close to.
    """

    def goals_held_while_withdrawing(self) -> tuple[HoldPose, ...]:
        """
        :return: The tasks held while the gripper moves away from the point: the hold
            on the base if :attr:`hold_base`, none otherwise.
        """
        return (self.base_hold(),) if self.hold_base else ()

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to touch what it pressed while it leaves.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
        return [
            self.tool_center_point_goal(
                self.fingertips_goal(self.distance),
                self.arm,
                in_a_straight_line=True,
                accompanying_goals=self.goals_held_while_withdrawing(),
            )
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class StandAndPressAction(MayReachThroughFreeSpace, OperatesGripper, SequentialAction):
    """
    Walks to where the robot stands, brings the gripper into the press's
    :attr:`PressAction.gripper_state` and presses the point.
    """

    navigate: NavigateAction
    """
    The step to where the robot stands.
    """

    press: PressAction | PushAction
    """
    The step that presses the point, and pushes on if it is a push.
    """

    @classmethod
    def from_stands(
        cls,
        point: np.ndarray,
        approach: np.ndarray,
        arm: Arm,
        stands: list[Pose],
        pushed_joint: ActiveConnection1DOF | None = None,
        goal_position: float = 0.0,
        threshold: float = 0.0005,
        through_free_space: bool = False,
    ) -> Match:
        """
        :param point: The point pressed, in the world's root frame.
        :param approach: The direction the fingertips come in along.
        :param arm: The arm that presses.
        :param stands: Where the robot may stand, best first.
        :param pushed_joint: The joint the press pushes on to ``goal_position``, the
            one the pressed part hangs from; ``None`` for a press alone.
        :param goal_position: The position ``pushed_joint`` is pushed to.
        :param threshold: How close ``pushed_joint`` has to come to ``goal_position``.
        :param through_free_space: Whether the gripper is led to where the press
            begins along a path through free space.
        :return: The step, with the stand left open.
        """
        press = a(PressAction)(point=point, approach=approach, arm=arm)
        return a(cls)(
            navigate=a(StandAction)(target_location=variable(Pose, domain=stands)),
            press=(
                press
                if pushed_joint is None
                else a(PushAction)(
                    press=press,
                    part=pushed_joint.child,
                    joint=pushed_joint,
                    goal_position=goal_position,
                    threshold=threshold,
                )
            ),
            through_free_space=through_free_space,
        )

    def create_steps(self) -> list[StatechartNode]:
        press = self.press if isinstance(self.press, PressAction) else self.press.press
        return [
            self.navigate,
            self.gripper_goal_type.moving_gripper_to(
                press.arm.end_effector.default_configuration(press.gripper_state)
            ),
            *self.reaching(
                a(ReachThroughFreeSpaceAction)(
                    goal=press.fingertips_goal(press.approach_distance), arm=press.arm
                )
            ),
            self.press,
        ]


# %% pushing a part with one finger of the open gripper


@dataclass
class FingerApproach:
    """
    How the open gripper comes in beside a part to push it with one fingertip, the way
    a finger nudges a lever.
    """

    approach: np.ndarray
    """
    The direction the fingertips come in along, across the push, in the world's root
    frame.
    """

    closing: np.ndarray
    """
    The direction the fingers close along, the direction the part is pushed in, square
    to :attr:`approach`.
    """

    @classmethod
    def across(
        cls, push: np.ndarray, tilts: tuple[float, ...] = (0.0, 30.0, 60.0, 90.0)
    ) -> list[FingerApproach]:
        """
        :param push: The direction the part is pushed in, in the world's root frame.
        :param tilts: How far below level the fingertips may come in, in degrees,
            best first; 90 comes in from straight above.
        :return: The ways to come in across the push from either side, tilted down by
            each of ``tilts``, the fingers closing along the push.
        """
        up = np.array([0.0, 0.0, 1.0])
        push = push / np.linalg.norm(push)
        across = np.cross(up, push)
        across = across / np.linalg.norm(across)
        downward = np.cross(across, push)
        downward = downward if downward @ up < 0.0 else -downward
        approaches = []
        for tilt in np.radians(tilts):
            for side in (1.0, -1.0) if np.cos(tilt) > 1e-9 else (1.0,):
                approach = side * across * np.cos(tilt) + downward * np.sin(tilt)
                approaches.append(cls(approach=approach, closing=push))
        return approaches


@dataclass(eq=False, repr=False, kw_only=True)
class PlacesOneFingertip(PlacesFingertips):
    """
    An action that puts the inner face of one fingertip of the open gripper on a point,
    coming in along a direction with the fingers spread along another; the other
    fingertip lies on the far side of the point.
    """

    closing: np.ndarray
    """
    The direction the fingers close along, in the world's root frame, square to
    :attr:`approach`; the fingertip that trails it is put on the point.
    """

    def fingertips_goal(
        self, distance_before_the_point: float, degenerate_closing: float = 1e-6
    ) -> Pose:
        """
        Differs from :meth:`PlacesFingertips.fingertips_goal` in closing the fingers
        along :attr:`closing` and putting the inner face of the fingertip that trails it
        on the point, rather than the middle between the fingertips;
        ``degenerate_closing`` is not used.
        """
        approach = self.approach / np.linalg.norm(self.approach)
        world_T_press = ApproachFrame(
            position=self.point - approach * distance_before_the_point,
            approach=approach,
            closing=self.closing,
        ).in_reference_frame(self.arm._world.root)
        end_effector = self.arm.end_effector
        return end_effector.tool_frame_goal_for_point(
            self.trailing_fingertip_face(), world_T_press.pose
        )

    def trailing_fingertip_face(self) -> Point3:
        """
        :return: The middle of the inner face of the fingertip that lies against the
            closing axis while the gripper is open, in the tool frame.
        """
        end_effector = self.arm.end_effector
        closing = end_effector.closing_axis.to_np()[:3]
        trailing = min(
            self.open_fingertip_boxes(),
            key=lambda bounds: (bounds.lower + bounds.upper) @ closing,
        )
        center = (trailing.lower + trailing.upper) / 2.0
        half_extent = (trailing.upper - trailing.lower) / 2.0
        face = center + closing * (np.abs(closing) @ half_extent)
        return Point3(*face, reference_frame=end_effector.tool_frame)


@dataclass(eq=False, repr=False, kw_only=True)
class FingerPressAction(PlacesOneFingertip, PressAction):
    """
    Opens the gripper and brings one fingertip onto a point beside a part, the fingers
    spread along the way the part is pushed; the gripper stays there.
    """

    @classproperty
    def gripper_state(cls) -> GripperState:
        """
        The state the gripper is in while it presses: open.
        """
        return GripperState.OPEN


@dataclass(eq=False, repr=False, kw_only=True)
class FingerWithdrawAction(PlacesOneFingertip, WithdrawAction):
    """
    Draws the open gripper straight back from a point it was brought to by a
    :class:`FingerPressAction`, the way it came in, then straight up.
    """

    rise: float = 0.15
    """
    How far the gripper rises straight up once drawn back, in metres, so that parking
    the arm sets off above what stands around the pushed part.
    """

    def create_steps(self) -> list[StatechartNode]:
        """
        Differs from :meth:`WithdrawAction.create_steps` in rising by :attr:`rise` once
        drawn back.
        """
        world = self.world
        world_T_risen = world.transform(
            self.fingertips_goal(self.distance), world.root
        ).homogeneous_matrix.to_np()
        world_T_risen[2, 3] += self.rise
        return [
            *super().create_steps(),
            self.tool_center_point_goal(
                HomogeneousTransformationMatrix(
                    world_T_risen, reference_frame=world.root
                ).pose,
                self.arm,
                in_a_straight_line=True,
                accompanying_goals=self.goals_held_while_withdrawing(),
            ),
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class StandAndPushWithAFingerAction(StandAndPressAction):
    """
    Walks to where the robot stands, opens the gripper, brings one fingertip beside a
    part and pushes the part on with it, moving the joint it hangs from to a position.
    """

    @classmethod
    def from_stands_beside(
        cls,
        point: np.ndarray,
        finger_approach: FingerApproach,
        arm: Arm,
        stands: list[Pose],
        pushed_joint: ActiveConnection1DOF,
        goal_position: float,
        threshold: float,
        through_free_space: bool,
    ) -> Match:
        """
        :param point: Where the fingertip comes to rest beside the part, in the world's
            root frame.
        :param finger_approach: How the gripper comes in.
        :param arm: The arm that pushes.
        :param stands: Where the robot may stand, best first.
        :param pushed_joint: The joint the part hangs from.
        :param goal_position: The position ``pushed_joint`` is pushed to.
        :param threshold: How close ``pushed_joint`` has to come to ``goal_position``.
        :param through_free_space: Whether the gripper is led to where it comes in from
            along a path through free space.
        :return: The step, with the stand left open.
        """
        return a(cls)(
            navigate=a(StandAction)(target_location=variable(Pose, domain=stands)),
            press=a(PushAction)(
                press=a(FingerPressAction)(
                    point=point,
                    approach=finger_approach.approach,
                    closing=finger_approach.closing,
                    arm=arm,
                ),
                part=pushed_joint.child,
                joint=pushed_joint,
                goal_position=goal_position,
                threshold=threshold,
            ),
            through_free_space=through_free_space,
        )


# %% picking an object up and putting it down


@dataclass(eq=False, repr=False, kw_only=True)
class PlaceHeldObjectAction(PlaceAction):
    """
    Puts down an object an arm is known to hold.

    Differs from :class:`~coraplex.robot_plans.actions.core.placing.PlaceAction` in
    taking an object that lies in :attr:`holding_arm`'s gripper as held by it, whether
    or not the rays between the fingers hit it.
    """

    holding_arm: Arm | None = None
    """
    The arm that holds the object.
    """

    reach: float = 0.12
    """
    How far from the holding arm's tool frame the object's bounding box may lie at most
    for the object to lie in its gripper, in metres.
    """

    def _holding_arm_and_grasp(self) -> tuple[Arm, GraspCandidate]:
        """
        Differs from :meth:`PlaceAction._holding_arm_and_grasp` in reading the grasp off
        :attr:`holding_arm`'s gripper wherever the object lies in it.
        """
        arm = self.holding_arm
        body = self.object_designator.root
        if arm is None or not self.lies_in_the_gripper_of(arm):
            return super()._holding_arm_and_grasp()
        end_effector = arm.end_effector
        if GripperHolds(end_effector, body)():
            return arm, GraspCandidate(
                self.object_designator, end_effector.held_body_T_grasp
            )
        return arm, GraspCandidate(
            self.object_designator, end_effector.body_T_grasp(body)
        )

    def lies_in_the_gripper_of(self, arm: Arm) -> bool:
        """
        :param arm: An arm.
        :return: Whether the object lies within :attr:`reach` of the arm's tool frame.
        """
        tool = arm.end_effector.tool_frame.global_pose.to_np()[:3, 3]
        return (
            BodyBounds(self.world).distance_to(self.object_designator.root, tool)
            <= self.reach
        )


@dataclass(eq=False, repr=False, kw_only=True)
class StandAndPickUpAction(MayReachThroughFreeSpace, SequentialAction):
    """
    Walks to where the robot stands and picks the object up.
    """

    navigate: NavigateAction
    """
    The step to where the robot stands.
    """

    pick_up: PickUpAction
    """
    The step that picks the object up.
    """

    @classmethod
    def from_stands(
        cls,
        graspable: HasGraspCandidates,
        arm: Arm,
        stands: list[Pose],
        grasps: list[GraspCandidate],
        number_of_grasps: int | None = None,
        through_free_space: bool = False,
    ) -> Match:
        """
        :param graspable: The object to pick up.
        :param arm: The arm to pick it up with.
        :param stands: Where the robot may stand, best first.
        :param grasps: The grasps the object may be taken by, best first.
        :param number_of_grasps: How many of the grasps closest to a stand are tried
            from there; all of them if not given.
        :param through_free_space: Whether the gripper is led to the object along a
            path through free space.
        :return: The step, with the stand and the grasp left open.
        """
        stand = variable(Pose, domain=stands)
        return a(cls)(
            navigate=a(StandAction)(target_location=stand),
            pick_up=a(PickUpAction)(
                grasp=flat_variable(GraspsClosestTo(grasps, stand, number_of_grasps)),
                arm=arm,
            ),
            through_free_space=through_free_space,
        )

    def create_steps(self) -> list[StatechartNode]:
        pick_up = self.pick_up
        pre_grasp = pick_up.grasp_pose_sequence(
            pick_up.grasp.grasp_pose, pick_up.arm.end_effector, pick_up.grasp
        ).pre_grasp
        return [
            self.navigate,
            *self.reaching(
                a(ReachThroughFreeSpaceAction)(goal=pre_grasp, arm=pick_up.arm)
            ),
            pick_up,
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class StandAndPlaceAction(MayReachThroughFreeSpace, SequentialAction):
    """
    Walks to where the robot stands and puts the object it holds down.
    """

    navigate: NavigateAction
    """
    The step to where the robot stands.
    """

    place: PlaceAction
    """
    The step that puts the object down.
    """

    arm: Arm
    """
    The arm that holds the object.
    """

    @classmethod
    def from_stands(
        cls,
        graspable: HasGraspCandidates,
        arm: Arm,
        target_poses: list[Pose],
        stands: list[Pose],
        retreat_distance: float = 0.1,
        through_free_space: bool = False,
    ) -> Match:
        """
        :param graspable: The object the robot holds.
        :param arm: The arm that holds it.
        :param target_poses: Where the object may be put down, best first.
        :param stands: Where the robot may stand, best first.
        :param retreat_distance: How far above where it is put down the object is
            carried before it is lowered there, and the gripper rises after letting go.
        :param through_free_space: Whether the object is carried to above where it is
            put down along a path through free space.
        :return: The step, with the stand and the pose left open.
        """
        return a(cls)(
            navigate=a(StandAction)(target_location=variable(Pose, domain=stands)),
            place=a(PlaceHeldObjectAction)(
                object_designator=graspable,
                target_location=variable(Pose, domain=target_poses),
                retreat_distance=retreat_distance,
                holding_arm=arm,
            ),
            arm=arm,
            through_free_space=through_free_space,
        )

    def create_steps(self) -> list[StatechartNode]:
        return [
            self.navigate,
            *self.reaching(
                a(ReachAboveThePlacementAction)(arm=self.arm, place=self.place)
            ),
            self.place,
        ]
