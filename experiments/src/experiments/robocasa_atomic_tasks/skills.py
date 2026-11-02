"""
The actions the PR2 performs RoboCasa's atomic tasks with, beyond those coraplex ships:
moving a fixture's mechanism by one of its parts to a given position, and pressing a
button.

Each comes with a counterpart that also walks the robot to a stand, picked among a
list of candidates, as is the grasp, by coraplex trying them, and then leads the
gripper along a path through the kitchen's free space to where its last straight
approach begins.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from typing_extensions import List, Sequence

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.graph_node import Goal, NodeArtifacts
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose, HoldPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from krrood.symbolic_math.symbolic_math import trinary_logic_and
from krrood.entity_query_language.factories import a, variable
from krrood.entity_query_language.query.match import Match
from coraplex.datastructures.enums import MovementType
from coraplex.plans.factories import sequential
from coraplex.plans.plan_node import PlanNode
from coraplex.querying.predicates import IsAmongTheClosestGraspsTo
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.container import ContainerAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.datastructures.enums import ExecutionType
from coraplex.exceptions import ObjectIsNotHeld
from coraplex.plans.factories import execute_single
from coraplex.plans.executables import Executable, GiskardExecutable
from coraplex.plans.plan_node import ExecutionBoundaryNode
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.robot_plans.motions.navigation import MoveMotion
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveTCPWaypointsMotion,
    MoveToolCenterPointMotion,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.exceptions import NothingHeld
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

from experiments.robocasa_atomic_tasks.free_space import path_through_free_space

# %% frames


def frame_at(
    position: np.ndarray, x_axis: np.ndarray, y_axis: np.ndarray, reference_frame: Body
) -> HomogeneousTransformationMatrix:
    """
    :param position: The origin of the frame.
    :param x_axis: The direction its x-axis points in.
    :param y_axis: The direction its y-axis points in, square to ``x_axis``.
    :param reference_frame: The frame all three are given in.
    :return: The frame.
    """
    x_axis = np.asarray(x_axis, dtype=float) / np.linalg.norm(x_axis)
    y_axis = np.asarray(y_axis, dtype=float) / np.linalg.norm(y_axis)
    matrix = np.eye(4)
    matrix[:3, 0] = x_axis
    matrix[:3, 1] = y_axis
    matrix[:3, 2] = np.cross(x_axis, y_axis)
    matrix[:3, 3] = position
    return HomogeneousTransformationMatrix(matrix, reference_frame=reference_frame)


# %% standing somewhere


@dataclass
class DriveToStandMotion(MoveMotion):
    """
    Drives the base to where the robot is to stand, as close as
    :class:`~coraplex.robot_plans.actions.core.navigation.NavigateAction` asks it to
    come, rather than as close as a pose goal does by default.

    A base driven by its wheels' velocities in a physical simulation settles a centimetre
    or two beside its goal and holds there; a pose goal that waits for it to come nearer
    never ends.
    """

    position_threshold: float = 0.03
    """
    How close the base has to come to where it is to stand, in metres.
    """

    orientation_threshold: float = 0.03
    """
    How close it has to come to the way it is to face, in radians.
    """

    @property
    def _motion_chart(self):
        if GiskardExecutable.execution_type == ExecutionType.KINEMATICALLY_SIMULATED:
            return super()._motion_chart
        return CartesianPose(
            root_link=self.world.root,
            tip_link=self.robot.root,
            goal_pose=self.target,
            translation_threshold=self.position_threshold,
            orientation_threshold=self.orientation_threshold,
        )


@dataclass
class StandAction(NavigateAction):
    """
    Takes the robot to where it is to stand, see :class:`DriveToStandMotion`.
    """

    @property
    def _action_plan(self) -> PlanNode:
        return execute_single(
            DriveToStandMotion(self.robot.mobile_base.pose_facing(self.target_location))
        )


# %% reaching through free space


@dataclass
class ReachThroughFreeSpaceAction(ActionDescription):
    """
    Leads the tool frame from where it is to a goal along a path through the free space
    around the two, planned on a graph of convex sets, turning it towards the goal's
    orientation on the way.

    Collision avoidance alone pulls the gripper straight towards its goal and stops it
    in front of whatever stands in the way; the path leads it around. Where the
    straight way is free, the tool frame moves straight to the goal.
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

    def tool_goal(self) -> Pose:
        """
        :return: The pose the tool frame is led to.
        """
        return self.goal

    @property
    def _action_plan(self) -> PlanNode:
        world = self.world
        goal = self.tool_goal()
        world_T_start = self.arm.end_effector.tool_frame.global_pose.to_np()
        world_T_goal = world.transform(goal, world.root).homogeneous_matrix.to_np()
        path = path_through_free_space(
            world,
            self.robot,
            world_T_start[:3, 3],
            world_T_goal[:3, 3],
            clearance=self.clearance,
        )
        # the last motion onto the goal is the one the action reached for afterwards
        # begins with, which lets the gripper come close to what it reaches for
        onto_the_goal = MoveToolCenterPointMotion(
            goal, self.arm, allow_gripper_collision=True
        )
        if path is None or len(path) <= 2:
            return sequential([onto_the_goal])
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
        return sequential(
            [
                MoveTCPWaypointsMotion(
                    waypoints,
                    self.arm,
                    position_threshold=self.waypoint_threshold,
                ),
                onto_the_goal,
            ]
        )


@dataclass
class ReachAboveThePlacementAction(ReachThroughFreeSpaceAction):
    """
    Leads the tool frame, holding an object, through free space to above where a place
    puts the object down, where the place's own motions begin.
    """

    place: PlaceAction | None = None
    """
    The place whose start is reached.
    """

    def tool_goal(self) -> Pose:
        end_effector = self.arm.end_effector
        grasp = grasp_held_by(self.arm, self.place.object_designator)
        return self.place.grasp_pose_sequence(
            grasp.moved_to(self.place.target_location), end_effector, grasp
        ).retreat


@dataclass
class MayReachThroughFreeSpace:
    """
    A step that walks the robot to a stand and acts from there, and may lead the
    gripper there along a path through free space first.
    """

    through_free_space: bool = field(default=False, kw_only=True)
    """
    Whether the gripper is led along a path through free space to where the action's
    own motions begin, rather than left to them alone.
    """

    def reaching(self, reach: ReachThroughFreeSpaceAction) -> List[ActionDescription]:
        """
        :param reach: The reach through free space to where the action begins.
        :return: The reach, if the step reaches through free space; nothing otherwise.
        """
        return [reach] if self.through_free_space else []


# %% grasping a fixture's part


def part_grasps(
    handle: Handle,
    depth: float = 0.02,
    upward_slope: float = 0.8,
    lower_end_height: float = 0.04,
) -> List[GraspCandidate]:
    """
    The grasps a part of a fixture is taken hold of by, such as a handle, a knob, a
    lever or the front edge of a rack.

    The gripper approaches the part horizontally from each of the four sides of the
    kitchen, or from above, and its fingers close across the thinner of the part's two
    extents square to the approach. The grasp lies on the face the gripper comes in
    through, no deeper into the part than ``depth``, so that a part as large as a rack
    is taken by its edge.

    The gripper also comes in from below each side, rising towards the part's lower
    end with its fingers closing level, which is how an arm whose shoulder sits below
    a wall cabinet's handle reaches it without its forearm running into the cabinet.

    :param handle: The annotation of the part.
    :param depth: How far behind the face the gripper comes in through the grasp lies
        at most, in metres.
    :param upward_slope: How steeply the gripper rises when coming in from below, as
        the rise per unit of horizontal travel.
    :param lower_end_height: How far above its lowest point a part is taken when
        coming in from below, in metres, at most half its height.
    :return: The grasps, the horizontal ones first, then those from above, then those
        from below.
    """
    body = handle.root
    world = body._world
    box = body.collision.as_bounding_box_collection_in_frame(world.root).bounding_box()
    center = np.array(
        [
            (box.min_x + box.max_x) / 2,
            (box.min_y + box.max_y) / 2,
            (box.min_z + box.max_z) / 2,
        ]
    )
    extents = np.array(
        [box.max_x - box.min_x, box.max_y - box.min_y, box.max_z - box.min_z]
    )
    approaches = [
        np.array([1.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, -1.0, 0.0]),
        np.array([0.0, 0.0, -1.0]),
    ]
    grasps = []
    for approach in approaches:
        along = int(np.argmax(np.abs(approach)))
        across = [axis for axis in range(3) if axis != along]
        thinner, wider = sorted(across, key=lambda axis: extents[axis])
        half_depth = extents[along] / 2
        position = center - approach * (half_depth - min(half_depth, depth))
        for closing_axis in (thinner, wider):
            closing = np.zeros(3)
            closing[closing_axis] = 1.0
            world_T_grasp = frame_at(position, approach, closing, world.root)
            grasps.append(
                GraspCandidate(handle, world.transform(world_T_grasp.pose, body))
            )
    for horizontal in approaches[:4]:
        approach = horizontal + np.array([0.0, 0.0, upward_slope])
        approach /= np.linalg.norm(approach)
        closing = np.cross([0.0, 0.0, 1.0], horizontal)
        position = center.copy()
        position[2] = box.min_z + min(extents[2] / 2, lower_end_height)
        world_T_grasp = frame_at(position, approach, closing, world.root)
        grasps.append(GraspCandidate(handle, world.transform(world_T_grasp.pose, body)))
    return grasps


def grasps_from_above(
    graspable: HasGraspCandidates,
    direction: np.ndarray,
    depth: float = 0.03,
    pitches: Sequence[float] = (math.pi / 4, math.pi / 2),
) -> List[GraspCandidate]:
    """
    Grasps that take an object from above: coming down onto its top along the
    direction the robot faces the kitchen in, pitched down by each of a list of angles,
    with the fingers closing level across the approach, once along each of the
    object's horizontal extents.

    Coming down onto an object keeps the wrist over the worktop; reaching in level
    with it, the forearm runs along the counter's edge.

    :param graspable: The object.
    :param direction: The horizontal direction the robot faces the kitchen in.
    :param depth: How far below the top of the object the grasp lies at most, in
        metres.
    :param pitches: How steeply the gripper comes down, in radians below the
        horizontal.
    :return: The grasps, the least steep first.
    """
    body = graspable.root
    world = body._world
    box = body.collision.as_bounding_box_collection_in_frame(world.root).bounding_box()
    height = box.max_z - box.min_z
    position = np.array(
        [
            (box.min_x + box.max_x) / 2,
            (box.min_y + box.max_y) / 2,
            box.max_z - min(depth, height / 2),
        ]
    )
    direction = np.asarray(direction, dtype=float)
    direction = np.append(direction[:2] / np.linalg.norm(direction[:2]), 0.0)
    grasps = []
    for pitch in pitches:
        approach = math.cos(pitch) * direction + np.array([0.0, 0.0, -math.sin(pitch)])
        for closing in (np.cross([0.0, 0.0, 1.0], direction), direction):
            closing = closing - (closing @ approach) * approach
            if np.linalg.norm(closing) < 1e-6:
                continue
            world_T_grasp = frame_at(position, approach, closing, world.root)
            grasps.append(
                GraspCandidate(graspable, world.transform(world_T_grasp.pose, body))
            )
    return grasps


def grasps_approaching_along(
    grasps: List[GraspCandidate], direction: np.ndarray, minimum_alignment: float = 0.5
) -> List[GraspCandidate]:
    """
    :param grasps: Grasps on a part.
    :param direction: The horizontal direction the robot faces the kitchen in.
    :param minimum_alignment: How well at least, as the cosine of the angle between
        them, a horizontal grasp's approach has to follow ``direction``.
    :return: The grasps that come in along the direction, the level ones before the
        rising ones and each best aligned first, followed by those coming in from
        above. A grasp from the side reaches across the counter or along the front of a
        fixture, where the arm sweeps through it.
    """
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)

    def approach_of(grasp: GraspCandidate) -> np.ndarray:
        return grasp.world_T_grasp.to_np()[:3, 0]

    def alignment_of(grasp: GraspCandidate) -> float:
        approach = approach_of(grasp)
        horizontal_part = np.linalg.norm(approach[:2])
        if horizontal_part < 1e-6:
            return 0.0
        return float(approach[:2] @ direction[:2]) / horizontal_part

    level_or_rising = [
        grasp
        for grasp in grasps
        if -0.5 < approach_of(grasp)[2] < 0.9
        and alignment_of(grasp) >= minimum_alignment
    ]
    level_or_rising.sort(
        key=lambda grasp: (abs(approach_of(grasp)[2]) > 0.3, -alignment_of(grasp))
    )
    from_above = [grasp for grasp in grasps if approach_of(grasp)[2] <= -0.5]
    return level_or_rising + from_above


# %% moving a mechanism to a position


@dataclass(eq=False, repr=False)
class DriveJoint(Goal):
    """
    Move one joint of the environment to a position while keeping the robot's tool
    frame fixed relative to a part the joint moves, as a hand holding or pushing that
    part would.

    Unlike :class:`~giskardpy.motion_statechart.goals.open_close.Open`, the joint is
    named rather than taken to be the first one above the part, so that a part hanging
    from two joints, such as a faucet handle turned on and swung for hot water, is moved
    about the right one.
    """

    tip_link: KinematicStructureEntity = field(kw_only=True)
    """
    The robot's tool frame.
    """

    environment_link: KinematicStructureEntity = field(kw_only=True)
    """
    The part the tool frame stays fixed to.
    """

    joint: ActiveConnection1DOF = field(kw_only=True)
    """
    The joint moved.
    """

    goal_position: float = field(kw_only=True)
    """
    The position the joint is moved to.
    """

    threshold: float = field(default=0.01, kw_only=True)
    """
    How close the joint has to come to :attr:`goal_position`.
    """

    mechanism_weight: float = field(
        default=DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE, kw_only=True
    )
    """
    Weight of the goal on the joint, below collision avoidance for the reason
    :attr:`~giskardpy.motion_statechart.goals.open_close.Open.mechanism_weight` gives.
    """

    grasp_weight: float = field(
        default=DefaultWeights.WEIGHT_ABOVE_COLLISION_AVOIDANCE, kw_only=True
    )
    """
    Weight of the goal keeping the tool frame fixed to the part, above collision
    avoidance for the reason
    :attr:`~giskardpy.motion_statechart.goals.open_close.Open.grasp_weight` gives.
    """

    def expand(self, context: MotionStatechartContext) -> None:
        self._add_children_to_motion_statechart(
            [
                JointPositionList(
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

    def build_artifacts(self, context: MotionStatechartContext) -> NodeArtifacts:
        """
        Observe the goal reached once the joint and the hold on the part both are.
        """
        return NodeArtifacts(
            observation=trinary_logic_and(*[node.goal_reached for node in self.nodes])
        )


@dataclass
class MechanismMotion(BaseMotion):
    """
    Moves a joint to a position, keeping the gripper where it holds or pushes a part
    the joint moves.
    """

    part: Body
    """
    The part the gripper holds or pushes.
    """

    arm: Arm
    """
    The arm whose gripper is on the part.
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

    hold_base: bool = False
    """
    Whether the base stands still while the joint moves in a physical simulation, as
    for a knob turned by the wrist alone: a base driven by its wheels' velocities drifts
    by a few millimetres, enough to drag the fingers off a knob's bar.
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        drive = DriveJoint(
            tip_link=self.arm.end_effector.tool_frame,
            environment_link=self.part,
            joint=self.joint,
            goal_position=self.goal_position,
            threshold=self.threshold,
        )
        if (
            not self.hold_base
            or GiskardExecutable.execution_type == ExecutionType.KINEMATICALLY_SIMULATED
        ):
            return drive
        return Parallel(
            [
                drive,
                HoldPose(
                    name="hold base", root_link=self.world.root, tip_link=self.robot.root
                ),
            ]
        )


@dataclass
class StandstillExecutable(Executable):
    """
    Does nothing itself: the motion before it is a chart of its own, which ends once
    the physics has brought every servo-driven joint to a standstill.
    """


@dataclass(eq=False)
class WaitForStandstillNode(ExecutionBoundaryNode):
    """
    Keeps the motions before and after it out of one chart, so that in a physical
    simulation the joints come to rest where the motions before it sent them before
    the ones after it begin.

    A controller takes the joints to be where it commands them, while the servos of a
    physically simulated robot only follow: fingers told to close on a part are still
    closing when the motion that closes them ends.
    """

    def notify(self) -> None:
        pass

    def parse(self) -> StandstillExecutable:
        return StandstillExecutable(context=self.context)


@dataclass
class OperateMechanismAction(ContainerAction):
    """
    Takes hold of a part of a fixture, moves a joint the part hangs from to a position
    once the fingers have closed on it, lets go of the part and backs off it.
    """

    joint: ActiveConnection1DOF | None = field(default=None, kw_only=True)
    """
    The joint moved; the first one above the part if not given.
    """

    goal_position: float = field(default=0.0, kw_only=True)
    """
    The position the joint is moved to.
    """

    threshold: float = field(default=0.01, kw_only=True)
    """
    How close the joint has to come to :attr:`goal_position`.
    """

    hold_base: bool = field(default=False, kw_only=True)
    """
    Whether the base stands still while the joint moves, see
    :attr:`MechanismMotion.hold_base`.
    """

    release_clearance: float = 0.12
    """
    The gap in metres between the part and the gripper once it has let go: enough for
    open fingers that turn back as the arms park to pass clear of a knob's bar.
    """

    @property
    def joint_moved(self) -> ActiveConnection1DOF:
        """
        The joint moved: :attr:`joint`, or the first one above the part, as it is in
        the world the action is performed in, where :attr:`joint` may be one of a copy
        of the world a trial had.
        """
        above_the_part = self.handle.root.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )
        if self.joint is None:
            return above_the_part
        return next(
            (
                connection
                for connection in self.world.connections
                if connection.name.name == self.joint.name.name
            ),
            above_the_part,
        )

    def back_off_pose(self, grasp_pose: Pose, end_effector) -> Pose:
        """
        The tool frame goal that clears the released part, where the part stands once
        the joint is at :attr:`goal_position`, stated relative to what the joint hangs
        from.

        A goal stated relative to the part would move with it: in a physical simulation
        the controller takes the part to follow the gripper all the way, and the part
        may stand elsewhere once the physics hands back where it really is; backing off
        relative to it then sweeps the open fingers across it.

        :param grasp_pose: The grasp frame the part is held by.
        :param end_effector: The end effector that holds it.
        :return: The pose the tool frame backs off to, in the frame of the body the
            joint hangs from.
        """
        world = self.world
        joint = self.joint_moved
        back_off = super().back_off_pose(grasp_pose, end_effector)
        start = joint.position
        joint.position = self.goal_position
        try:
            world_T_back_off = world.transform(back_off, world.root)
        finally:
            joint.position = start
        return world.transform(world_T_back_off, joint.parent)

    @property
    def _action_plan(self) -> PlanNode:
        handle_grasp = self.handle_grasp
        end_effector = self.arm.end_effector
        # in a physical simulation the fingers close on the part before it is moved
        settling = (
            [WaitForStandstillNode()]
            if GiskardExecutable.execution_type == ExecutionType.PHYSICALLY_SIMULATED
            else []
        )
        return sequential(
            [
                self.grasping_of(handle_grasp, self.arm, self.tolerate_grasp_stall),
                *settling,
                self._mechanism_motion,
                MoveGripperMotion(
                    GripperState.OPEN, end_effector, allow_gripper_collision=True
                ),
                MoveToolCenterPointMotion(
                    self.back_off_pose(handle_grasp.grasp_pose, end_effector),
                    self.arm,
                    allow_gripper_collision=True,
                    position_threshold=self.position_threshold,
                    orientation_threshold=self.orientation_threshold,
                ),
            ]
        )

    @property
    def _mechanism_motion(self) -> BaseMotion:
        joint = self.joint_moved
        return MechanismMotion(
            self.handle.root,
            self.arm,
            joint=joint,
            goal_position=self.goal_position,
            threshold=self.threshold,
            hold_base=self.hold_base,
        )


@dataclass
class StandAndOperateMechanismAction(MayReachThroughFreeSpace, ActionDescription):
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
        stands: List[Pose],
        joint: ActiveConnection1DOF,
        goal_position: float,
        grasps: List[GraspCandidate],
        threshold: float = 0.01,
        number_of_grasps: int | None = None,
        approach_clearance: float = 0.2,
        through_free_space: bool = False,
        hold_base: bool = False,
    ) -> Match:
        """
        :param handle: The part the mechanism is moved by.
        :param arm: The arm that moves it.
        :param stands: Where the robot may stand, best first.
        :param joint: The joint moved.
        :param goal_position: The position the joint is moved to.
        :param threshold: How close the joint has to come to ``goal_position``.
        :param grasps: The grasps the part may be taken hold of by.
        :param number_of_grasps: How many of the grasps closest to a stand are tried
            from there; all of them if not given.
        :param approach_clearance: How far from the part the gripper starts its last
            straight approach onto it, in metres: outside the distance collision
            avoidance keeps the gripper from what the part is fixed to.
        :param through_free_space: Whether the gripper is led to the part along a
            path through free space.
        :param hold_base: Whether the base stands still while the joint moves, see
            :attr:`MechanismMotion.hold_base`.
        :return: The step, with the stand and the grasp left open.
        """
        step = a(cls)(
            navigate=a(StandAction)(
                target_location=variable(Pose, domain=stands)
            ),
            operate=a(OperateMechanismAction)(
                handle=handle,
                arm=arm,
                grasp=variable(GraspCandidate, domain=grasps),
                joint=joint,
                goal_position=goal_position,
                threshold=threshold,
                approach_clearance=approach_clearance,
                hold_base=hold_base,
            ),
            through_free_space=through_free_space,
        )
        return step.where(
            IsAmongTheClosestGraspsTo(
                step.operate.grasp,
                step.navigate.target_location,
                grasps,
                number_of_grasps or len(grasps),
            )
        )

    @property
    def _action_plan(self) -> PlanNode:
        operate = self.operate
        pre_grasp = operate.grasp_pose_sequence(
            operate.handle_grasp.grasp_pose, operate.arm.end_effector, operate.handle_grasp
        ).pre_grasp
        return sequential(
            [
                self.navigate,
                *self.reaching(ReachThroughFreeSpaceAction(pre_grasp, operate.arm)),
                operate,
            ]
        )


# %% pressing a button


def fingertips_goal(
    arm: Arm, point: np.ndarray, approach: np.ndarray, distance_before_the_point: float
) -> Pose:
    """
    :param arm: The arm whose fingertips are placed.
    :param point: The point the fingertips come in to, in the world's root frame.
    :param approach: The direction they come in along.
    :param distance_before_the_point: How far back along the approach the fingertips
        stay from the point, in metres.
    :return: The tool frame goal that puts the fingertips there, with the fingers
        closing level across the approach.
    """
    world = arm._world
    approach = approach / np.linalg.norm(approach)
    closing = np.cross([0.0, 0.0, 1.0], approach)
    if np.linalg.norm(closing) < 1e-6:
        closing = np.array([0.0, 1.0, 0.0])
    world_T_press = frame_at(
        point - approach * distance_before_the_point,
        approach,
        closing / np.linalg.norm(closing),
        world.root,
    )
    return arm.end_effector.tool_frame_goal_for_grasp(world_T_press.pose)


@dataclass
class PressAction(ActionDescription):
    """
    Closes the gripper and pushes its fingertips against a point, coming in along a
    direction.

    The gripper stays on the point; :class:`WithdrawAction` takes it off again.
    """

    point: np.ndarray
    """
    The point pressed, in the world's root frame.
    """

    approach: np.ndarray
    """
    The direction the fingertips come in along, in the world's root frame.
    """

    arm: Arm
    """
    The arm that presses.
    """

    approach_distance: float = 0.25
    """
    How far before the point the straight push onto it starts, in metres: outside the
    distance collision avoidance keeps the gripper from the kitchen, which would hold
    it back from a nearer start.
    """

    def press_goal(self, distance_before_the_point: float) -> Pose:
        """
        :param distance_before_the_point: How far back along the approach the
            fingertips stay from the point, in metres.
        :return: The tool frame goal that puts the fingertips there.
        """
        return fingertips_goal(
            self.arm, self.point, self.approach, distance_before_the_point
        )

    @property
    def _action_plan(self) -> PlanNode:
        return sequential(
            [
                MoveGripperMotion(GripperState.CLOSE, self.arm.end_effector),
                MoveToolCenterPointMotion(
                    self.press_goal(self.approach_distance), self.arm
                ),
                MoveToolCenterPointMotion(
                    self.press_goal(0.0),
                    self.arm,
                    allow_gripper_collision=True,
                    movement_type=MovementType.STRAIGHT_CARTESIAN,
                ),
            ]
        )


@dataclass
class PushAction(ActionDescription):
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

    @property
    def _action_plan(self) -> PlanNode:
        return sequential(
            [
                self.press,
                MechanismMotion(
                    self.part,
                    self.press.arm,
                    joint=self.joint,
                    goal_position=self.goal_position,
                    threshold=self.threshold,
                ),
            ]
        )


@dataclass
class WithdrawAction(ActionDescription):
    """
    Draws the gripper straight back from a point it pressed, the way it came in.
    """

    point: np.ndarray
    """
    The point pressed, in the world's root frame.
    """

    approach: np.ndarray
    """
    The direction the fingertips came in along.
    """

    arm: Arm
    """
    The arm that pressed.
    """

    distance: float = PressAction.approach_distance
    """
    How far back from the point the gripper withdraws, in metres.
    """

    @property
    def _action_plan(self) -> PlanNode:
        return sequential(
            [
                MoveToolCenterPointMotion(
                    fingertips_goal(self.arm, self.point, self.approach, self.distance),
                    self.arm,
                    allow_gripper_collision=True,
                    movement_type=MovementType.STRAIGHT_CARTESIAN,
                )
            ]
        )


@dataclass
class StandAndPressAction(MayReachThroughFreeSpace, ActionDescription):
    """
    Walks to where the robot stands and presses the point.
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
        stands: List[Pose],
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
            navigate=a(StandAction)(
                target_location=variable(Pose, domain=stands)
            ),
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

    @property
    def _action_plan(self) -> PlanNode:
        press = self.press if isinstance(self.press, PressAction) else self.press.press
        return sequential(
            [
                self.navigate,
                MoveGripperMotion(GripperState.CLOSE, press.arm.end_effector),
                *self.reaching(
                    ReachThroughFreeSpaceAction(
                        press.press_goal(press.approach_distance), press.arm
                    )
                ),
                self.press,
            ]
        )


def horizontal_direction(from_point: np.ndarray, to_point: np.ndarray) -> np.ndarray:
    """
    :param from_point: Where the direction starts.
    :param to_point: Where it points to.
    :return: The unit direction from one point to the other, levelled.
    """
    direction = np.array(to_point[:2]) - np.array(from_point[:2])
    norm = np.linalg.norm(direction)
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0])
    return np.append(direction / norm, 0.0)


def yaw_of(direction: np.ndarray) -> float:
    """
    :param direction: A horizontal direction.
    :return: The yaw it points at.
    """
    return math.atan2(direction[1], direction[0])


# %% picking an object up and putting it down


def grasp_held_by(arm: Arm, graspable: HasGraspCandidates) -> GraspCandidate:
    """
    :param arm: An arm holding an object.
    :param graspable: The object.
    :return: The grasp the arm's gripper holds the object by: the one it hangs off the
        tool frame by, or, where the fingers hold it by contact alone, the one where it
        sits relative to the tool frame now.
    """
    end_effector = arm.end_effector
    try:
        return GraspCandidate(graspable, end_effector.held_body_T_grasp)
    except NothingHeld:
        return GraspCandidate(graspable, end_effector.body_T_grasp(graspable.root))


def lies_between_the_fingers(
    arm: Arm, graspable: HasGraspCandidates, reach: float = 0.12
) -> bool:
    """
    :param arm: An arm.
    :param graspable: An object.
    :param reach: How far from the arm's tool frame the object's collision geometry may
        lie at most, in metres.
    :return: Whether the object lies in the arm's gripper.
    """
    world = arm._world
    tool = arm.end_effector.tool_frame.global_pose.to_np()[:3, 3]
    box = graspable.root.collision.as_bounding_box_collection_in_frame(
        world.root
    ).bounding_box()
    lower = np.array([box.min_x, box.min_y, box.min_z])
    upper = np.array([box.max_x, box.max_y, box.max_z])
    return float(np.linalg.norm(tool - np.clip(tool, lower, upper))) <= reach


@dataclass
class PlaceHeldObjectAction(PlaceAction):
    """
    Puts down an object an arm is known to hold.

    In a physical simulation the fingers hold the object by friction alone. Where the
    rays between them that coraplex looks for it with miss an object held at their
    tips, the grasp is read off where the object sits in the gripper instead.
    """

    holding_arm: Arm | None = field(default=None, kw_only=True)
    """
    The arm that holds the object.
    """

    def _holding_arm_and_grasp(self) -> tuple[Arm, GraspCandidate]:
        try:
            return super()._holding_arm_and_grasp()
        except ObjectIsNotHeld:
            if self.holding_arm is None or not lies_between_the_fingers(
                self.holding_arm, self.object_designator
            ):
                raise
            return self.holding_arm, grasp_held_by(
                self.holding_arm, self.object_designator
            )


@dataclass
class StandAndPickUpAction(MayReachThroughFreeSpace, ActionDescription):
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
        stands: List[Pose],
        grasps: List[GraspCandidate],
        number_of_grasps: int | None = None,
        through_free_space: bool = False,
    ) -> Match:
        """
        :param graspable: The object to pick up.
        :param arm: The arm to pick it up with.
        :param stands: Where the robot may stand, best first.
        :param grasps: The grasps the object may be taken by.
        :param number_of_grasps: How many of the grasps closest to a stand are tried
            from there; all of them if not given.
        :param through_free_space: Whether the gripper is led to the object along a
            path through free space.
        :return: The step, with the stand and the grasp left open.
        """
        step = a(cls)(
            navigate=a(StandAction)(
                target_location=variable(Pose, domain=stands)
            ),
            pick_up=a(PickUpAction)(
                grasp=variable(GraspCandidate, domain=grasps), arm=arm
            ),
            through_free_space=through_free_space,
        )
        return step.where(
            IsAmongTheClosestGraspsTo(
                step.pick_up.grasp,
                step.navigate.target_location,
                grasps,
                number_of_grasps or len(grasps),
            )
        )

    @property
    def _action_plan(self) -> PlanNode:
        pick_up = self.pick_up
        pre_grasp = pick_up.grasp_pose_sequence(
            pick_up.grasp.grasp_pose, pick_up.arm.end_effector, pick_up.grasp
        ).pre_grasp
        return sequential(
            [
                self.navigate,
                *self.reaching(ReachThroughFreeSpaceAction(pre_grasp, pick_up.arm)),
                pick_up,
            ]
        )


@dataclass
class StandAndPlaceAction(MayReachThroughFreeSpace, ActionDescription):
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
        target_poses: List[Pose],
        stands: List[Pose],
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
            navigate=a(StandAction)(
                target_location=variable(Pose, domain=stands)
            ),
            place=a(PlaceHeldObjectAction)(
                object_designator=graspable,
                target_location=variable(Pose, domain=target_poses),
                retreat_distance=retreat_distance,
                holding_arm=arm,
            ),
            arm=arm,
            through_free_space=through_free_space,
        )

    @property
    def _action_plan(self) -> PlanNode:
        return sequential(
            [
                self.navigate,
                *self.reaching(
                    ReachAboveThePlacementAction(
                        goal=None, arm=self.arm, place=self.place
                    )
                ),
                self.place,
            ]
        )
