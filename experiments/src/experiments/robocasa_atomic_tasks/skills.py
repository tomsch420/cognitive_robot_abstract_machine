"""
The actions the PR2 performs RoboCasa's atomic tasks with, beyond those coraplex ships:
moving a fixture's mechanism by one of its parts to a given position, and pressing a
button.

Each comes with a counterpart that also walks the robot to a stand, picked among a
list of candidates, as is the grasp, by coraplex trying them, and then may lead the
gripper along a path through the kitchen's free space to where its last straight
approach begins.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from typing_extensions import Sequence

from coraplex.datastructures.trajectory import PoseTrajectory
from coraplex.exceptions import ObjectIsNotHeld
from coraplex.querying.grasp_closeness import GraspCloseness
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
from cramph.monitors import CountSimulationTimeSeconds
from cramph.node import CompositeNode, EndedByOwner, NodeArtifacts, StatechartNode
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
from krrood.symbolic_math.symbolic_math import trinary_logic_and
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForEndEffector,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.exceptions import NothingHeld
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import Arm, EndEffector
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


@dataclass(eq=False, repr=False, kw_only=True)
class StandAction(NavigateAction):
    """
    Takes the robot to where it is to stand, as close as a base driven by its wheels'
    velocities settles in a physical simulation.
    """

    base_pose_threshold: float = 0.03
    """
    How far the base may be from where it is to stand, in metres and in radians: a base
    driven by its wheels' velocities in a physical simulation settles a centimetre or two
    beside its goal and holds there, and a goal that waits for it to come nearer never
    ends.
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

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to come close to what it reaches for, as
            the last motion onto the goal is the one the action reached for afterwards
            begins with.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
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
        end_effector = self.arm.end_effector
        grasp = grasp_held_by(self.arm, self.place.object_designator)
        return self.place.grasp_pose_sequence(
            grasp.moved_to(self.place.target_location), end_effector, grasp
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
            begins, grounded only once the reach begins, so that its path sets off from
            where the tool frame then is.
        :return: The reach, if the step reaches through free space; nothing otherwise.
        """
        return [reach] if self.through_free_space else []


# %% grasping a fixture's part


def part_grasps(
    handle: Handle,
    depth: float = 0.02,
    upward_slope: float = 0.8,
    lower_end_height: float = 0.04,
) -> list[GraspCandidate]:
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
) -> list[GraspCandidate]:
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
    grasps: list[GraspCandidate], direction: np.ndarray, minimum_alignment: float = 0.5
) -> list[GraspCandidate]:
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


@dataclass(eq=False, repr=False, kw_only=True)
class DriveJoint(EndedByOwner, CompositeNode):
    """
    Move one joint of the environment to a position while keeping the robot's tool
    frame fixed relative to a part the joint moves, as a hand holding or pushing that
    part would.

    Unlike
    :class:`~giskardpy.motion_statechart.goals.grasped_mechanism.ArticulateGraspedMechanism`,
    the joint is named rather than taken to be the first one above the part, so that a
    part hanging from two joints, such as a faucet handle turned on and swung for hot
    water, is moved about the right one.
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
    Weight of the goal on the joint, below collision avoidance for the reason
    :attr:`~giskardpy.motion_statechart.goals.grasped_mechanism.ArticulateGraspedMechanism.mechanism_weight`
    gives.
    """

    grasp_weight: float = DefaultWeights.WEIGHT_ABOVE_COLLISION_AVOIDANCE
    """
    Weight of the goal keeping the tool frame fixed to the part, above collision
    avoidance for the reason
    :attr:`~giskardpy.motion_statechart.goals.grasped_mechanism.ArticulateGraspedMechanism.grasp_weight`
    gives.
    """

    def expand(self) -> None:
        self._add_children_to_statechart(
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

    def build_artifacts(self) -> NodeArtifacts:
        """
        Observe the goal reached once the joint and the hold on the part both are.
        """
        return NodeArtifacts(
            observation=trinary_logic_and(
                *[node.observation_variable for node in self.nodes]
            )
        )


def mechanism_goal(
    action: MovesToolCenterPoint,
    part: Body,
    arm: Arm,
    joint: ActiveConnection1DOF,
    goal_position: float,
    threshold: float,
    hold_base: bool,
) -> StatechartNode:
    """
    :param action: The action the goal is a step of.
    :param part: The part the gripper holds or pushes.
    :param arm: The arm whose gripper is on the part.
    :param joint: The joint moved.
    :param goal_position: The position the joint is moved to.
    :param threshold: How close the joint has to come to `goal_position`.
    :param hold_base: Whether the base stands still while the joint moves, as for a
        knob turned by the wrist alone: a base driven by its wheels' velocities drifts
        by a few millimetres, enough to drag the fingers off a knob's bar.
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
    return Parallel(
        nodes=[
            drive,
            HoldPose(
                name="hold base",
                root_link=action.world.root,
                tip_link=action.robot.root,
            ),
        ]
    )


@dataclass(eq=False, repr=False, kw_only=True)
class OperateMechanismAction(MovesContainerByItsHandle):
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
    Whether the base stands still while the joint moves, see :func:`mechanism_goal`.
    """

    settling_time: float = 0.0
    """
    How much simulated time, in seconds, the fingers are given to close on the part
    before the joint moves. A controller takes the joints to be where it commands them,
    while the servos of a physically simulated robot only follow: fingers told to close
    on a part are still closing when the motion that closes them ends.
    """

    release_clearance: float = 0.12
    """
    The gap in metres between the part and the gripper once it has let go: enough for
    open fingers that turn back as the arms park to pass clear of a knob's bar.
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

        A goal stated relative to the part would move with it: in a physical simulation
        the controller takes the part to follow the gripper all the way, and the part
        may stand elsewhere once the physics hands back where it really is; backing off
        relative to it then sweeps the open fingers across it.
        """
        world = self.world
        joint = self.mechanism
        back_off = super().back_off_pose(grasp_pose, end_effector)
        with world.reset_state_context():
            joint.position = self.goal_position
            world_T_back_off = world.transform(back_off, world.root)
        return world.transform(world_T_back_off, joint.parent)

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

    def _create_mechanism_goal(self) -> StatechartNode:
        return mechanism_goal(
            self,
            self.grasp.graspable.root,
            self.arm,
            self.mechanism,
            self.goal_position,
            self.threshold,
            self.hold_base,
        )


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
            straight approach onto it, in metres: outside the distance collision
            avoidance keeps the gripper from what the part is fixed to.
        :param through_free_space: Whether the gripper is led to the part along a
            path through free space.
        :param hold_base: Whether the base stands still while the joint moves, see
            :func:`mechanism_goal`.
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


@dataclass(eq=False, repr=False, kw_only=True)
class PressAction(MovesToolCenterPoint, OperatesGripper, SequentialAction):
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

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to touch what it presses.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
        return [
            self.gripper_goal_type.moving_gripper_to(
                self.arm.end_effector.default_configuration(GripperState.CLOSE)
            ),
            self.tool_center_point_goal(
                self.press_goal(self.approach_distance), self.arm
            ),
            self.tool_center_point_goal(
                self.press_goal(0.0), self.arm, in_a_straight_line=True
            ),
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class PushAction(MovesToolCenterPoint, SequentialAction):
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

    def create_steps(self) -> list[StatechartNode]:
        return [
            self.press,
            mechanism_goal(
                self,
                self.part,
                self.press.arm,
                self.joint,
                self.goal_position,
                self.threshold,
                hold_base=False,
            ),
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class WithdrawAction(MovesToolCenterPoint, SequentialAction):
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

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to touch what it pressed while it leaves.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
        return [
            self.tool_center_point_goal(
                fingertips_goal(self.arm, self.point, self.approach, self.distance),
                self.arm,
                in_a_straight_line=True,
            )
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class StandAndPressAction(MayReachThroughFreeSpace, OperatesGripper, SequentialAction):
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
                press.arm.end_effector.default_configuration(GripperState.CLOSE)
            ),
            *self.reaching(
                a(ReachThroughFreeSpaceAction)(
                    goal=press.press_goal(press.approach_distance), arm=press.arm
                )
            ),
            self.press,
        ]


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


@dataclass(eq=False, repr=False, kw_only=True)
class PlaceHeldObjectAction(PlaceAction):
    """
    Puts down an object an arm is known to hold.

    In a physical simulation the fingers hold the object by friction alone. Where the
    rays between them that coraplex looks for it with miss an object held at their
    tips, the grasp is read off where the object sits in the gripper instead.
    """

    holding_arm: Arm | None = None
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
