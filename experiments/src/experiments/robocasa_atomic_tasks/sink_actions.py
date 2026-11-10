"""
Actions at the sink, written from what worked when the robot was driven there step by
step: lifting the hand out of the basin, and turning the faucet on and off by pushing
its handle with one finger.

Each parameter is a field defaulting to the value that worked, so that the values can be
varied and what works learned; the preconditions are EQL conditions checked when the
action starts.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from coraplex.robot_plans.actions.base import GoalAction, SequentialAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import (
    MoveTorsoAction,
    SetGripperAction,
)
from coraplex.robot_plans.mixins import MovesToolCenterPoint
from cramph.composites import Parallel
from cramph.data_types import ObservationStateValues
from cramph.node import EndedByOwner, StatechartNode
from cramph.threaded_nodes import FunctionCall
from experiments.robocasa_atomic_tasks.exceptions import (
    FaucetNotTurnedOff,
    FaucetNotTurnedOn,
    HandleHasNoBar,
    LeverSwungAside,
    PreconditionsNotMet,
)
from experiments.robocasa_atomic_tasks.skills import ReachThroughFreeSpaceAction
from giskardpy.motion_statechart.tasks.cartesian_tasks import HoldPose
from krrood.entity_query_language.factories import (
    a,
    and_,
    evaluate_condition,
    get_false_statements,
    not_,
    variable,
    ConditionType,
)
from krrood.entity_query_language.predicate import Predicate, RenderedFields
from krrood.entity_query_language.query.match import Match
from krrood.entity_query_language.verbalization.fragments.base import (
    VerbalizationFragment,
)
from krrood.entity_query_language.verbalization.pipeline import VerbalizationPipeline
from krrood.entity_query_language.verbalization.vocabulary.parts_of_speech import (
    Adjective,
    Copula,
    Noun,
    clause,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.adapters.multi_sim import MujocoSynchronizer
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AllowCollisionForEndEffector,
)
from semantic_digital_twin.datastructures.definitions import GripperState, TorsoState
from semantic_digital_twin.reasoning.predicates import InsideOf, PlaceIsOccupied
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.geometry import (
    Cylinder,
    VolumetricBoundingBox,
)
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

# %% reading joints and boxes


@dataclass
class JointReading:
    """
    Where the joints of a world stand: as the physics last moved them where the world is
    simulated physically, as the world holds them otherwise.
    """

    world: World
    """
    The world the joints belong to.
    """

    def position(self, joint: ActiveConnection1DOF) -> float:
        """
        :param joint: A joint of :attr:`world`.
        :return: Where the joint stands.
        """
        synchronizer = next(
            iter(MujocoSynchronizer.all_callbacks_of_this_type_from_world(self.world)),
            None,
        )
        if synchronizer is None:
            return joint.position
        return synchronizer.simulated_position(joint)


@dataclass
class Extent:
    """
    How far something reaches along each of three directions.
    """

    lower: np.ndarray
    """
    The least coordinate along each direction.
    """

    upper: np.ndarray
    """
    The greatest coordinate along each direction.
    """

    @classmethod
    def from_body(
        cls, body: Body, frame: KinematicStructureEntity, axes: np.ndarray
    ) -> Extent:
        """
        :param body: A body with collision geometry.
        :param frame: The frame its collision's bounding box is taken in.
        :param axes: The directions, as the rows of a matrix, in that frame.
        :return: How far the corners of that bounding box reach along the directions.
        """
        bounds = (
            body.collision.as_bounding_box_collection_in_frame(frame)
            .bounding_box()
            .to_array_bounds()
        )
        corners = np.array(
            [
                [x, y, z]
                for x in (bounds.lower[0], bounds.upper[0])
                for y in (bounds.lower[1], bounds.upper[1])
                for z in (bounds.lower[2], bounds.upper[2])
            ]
        )
        along = corners @ axes.T
        return cls(lower=along.min(axis=0), upper=along.max(axis=0))


@dataclass
class SpaceInTheWorld:
    """
    A box of space at a pose, as
    :class:`~semantic_digital_twin.reasoning.predicates.PlaceIsOccupied` takes it.
    """

    box: VolumetricBoundingBox
    """
    The box, about its own middle.
    """

    pose: Pose
    """
    Where the middle of the box stands, turned as the box is.
    """

    @classmethod
    def from_middle(
        cls, middle: np.ndarray, rotation: np.ndarray, size: np.ndarray, world: World
    ) -> SpaceInTheWorld:
        """
        :param middle: The middle of the box, in the world's root frame.
        :param rotation: The box's axes, as the columns of a matrix, in the world's root
            frame.
        :param size: How long the box is along each of its axes, in metres.
        :param world: The world the box stands in.
        :return: The box.
        """
        half = size / 2
        box = VolumetricBoundingBox(
            min_x=-half[0],
            min_y=-half[1],
            min_z=-half[2],
            max_x=half[0],
            max_y=half[1],
            max_z=half[2],
            origin=HomogeneousTransformationMatrix(reference_frame=world.root),
        )
        world_T_middle = np.eye(4)
        world_T_middle[:3, :3] = rotation
        world_T_middle[:3, 3] = middle
        return cls(
            box=box,
            pose=HomogeneousTransformationMatrix(
                world_T_middle, reference_frame=world.root
            ).pose,
        )

    def occupied(self, world: World, allowed_bodies: list[Body]) -> PlaceIsOccupied:
        """
        :param world: The world the box stands in.
        :param allowed_bodies: The bodies that may stand in the box.
        :return: The statement that something other than ``allowed_bodies`` stands in
            the box.
        """
        return PlaceIsOccupied(
            box=variable(VolumetricBoundingBox, [self.box]),
            pose=self.pose,
            world=world,
            allowed_bodies=allowed_bodies,
        )


@dataclass
class FingerExtents:
    """
    How far each body of a gripper's fingers reaches along the axes of its tool frame,
    as the fingers stand now.
    """

    extents: list[Extent]
    """
    The extent of each finger body, in the tool frame.
    """

    @classmethod
    def from_arm(cls, arm: Arm) -> FingerExtents:
        """
        :param arm: An arm with a fingered gripper.
        :return: The extents of the bodies of its fingers, without the body the fingers
            hang from.
        """
        gripper = arm.end_effector
        hand = {finger.root for finger in gripper.fingers}
        return cls(
            extents=[
                Extent.from_body(body, gripper.tool_frame, np.eye(3))
                for body in gripper.bodies_with_collision
                if body not in hand
            ]
        )


# %% the faucet's handle


@dataclass(eq=False)
class FaucetIsOn(Predicate):
    """
    Whether water runs from a faucet, by how far its handle is lifted.
    """

    lift_joint: ActiveConnection1DOF
    """
    The joint the faucet's handle is lifted about.
    """

    water_runs_from: float = 0.4
    """
    How far the handle has to be lifted for the water to run, in radians; RoboCasa's own
    threshold.
    """

    def __call__(self) -> bool:
        return self.lift_joint.position > self.water_runs_from

    @classmethod
    def _verbalization_fragment_(cls, fields: RenderedFields) -> VerbalizationFragment:
        """
        Reads as *"the lift joint is open"*.

        :param fields: The rendered fragment for each field, keyed by field name.
        """
        return clause(Noun(fields["lift_joint"]), Copula(), Adjective("open"))


@dataclass
class FaucetHandle:
    """
    A faucet's handle as a finger pushes it: the level direction pushing its top lifts
    it, the level direction from a robot in front of it towards it, and how far its
    collision reaches along these and upwards.
    """

    push: np.ndarray
    """
    The level direction pushing the handle's top lifts it, in the world's root frame.
    """

    facing: np.ndarray
    """
    The level direction along the lift joint's axis, from in front of the handle towards
    it, in the world's root frame.
    """

    anchor: np.ndarray
    """
    Where the lift joint's axis passes through, in the world's root frame.
    """

    extent: Extent
    """
    How far the handle's collision reaches along :attr:`push`, :attr:`facing` and up.
    """

    @classmethod
    def from_lift_joint(
        cls, lift_joint: ActiveConnection1DOF, handle: Body
    ) -> FaucetHandle:
        """
        :param lift_joint: The joint the handle is lifted about.
        :param handle: The handle.
        :return: The handle, as it stands now.
        """
        world = handle._world
        world_T_joint = (
            lift_joint.parent.global_pose.to_np()
            @ lift_joint.parent_T_connection_expression.to_np()
        )
        anchor = world_T_joint[:3, 3]
        axis = world_T_joint[:3, :3] @ lift_joint.axis.to_np()[:3]
        up = np.array([0.0, 0.0, 1.0])
        facing = axis - (axis @ up) * up
        facing = facing / np.linalg.norm(facing)
        box = Extent.from_body(handle, world.root, np.eye(3))
        lever_arm = (box.lower + box.upper) / 2 - anchor
        push = np.cross(axis, lever_arm)
        push = push - (push @ up) * up - (push @ facing) * facing
        push = push / np.linalg.norm(push)
        axes = np.array([push, facing, up])
        return cls(
            push=push,
            facing=facing,
            anchor=anchor,
            extent=Extent.from_body(handle, world.root, axes),
        )

    @property
    def axes(self) -> np.ndarray:
        """
        :return: :attr:`push`, :attr:`facing` and up, as the rows of a matrix.
        """
        return np.array([self.push, self.facing, [0.0, 0.0, 1.0]])

    def point(self, along: np.ndarray) -> np.ndarray:
        """
        :param along: Coordinates along :attr:`push`, :attr:`facing` and up.
        :return: The point with these coordinates, in the world's root frame.
        """
        return self.axes.T @ along

    @property
    def tool_rotation(self) -> np.ndarray:
        """
        :return: The orientation of a tool frame facing the handle from the front, its
            fingers closing along :attr:`push`: its z-axis along :attr:`facing`, its
            x-axis against :attr:`push`, its y-axis up.
        """
        return np.column_stack([-self.push, [0.0, 0.0, 1.0], self.facing])

    def standing_pose_in_front(
        self, distance: float, sideways_offset: float
    ) -> np.ndarray:
        """
        :param distance: How far in front of the lift joint's axis the pose lies, in
            metres.
        :param sideways_offset: How far along :attr:`push` from the middle of the
            handle the pose lies, in metres.
        :return: The pose on the floor facing the handle, in the world's root frame.
        """
        middle = (self.extent.lower[0] + self.extent.upper[0]) / 2
        up = np.array([0.0, 0.0, 1.0])
        world_T_stand = np.eye(4)
        world_T_stand[:3, :3] = np.column_stack(
            [self.facing, np.cross(up, self.facing), up]
        )
        world_T_stand[:3, 3] = self.point(
            np.array(
                [middle + sideways_offset, self.anchor @ self.facing - distance, 0.0]
            )
        )
        return world_T_stand


@dataclass
class LeverBar:
    """
    The bar of a faucet's lever handle, the longest cylinder of the handle's collision.
    """

    bottom: np.ndarray
    """
    The middle of the bar's lower end, in the world's root frame.
    """

    up_the_bar: np.ndarray
    """
    The direction along the bar towards its upper end, in the world's root frame.
    """

    length: float
    """
    How long the bar is, in metres.
    """

    radius: float
    """
    How thick the bar is from its middle line, in metres.
    """

    @classmethod
    def from_handle(cls, handle: Body) -> LeverBar:
        """
        :param handle: A lever handle.
        :return: Its bar, as it stands now.
        :raises HandleHasNoBar: If the handle's collision holds no cylinder.
        """
        cylinders = [
            shape for shape in handle.collision.shapes if isinstance(shape, Cylinder)
        ]
        if not cylinders:
            raise HandleHasNoBar(handle=handle)
        bar = max(cylinders, key=lambda cylinder: cylinder.height)
        world_T_bar = handle.global_pose.to_np() @ bar.origin.to_np()
        axis = world_T_bar[:3, 2]
        up_the_bar = axis if axis[2] > 0.0 else -axis
        return cls(
            bottom=world_T_bar[:3, 3] - bar.height / 2 * up_the_bar,
            up_the_bar=up_the_bar,
            length=bar.height,
            radius=bar.width / 2,
        )

    def point_up_the_bar(self, share: float) -> np.ndarray:
        """
        :param share: How far up the bar, as a share of its length.
        :return: The point on the bar's middle line there, in the world's root frame.
        """
        return self.bottom + share * self.length * self.up_the_bar


# %% preconditions


@dataclass(eq=False, repr=False, kw_only=True)
class HasPreconditions(SequentialAction, ABC):
    """
    A sequential action that first checks its preconditions, and fails with
    :class:`~experiments.robocasa_atomic_tasks.exceptions.PreconditionsNotMet` where
    they do not hold.
    """

    @abstractmethod
    def preconditions(self) -> ConditionType:
        """
        :return: What has to hold in the world for this action to start.
        """

    @abstractmethod
    def create_steps_once_the_preconditions_hold(self) -> list[StatechartNode | Match]:
        """
        :return: New nodes doing what this action does, in order, run once the
            preconditions held.
        """

    def create_steps(self) -> list[StatechartNode | Match]:
        return [
            FunctionCall(
                name="check the preconditions",
                function=self.raise_unless_the_preconditions_hold,
            ),
            *self.create_steps_once_the_preconditions_hold(),
        ]

    def raise_unless_the_preconditions_hold(self) -> None:
        """
        :raises PreconditionsNotMet: If :meth:`preconditions` does not hold.
        """
        condition = self.preconditions()
        if evaluate_condition(condition):
            return
        pipeline = VerbalizationPipeline.plain()
        raise PreconditionsNotMet(
            action_name=type(self).__name__,
            false_preconditions=[
                pipeline.verbalize(statement)
                for statement in get_false_statements(condition)
            ],
        )


# %% backing off


@dataclass(eq=False, repr=False, kw_only=True)
class BackOffAction(NavigateAction):
    """
    Drives the robot's base straight backwards from where it stands when the action is
    expanded.
    """

    distance: float
    """
    How far the base backs off, in metres.
    """

    target_location: Pose = field(init=False, default=None)
    """
    Where the base backs off to, set when the action is expanded.
    """

    def create_goal(self) -> StatechartNode:
        world_T_base = self.robot.root.global_pose.to_np().copy()
        world_T_base[:3, 3] -= self.distance * world_T_base[:3, 0]
        self.target_location = HomogeneousTransformationMatrix(
            world_T_base, reference_frame=self.world.root
        ).pose
        return super().create_goal()


# %% leaving the sink


@dataclass(eq=False, repr=False, kw_only=True)
class LeaveTheSinkAction(SequentialAction):
    """
    Leads the hand through free space to a point above and in front of a sink's basin,
    turned as it is, then backs the base off.
    """

    arm: Arm
    """
    The arm whose hand leaves the sink.
    """

    basin: Body
    """
    The sink's basin.
    """

    exit_height_above_the_basin: float = 0.089
    """
    How far above the top of the basin's collision the tool frame is led, in metres.
    """

    exit_distance_in_front_of_the_basin: float = 0.136
    """
    How far in front of the basin's collision, along the way the robot faces, the tool
    frame is led, in metres.
    """

    path_clearance: float = 0.05
    """
    How far the path keeps the tool frame from everything else, in metres.
    """

    base_back_off: float = 0.3
    """
    How far the base backs off once the hand is out, in metres.
    """

    def space_behind_the_base(self) -> SpaceInTheWorld:
        """
        :return: The space the base sweeps through while it backs off.
        """
        base = self.robot.mobile_base
        root = self.robot.root
        footprints = [
            Extent.from_body(body, root, np.eye(3))
            for body in base.bodies_with_collision
        ]
        lower = np.min([footprint.lower for footprint in footprints], axis=0)
        upper = np.max([footprint.upper for footprint in footprints], axis=0)
        behind = np.array(
            [
                lower[0] - self.base_back_off / 2,
                (lower[1] + upper[1]) / 2,
                (lower[2] + upper[2]) / 2,
            ]
        )
        world_T_root = root.global_pose.to_np()
        return SpaceInTheWorld.from_middle(
            world_T_root[:3, :3] @ behind + world_T_root[:3, 3],
            world_T_root[:3, :3],
            np.array([self.base_back_off, upper[1] - lower[1], upper[2] - lower[2]]),
            self.world,
        )

    def create_steps(self) -> list[StatechartNode | Match]:
        return self.create_steps_leaving_the_sink()

    def create_steps_leaving_the_sink(self) -> list[StatechartNode | Match]:
        """
        :return: New nodes leading the hand out above and in front of the basin and
            backing the base off.
        """
        world = self.world
        world_T_tool = self.arm.end_effector.tool_frame.global_pose.to_np()
        forward = self.robot.root.global_pose.to_np()[:3, 0]
        forward[2] = 0.0
        forward = forward / np.linalg.norm(forward)
        sideways = np.cross([0.0, 0.0, 1.0], forward)
        axes = np.array([forward, sideways, [0.0, 0.0, 1.0]])
        basin = Extent.from_body(self.basin, world.root, axes)
        exit_along = np.array(
            [
                basin.lower[0] - self.exit_distance_in_front_of_the_basin,
                world_T_tool[:3, 3] @ sideways,
                basin.upper[2] + self.exit_height_above_the_basin,
            ]
        )
        world_T_exit = world_T_tool.copy()
        world_T_exit[:3, 3] = axes.T @ exit_along
        return [
            a(ReachThroughFreeSpaceAction)(
                goal=HomogeneousTransformationMatrix(
                    world_T_exit, reference_frame=world.root
                ).pose,
                arm=self.arm,
                clearance=self.path_clearance,
            ),
            a(BackOffAction)(distance=self.base_back_off),
        ]


# %% lifting the hand out of the sink


@dataclass(eq=False, repr=False, kw_only=True)
class LiftTheHandOutOfTheSinkAction(HasPreconditions, LeaveTheSinkAction):
    """
    Differs from :class:`LeaveTheSinkAction` in starting only with the hand inside the
    basin and the space behind the base free.
    """

    def preconditions(self) -> ConditionType:
        hand = variable(Body, self.arm.end_effector.bodies_with_collision)
        return and_(
            InsideOf(body=hand, other=self.basin),
            not_(
                self.space_behind_the_base().occupied(
                    self.world, list(self.robot.bodies)
                )
            ),
        )

    def create_steps_once_the_preconditions_hold(self) -> list[StatechartNode | Match]:
        return self.create_steps_leaving_the_sink()


# %% watching joints


@dataclass(eq=False, repr=False, kw_only=True)
class JointPositionBound(EndedByOwner, StatechartNode, ABC):
    """
    Observes whether a joint stands on one side of a position, as :class:`JointReading`
    reads it.
    """

    joint: ActiveConnection1DOF
    """
    The joint observed; the joint of the same name is read in the world the node runs
    in, which may be a copy of this joint's world.
    """

    position: float
    """
    The position the joint is compared with.
    """

    @abstractmethod
    def is_on_the_side_observed(self, reading: float) -> bool:
        """
        :param reading: Where the joint stands.
        :return: Whether that is on the side of :attr:`position` this node observes.
        """

    def on_tick(self) -> ObservationStateValues | None:
        world = self.context.world
        joint = world.get_connection_by_name(self.joint.name)
        if self.is_on_the_side_observed(JointReading(world).position(joint)):
            return ObservationStateValues.TRUE
        return ObservationStateValues.FALSE


@dataclass(eq=False, repr=False, kw_only=True)
class JointPositionAtLeast(JointPositionBound):
    """
    Observes whether a joint stands at :attr:`position` or beyond it.
    """

    def is_on_the_side_observed(self, reading: float) -> bool:
        return reading >= self.position


@dataclass(eq=False, repr=False, kw_only=True)
class JointPositionAtMost(JointPositionBound):
    """
    Observes whether a joint stands at :attr:`position` or below it.
    """

    def is_on_the_side_observed(self, reading: float) -> bool:
        return reading <= self.position


# %% touching the faucet's handle


@dataclass(eq=False, repr=False, kw_only=True)
class TouchesTheFaucetHandle(MovesToolCenterPoint, ABC):
    """
    An action whose gripper comes against a faucet's handle.
    """

    arm: Arm
    """
    The arm whose finger pushes the handle.
    """

    handle: Body
    """
    The faucet's handle.
    """

    lift_joint: ActiveConnection1DOF
    """
    The joint the handle is lifted about.
    """

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to touch the handle and the faucet around
            it.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def tool_goal_with_the_base_held(
        self, world_T_tool: np.ndarray, in_a_straight_line: bool
    ) -> StatechartNode:
        """
        :param world_T_tool: Where the tool frame is to go, in the world's root frame.
        :param in_a_straight_line: Whether the tool frame keeps to the line there.
        :return: The goal moving the tool frame there while the base stands still.
        """
        return self.tool_center_point_goal(
            HomogeneousTransformationMatrix(
                world_T_tool, reference_frame=self.world.root
            ).pose,
            self.arm,
            in_a_straight_line=in_a_straight_line,
            accompanying_goals=(
                HoldPose(
                    name="hold base",
                    root_link=self.world.root,
                    tip_link=self.robot.root,
                ),
            ),
        )

    def tool_goal_with_the_base_free(
        self, world_T_tool: np.ndarray, in_a_straight_line: bool
    ) -> StatechartNode:
        """
        :param world_T_tool: Where the tool frame is to go, in the world's root frame.
        :param in_a_straight_line: Whether the tool frame keeps to the line there.
        :return: The goal moving the tool frame there, the base moving along as needed.
        """
        return self.tool_center_point_goal(
            HomogeneousTransformationMatrix(
                world_T_tool, reference_frame=self.world.root
            ).pose,
            self.arm,
            in_a_straight_line=in_a_straight_line,
        )


@dataclass(eq=False, repr=False, kw_only=True)
class PushesTheFaucetHandle(TouchesTheFaucetHandle, GoalAction, ABC):
    """
    Moves the hand straight one way, with the base held still, until the faucet's handle
    reaches a position or the hand went a distance.
    """

    push_distance: float
    """
    How far the hand goes at most, in metres.
    """

    @abstractmethod
    def push_direction(self, handle: FaucetHandle) -> np.ndarray:
        """
        :param handle: The handle as it stands when the push starts.
        :return: The level direction the hand goes, in the world's root frame.
        """

    @abstractmethod
    def create_handle_position_reached(self) -> JointPositionBound:
        """
        :return: A new node observing whether the handle reached the position it is
            pushed to.
        """

    def create_goal(self) -> StatechartNode:
        handle = FaucetHandle.from_lift_joint(self.lift_joint, self.handle)
        world_T_pushed = self.arm.end_effector.tool_frame.global_pose.to_np().copy()
        world_T_pushed[:3, 3] += self.push_distance * self.push_direction(handle)
        return Parallel(
            name="push the handle",
            nodes=[
                self.tool_goal_with_the_base_held(
                    world_T_pushed, in_a_straight_line=True
                ),
                self.create_handle_position_reached(),
            ],
            minimum_success=1,
        )


@dataclass(eq=False, repr=False, kw_only=True)
class PutAFingerBesideTheFaucetHandleAction(TouchesTheFaucetHandle, SequentialAction):
    """
    Brings the hand in front of a faucet's handle, facing it, and straight in from there
    until one finger stands beside the handle, on the side the handle is pushed from.
    """

    finger_gap: float
    """
    How far the finger's collision stays from the handle's, in metres.
    """

    contact_height_share: float
    """
    How high up the handle's collision the tool frame comes in, as a share of its
    height.
    """

    insertion_depth: float
    """
    How far beyond the front of the handle's collision the tool frame comes in, in
    metres.
    """

    approach_distance: float
    """
    How far in front of where it comes in the tool frame starts its straight way in, in
    metres.
    """

    def create_steps(self) -> list[StatechartNode]:
        handle = FaucetHandle.from_lift_joint(self.lift_joint, self.handle)
        # the finger beside the handle is the one on the tool frame's positive x side,
        # since the tool frame's x-axis points against the push
        finger_offset = min(
            extent.lower[0]
            for extent in FingerExtents.from_arm(self.arm).extents
            if extent.lower[0] > 0.0
        )
        extent = handle.extent
        world_T_in = np.eye(4)
        world_T_in[:3, :3] = handle.tool_rotation
        world_T_in[:3, 3] = handle.point(
            np.array(
                [
                    extent.lower[0] - self.finger_gap + finger_offset,
                    extent.lower[1] + self.insertion_depth,
                    extent.lower[2]
                    + self.contact_height_share * (extent.upper[2] - extent.lower[2]),
                ]
            )
        )
        world_T_approach = world_T_in.copy()
        world_T_approach[:3, 3] -= self.approach_distance * handle.facing
        return [
            self.tool_goal_with_the_base_held(
                world_T_approach, in_a_straight_line=False
            ),
            self.tool_goal_with_the_base_held(world_T_in, in_a_straight_line=True),
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class PushTheFaucetHandleAction(PushesTheFaucetHandle):
    """
    Moves the hand straight along the way that lifts a faucet's handle, until the handle
    is lifted to a position or the hand went a distance.
    """

    lift_goal: float
    """
    How far the handle is to be lifted, in radians.
    """

    def push_direction(self, handle: FaucetHandle) -> np.ndarray:
        return handle.push

    def create_handle_position_reached(self) -> JointPositionBound:
        return JointPositionAtLeast(
            name="handle lifted", joint=self.lift_joint, position=self.lift_goal
        )


@dataclass(eq=False, repr=False, kw_only=True)
class PushTheFaucetHandleDownAction(PushesTheFaucetHandle):
    """
    Moves the hand straight against the way that lifts a faucet's handle, until the
    handle is lowered to a position or the hand went a distance.
    """

    lowered_goal: float
    """
    How far down the handle is to be lowered, in radians of lift.
    """

    def push_direction(self, handle: FaucetHandle) -> np.ndarray:
        return -handle.push

    def create_handle_position_reached(self) -> JointPositionBound:
        return JointPositionAtMost(
            name="handle lowered", joint=self.lift_joint, position=self.lowered_goal
        )


@dataclass(eq=False, repr=False, kw_only=True)
class PutTheLowerFingerBesideTheLeverBarAction(
    TouchesTheFaucetHandle, SequentialAction
):
    """
    Brings the hand in front of a faucet's lever, its fingers spread up the lever's bar,
    and straight in from there until the lower finger stands beside the bar, on the side
    pushing the bar's top lowers the lever.

    The base moves along as needed.
    """

    bar_share: float
    """
    How far up the bar the lower finger comes, as a share of the bar's length.
    """

    finger_gap: float
    """
    How far the lower finger's collision stays from the bar's, in metres.
    """

    approach_distance: float
    """
    How far in front of where it comes in the tool frame starts its straight way in, in
    metres.
    """

    def create_steps(self) -> list[StatechartNode]:
        handle = FaucetHandle.from_lift_joint(self.lift_joint, self.handle)
        bar = LeverBar.from_handle(self.handle)
        beside = np.cross(handle.facing, bar.up_the_bar)
        rotation = np.column_stack([bar.up_the_bar, beside, handle.facing])
        # the tool frame's x-axis points up the bar, so the lower finger is the one on
        # its negative x side, and the bar is on the finger's negative y side
        lower_finger = [
            extent
            for extent in FingerExtents.from_arm(self.arm).extents
            if extent.lower[0] + extent.upper[0] < 0.0
        ]
        finger_side = np.array(
            [
                (
                    min(extent.lower[0] for extent in lower_finger)
                    + max(extent.upper[0] for extent in lower_finger)
                )
                / 2,
                min(extent.lower[1] for extent in lower_finger),
                (
                    min(extent.lower[2] for extent in lower_finger)
                    + max(extent.upper[2] for extent in lower_finger)
                )
                / 2,
            ]
        )
        world_T_in = np.eye(4)
        world_T_in[:3, :3] = rotation
        world_T_in[:3, 3] = (
            bar.point_up_the_bar(self.bar_share)
            + (bar.radius + self.finger_gap) * beside
            - rotation @ finger_side
        )
        world_T_approach = world_T_in.copy()
        world_T_approach[:3, 3] -= self.approach_distance * handle.facing
        return [
            self.tool_goal_with_the_base_free(
                world_T_approach, in_a_straight_line=False
            ),
            self.tool_goal_with_the_base_free(world_T_in, in_a_straight_line=True),
        ]


@dataclass(eq=False, repr=False, kw_only=True)
class PullTheHandBackFromTheFaucetHandleAction(TouchesTheFaucetHandle, GoalAction):
    """
    Moves the hand straight back from a faucet's handle, towards the robot in front of
    it, with the base held still.
    """

    retreat_distance: float
    """
    How far the hand goes back, in metres.
    """

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        Differs from :meth:`TouchesTheFaucetHandle.create_temporary_collision_rules` in
        allowing the whole arm to touch, since pushing the handle may leave a link of
        the arm against the faucet.
        """
        arm = set(self.arm.bodies_with_collision)
        return [
            AllowCollisionBetweenGroups(
                body_group_a=list(arm),
                body_group_b=[
                    body for body in self.world.bodies_with_collision if body not in arm
                ],
            )
        ]

    def create_goal(self) -> StatechartNode:
        handle = FaucetHandle.from_lift_joint(self.lift_joint, self.handle)
        world_T_back = self.arm.end_effector.tool_frame.global_pose.to_np().copy()
        world_T_back[:3, 3] -= self.retreat_distance * handle.facing
        return self.tool_goal_with_the_base_held(world_T_back, in_a_straight_line=True)


# %% moving the faucet's handle with a finger


@dataclass(eq=False, repr=False, kw_only=True)
class FaucetMovedWithAFingerAction(HasPreconditions, ABC):
    """
    An action that stands in front of a faucet's handle and moves the handle with one
    finger.
    """

    arm: Arm
    """
    The arm whose finger pushes the handle.
    """

    handle: Body
    """
    The faucet's handle.
    """

    lift_joint: ActiveConnection1DOF
    """
    The joint the handle is lifted about.
    """

    basin: Body
    """
    The sink's basin, which the hand has to be out of when the action starts; the bodies
    below it in the world, such as the faucet's spout and handle, are what the gripper
    may touch.
    """

    stand_distance: float = 0.84
    """
    How far in front of the lift joint's axis the robot's root stands, in metres.
    """

    stand_sideways_offset: float = 0.0
    """
    How far along the push from the middle of the handle the robot's root stands, in
    metres.
    """

    def create_navigation_in_front_of_the_handle(self) -> NavigateAction:
        """
        :return: A new action driving the robot to stand in front of the handle, facing
            it.
        """
        handle = FaucetHandle.from_lift_joint(self.lift_joint, self.handle)
        return NavigateAction(
            target_location=HomogeneousTransformationMatrix(
                handle.standing_pose_in_front(
                    self.stand_distance, self.stand_sideways_offset
                ),
                reference_frame=self.world.root,
            ).pose
        )

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper to touch the sink's bodies, the handle and
            the spout among them, from one step to the next.
        """
        return [
            AllowCollisionBetweenGroups(
                body_group_a=list(self.arm.end_effector.bodies_with_collision),
                body_group_b=self.sink_bodies,
            )
        ]

    @property
    def sink_bodies(self) -> list[Body]:
        """
        :return: The bodies with collision of the sink, the basin and those below it.
        """
        branch = set(self.world.get_kinematic_structure_entities_of_branch(self.basin))
        return [body for body in self.world.bodies_with_collision if body in branch]

    def hand_is_outside_the_basin(self) -> ConditionType:
        """
        :return: The statement that no body of the gripper is inside the basin.
        """
        hand = variable(Body, self.arm.end_effector.bodies_with_collision)
        return not_(InsideOf(body=hand, other=self.basin))

    def faucet_is_on(self) -> FaucetIsOn:
        """
        :return: The statement that the water runs.
        """
        return FaucetIsOn(lift_joint=variable(ActiveConnection1DOF, [self.lift_joint]))


# %% turning the faucet on with a finger


@dataclass(eq=False, repr=False, kw_only=True)
class TurnOnTheFaucetWithAFingerAction(FaucetMovedWithAFingerAction):
    """
    Stands in front of a faucet's handle, lowers the torso, opens the gripper, puts one
    finger beside the handle and pushes the handle up with it until the water runs.
    """

    temperature_joint: ActiveConnection1DOF
    """
    The joint the handle is swung about to choose the water's temperature.
    """

    torso_state: TorsoState = TorsoState.LOW
    """
    Where the torso is moved before the arm reaches.
    """

    finger_gap: float = 0.01
    """
    See :attr:`PutAFingerBesideTheFaucetHandleAction.finger_gap`.
    """

    contact_height_share: float = 0.79
    """
    See :attr:`PutAFingerBesideTheFaucetHandleAction.contact_height_share`.
    """

    insertion_depth: float = 0.04
    """
    See :attr:`PutAFingerBesideTheFaucetHandleAction.insertion_depth`.
    """

    approach_distance: float = 0.13
    """
    See :attr:`PutAFingerBesideTheFaucetHandleAction.approach_distance`.
    """

    placement_tolerance: float = 0.02
    """
    How close the tool frame has to come to where it is led in front of the handle and
    beside it, in metres.
    """

    placement_orientation_tolerance: float = 0.35
    """
    How far the tool frame may stay turned away from facing the handle there, in
    radians; the wrist may not turn far enough to face it squarely.
    """

    push_distance: float = 0.1
    """
    See :attr:`PushTheFaucetHandleAction.push_distance`.
    """

    lift_goal: float = 0.47
    """
    How far the handle is to be lifted, in radians; the water runs above 0.4 rad, and
    the handle stops at 0.5 rad.
    """

    temperature_tolerance: float = 0.02
    """
    How far the handle may be swung about :attr:`temperature_joint` meanwhile, in
    radians.
    """

    temperature_at_the_start: float = field(init=False, default=0.0)
    """
    Where :attr:`temperature_joint` stood when the action started.
    """

    def on_start(self) -> None:
        super().on_start()
        self.temperature_at_the_start = JointReading(self.world).position(
            self.temperature_joint
        )

    def preconditions(self) -> ConditionType:
        return and_(
            not_(self.faucet_is_on()),
            self.hand_is_outside_the_basin(),
            not_(
                self.space_for_the_finger().occupied(
                    self.world, [*self.robot.bodies, *self.sink_bodies]
                )
            ),
        )

    def space_for_the_finger(self) -> SpaceInTheWorld:
        """
        :return: The space beside the handle the finger comes into: as thick as the
            finger, :attr:`finger_gap` from the handle, from its
            front to :attr:`insertion_depth` beyond it, at the height the finger comes
            in.
        """
        handle = FaucetHandle.from_lift_joint(self.lift_joint, self.handle)
        finger = FingerExtents.from_arm(self.arm).extents
        thickness = max(extent.upper[0] - extent.lower[0] for extent in finger)
        height = max(extent.upper[1] - extent.lower[1] for extent in finger)
        extent = handle.extent
        lower = np.array(
            [
                extent.lower[0] - self.finger_gap - thickness,
                extent.lower[1],
                extent.lower[2]
                + self.contact_height_share * (extent.upper[2] - extent.lower[2])
                - height / 2,
            ]
        )
        size = np.array([thickness, self.insertion_depth, height])
        return SpaceInTheWorld.from_middle(
            handle.point(lower + size / 2), handle.axes.T, size, self.world
        )

    def create_steps_once_the_preconditions_hold(self) -> list[StatechartNode | Match]:
        gripper = self.arm.end_effector
        return [
            self.create_navigation_in_front_of_the_handle(),
            MoveTorsoAction(torso_state=self.torso_state),
            SetGripperAction(
                configuration=gripper.default_configuration(GripperState.OPEN)
            ),
            a(PutAFingerBesideTheFaucetHandleAction)(
                arm=self.arm,
                handle=self.handle,
                lift_joint=self.lift_joint,
                finger_gap=self.finger_gap,
                contact_height_share=self.contact_height_share,
                insertion_depth=self.insertion_depth,
                approach_distance=self.approach_distance,
                position_threshold=self.placement_tolerance,
                orientation_threshold=self.placement_orientation_tolerance,
            ),
            a(PushTheFaucetHandleAction)(
                arm=self.arm,
                handle=self.handle,
                lift_joint=self.lift_joint,
                lift_goal=self.lift_goal,
                push_distance=self.push_distance,
            ),
            FunctionCall(
                name="check the water runs",
                function=self.raise_unless_the_water_runs_as_it_did,
            ),
        ]

    def raise_unless_the_water_runs_as_it_did(self) -> None:
        """
        :raises FaucetNotTurnedOn: If the handle stands short of :attr:`lift_goal`.
        :raises LeverSwungAside: If the handle was swung about :attr:`temperature_joint`
            by more than :attr:`temperature_tolerance`.
        """
        reading = JointReading(self.world)
        lift = reading.position(self.lift_joint)
        if lift < self.lift_goal:
            raise FaucetNotTurnedOn(
                lift_joint=self.lift_joint, position=lift, goal=self.lift_goal
            )
        change = (
            reading.position(self.temperature_joint) - self.temperature_at_the_start
        )
        if abs(change) > self.temperature_tolerance:
            raise LeverSwungAside(
                joint=self.temperature_joint,
                change=change,
                tolerance=self.temperature_tolerance,
            )


# %% turning the faucet off with a finger


@dataclass(eq=False, repr=False, kw_only=True)
class TurnOffTheFaucetWithAFingerAction(FaucetMovedWithAFingerAction):
    """
    Stands in front of a faucet's lever, opens the gripper, puts the lower finger beside
    the lever's bar and pushes the bar down with it until the water stops, then pulls
    the hand back and leaves the sink.
    """

    bar_share: float = 2 / 3
    """
    See :attr:`PutTheLowerFingerBesideTheLeverBarAction.bar_share`.
    """

    finger_gap: float = 0.01
    """
    See :attr:`PutTheLowerFingerBesideTheLeverBarAction.finger_gap`.
    """

    approach_distance: float = 0.13
    """
    See :attr:`PutTheLowerFingerBesideTheLeverBarAction.approach_distance`.
    """

    push_distance: float = 0.06
    """
    See :attr:`PushTheFaucetHandleDownAction.push_distance`.
    """

    lowered_goal: float = 0.005
    """
    How far down the lever is to be lowered, in radians of lift; the water stops at or
    below 0.4 rad, and the lever stops at 0 rad.
    """

    retreat_distance: float = 0.13
    """
    See :attr:`PullTheHandBackFromTheFaucetHandleAction.retreat_distance`.
    """

    def preconditions(self) -> ConditionType:
        return and_(self.faucet_is_on(), self.hand_is_outside_the_basin())

    def create_steps_once_the_preconditions_hold(self) -> list[StatechartNode | Match]:
        gripper = self.arm.end_effector
        return [
            self.create_navigation_in_front_of_the_handle(),
            SetGripperAction(
                configuration=gripper.default_configuration(GripperState.OPEN)
            ),
            a(PutTheLowerFingerBesideTheLeverBarAction)(
                arm=self.arm,
                handle=self.handle,
                lift_joint=self.lift_joint,
                bar_share=self.bar_share,
                finger_gap=self.finger_gap,
                approach_distance=self.approach_distance,
            ),
            a(PushTheFaucetHandleDownAction)(
                arm=self.arm,
                handle=self.handle,
                lift_joint=self.lift_joint,
                lowered_goal=self.lowered_goal,
                push_distance=self.push_distance,
            ),
            FunctionCall(
                name="check the water stopped",
                function=self.raise_unless_the_lever_is_down,
            ),
            a(PullTheHandBackFromTheFaucetHandleAction)(
                arm=self.arm,
                handle=self.handle,
                lift_joint=self.lift_joint,
                retreat_distance=self.retreat_distance,
            ),
            a(LeaveTheSinkAction)(arm=self.arm, basin=self.basin),
        ]

    def raise_unless_the_lever_is_down(self) -> None:
        """
        :raises FaucetNotTurnedOff: If the lever stands above :attr:`lowered_goal`.
        """
        lift = JointReading(self.world).position(self.lift_joint)
        if lift > self.lowered_goal:
            raise FaucetNotTurnedOff(
                lift_joint=self.lift_joint, position=lift, goal=self.lowered_goal
            )
