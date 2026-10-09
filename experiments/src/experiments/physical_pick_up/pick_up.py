"""
Picking an object up in a physical simulation, where nothing but the contact between the
fingers and the object holds it.
"""

from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum

import numpy as np
from scipy.spatial.transform import Rotation
from typing_extensions import List

from giskardpy.executor import Executor, SteppedSimulationPacer
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import LifeCycleValues
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.graph_node import EndMotion, MotionStatechartNode
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.adapters.mujoco_video_recording import (
    RecordedVideo,
    VideoResolution,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.grasping.surface_grasp import GraspResult
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.spatial_types.spatial_types import Pose

# %% filming


@dataclass
class FilmingSimulationPacer(SteppedSimulationPacer):
    """
    Steps the simulation between two ticks like its parent and films it while doing so.
    """

    camera_name: str = field(kw_only=True)
    """
    Name of the simulated camera to film with.
    """

    resolution: VideoResolution = field(
        default_factory=lambda: VideoResolution(width=640, height=480), kw_only=True
    )
    """
    Size of the frames, which MuJoCo limits to its offscreen buffer of 640 by 480
    pixels.
    """

    ticks_per_frame: int = field(default=2, kw_only=True)
    """
    How many ticks pass between two frames.
    """

    frames: List[np.ndarray] = field(default_factory=list, init=False)
    """
    The frames filmed so far.
    """

    _ticks_since_frame: int = field(default=0, init=False)
    """
    Ticks that passed since the last frame was filmed.
    """

    def sleep(self) -> None:
        super().sleep()
        self._ticks_since_frame += 1
        if self._ticks_since_frame < self.ticks_per_frame:
            return
        self._ticks_since_frame = 0
        self.frames.append(
            self.simulation.simulator.capture_rgb(
                camera_name=self.camera_name,
                height=self.resolution.height,
                width=self.resolution.width,
            ).result
        )

    @property
    def recorded_video(self) -> RecordedVideo:
        """
        :return: The frames filmed so far, played back at the speed they were simulated.
        """
        return RecordedVideo(
            frames=self.frames,
            frames_per_second=round(self.target_frequency / self.ticks_per_frame),
        )


# %% the pick-up


class PickUpStep(StrEnum):
    """
    The steps of a pick-up, in the order they are performed.
    """

    OPEN_GRIPPER = "open gripper"
    MOVE_ABOVE_GRASP = "move above grasp"
    REACH_GRASP = "reach grasp"
    CLOSE_GRIPPER = "close gripper"
    LIFT = "lift"


@dataclass
class PhysicalPickUp:
    """
    One attempt of a robot to pick an object up by a grasp in a stepped MuJoCo
    simulation.

    The gripper is commanded to close completely. The object stops the fingers before
    they get there, so their servos keep pressing with all the torque they are allowed,
    and the friction that pressure creates is all that lifts the object: the object is
    never attached to the gripper.

    The robot's mobile base stays where it is; only the joints between the robot's root
    and the tool frame move.

    Where the object sits in the gripper is taken once the gripper has closed, when the
    lift starts, and again after the hold; the difference is the slip. An attempt that
    never gets to lift takes the first of the two when it gives up.
    """

    robot: AbstractRobot
    """
    The robot picking up, prepared for physical simulation.
    """

    arm: Arm
    """
    The arm to pick up with.
    """

    grasp: GraspCandidate
    """
    The grasp to take the object by.
    """

    pacer: SteppedSimulationPacer
    """
    Steps the simulation of the robot's world between two control cycles.
    """

    approach_clearance: float = 0.12
    """
    How far above the grasp the gripper stops before it descends onto it, in meters.
    """

    lift_height: float = 0.2
    """
    How far above the grasp the gripper lifts the object, in meters.
    """

    approach_reference_velocity: float = 0.1
    """
    The speed Giskard aims for while moving above the grasp, in meters per second.
    """

    contact_reference_velocity: float = 0.05
    """
    The speed Giskard aims for while descending onto the grasp and while lifting, in
    meters per second.
    """

    closing_velocity: float = 0.3
    """
    The fastest the finger joints close, in radians per second.
    """

    hold_duration: timedelta = timedelta(seconds=2)
    """
    How long the lifted object is held before the outcome is measured, so that an object
    slipping out of the fingers has time to fall.
    """

    minimum_rise: float = 0.05
    """
    How far the object has to have risen to count as raised, in meters.
    """

    control_frequency: float = 50
    """
    Control cycles per simulated second.
    """

    time_limit: timedelta = timedelta(seconds=60)
    """
    The simulated time after which an attempt that is still moving is given up on.
    """

    _lift_step: MotionStatechartNode = field(init=False)
    """
    The last step of the motion, which starts once the gripper has closed.
    """

    _gripper_T_grasp_when_closed: np.ndarray = field(init=False)
    """
    Where the grasp, fixed to the object, sat in the gripper when the lift started.
    """

    def perform(self) -> GraspResult:
        """
        Pick the object up and hold it.

        :return: What happened to the object.
        """
        world = self.robot._world
        world_T_resting_object = self._world_T_object()
        motion_statechart = self._motion_statechart()
        executor = Executor(
            context=MotionStatechartContext(
                world=world,
                qp_controller_config=QPControllerConfig(
                    target_frequency=self.control_frequency
                ),
            ),
            pacer=self.pacer,
        )
        with ExitStack() as cleanup:
            cleanup.callback(executor.context.cleanup)
            cleanup.callback(motion_statechart.cleanup_nodes, context=executor.context)
            cleanup.callback(executor.set_velocity_acceleration_jerk_to_zero)
            executor.compile(motion_statechart=motion_statechart)
            motion_completed = self._tick_until_end(executor)
        self._wait(self.hold_duration)
        world_T_held_object = self._world_T_object()
        displacement = world_T_held_object[:3, 3] - world_T_resting_object[:3, 3]
        slip = (
            np.linalg.inv(self._gripper_T_grasp_when_closed) @ self._gripper_T_grasp()
        )
        return GraspResult(
            object_raised=bool(displacement[2] >= self.minimum_rise),
            motion_completed=motion_completed,
            object_displacement=Vector3(*displacement),
            object_rotation=Vector3(
                *self._rotation_vector(
                    world_T_held_object[:3, :3] @ world_T_resting_object[:3, :3].T
                )
            ),
            translational_slip=Vector3(*slip[:3, 3]),
            rotational_slip=Vector3(*self._rotation_vector(slip[:3, :3])),
        )

    def _motion_statechart(self) -> MotionStatechart:
        """
        :return: The steps of :class:`PickUpStep` one after another, ending the motion
            after the last.
        """
        gripper = self.arm.end_effector
        self._lift_step = self._move_tool_frame(
            PickUpStep.LIFT, self.lift_height, self.contact_reference_velocity
        )
        steps = Sequence(
            nodes=[
                JointPositionList(
                    goal_state=gripper.get_joint_state_by_type(GripperState.OPEN),
                    name=PickUpStep.OPEN_GRIPPER,
                ),
                self._move_tool_frame(
                    PickUpStep.MOVE_ABOVE_GRASP,
                    self.approach_clearance,
                    self.approach_reference_velocity,
                ),
                self._move_tool_frame(
                    PickUpStep.REACH_GRASP, 0.0, self.contact_reference_velocity
                ),
                JointPositionList(
                    goal_state=gripper.get_joint_state_by_type(GripperState.CLOSE),
                    name=PickUpStep.CLOSE_GRIPPER,
                    max_velocity=self.closing_velocity,
                ),
                self._lift_step,
            ]
        )
        motion_statechart = MotionStatechart()
        motion_statechart.add_nodes([steps, EndMotion.when_true(steps)])
        return motion_statechart

    def _move_tool_frame(
        self, step: PickUpStep, height_above_grasp: float, reference_velocity: float
    ) -> CartesianPose:
        """
        :param step: The step this motion is.
        :param height_above_grasp: How far above the grasp the tool frame is to stop.
        :param reference_velocity: The speed Giskard aims for.
        :return: The task moving the tool frame there, oriented as the grasp asks.
        """
        return CartesianPose(
            root_link=self.robot.root,
            tip_link=self.arm.end_effector.tool_frame,
            goal_pose=self.arm.end_effector.tool_frame_goal(
                self._pose_above_grasp(height_above_grasp)
            ),
            reference_linear_velocity=reference_velocity,
            name=step,
        )

    def _pose_above_grasp(self, height: float) -> Pose:
        """
        :param height: How far above the grasp the pose is.
        :return: The grasp frame where the object stands now, raised by ``height`` and
            fixed in the world, so that it stays put when the object moves.
        """
        world = self.robot._world
        world_T_grasp = world.transform(self.grasp.grasp_pose, world.root).to_np()
        world_T_grasp[2, 3] += height
        return HomogeneousTransformationMatrix(
            world_T_grasp, reference_frame=world.root
        ).pose

    def _tick_until_end(self, executor: Executor) -> bool:
        """
        Tick the control loop, stepping the simulation in between, until the motion ends
        or :attr:`time_limit` has passed.

        :param executor: The compiled control loop.
        :return: Whether the motion ended in time.
        """
        tick_limit = round(self.time_limit.total_seconds() * self.control_frequency)
        closed = False
        for _ in range(tick_limit):
            executor.tick()
            self.pacer.sleep()
            if (
                not closed
                and self._lift_step.life_cycle_state != LifeCycleValues.NOT_STARTED
            ):
                self._gripper_T_grasp_when_closed = self._gripper_T_grasp()
                closed = True
            if executor.motion_statechart.is_end_motion():
                return True
        if not closed:
            self._gripper_T_grasp_when_closed = self._gripper_T_grasp()
        return False

    def _wait(self, duration: timedelta) -> None:
        """
        Let the simulation run without commanding anything new.

        :param duration: How much simulated time passes.
        """
        for _ in range(round(duration.total_seconds() * self.control_frequency)):
            self.pacer.sleep()

    @staticmethod
    def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
        """
        :param rotation: A rotation matrix.
        :return: Its rotation axis scaled by its angle in radians.
        """
        return Rotation.from_matrix(rotation).as_rotvec()

    def _gripper_T_grasp(self) -> np.ndarray:
        """
        :return: The pose of the grasp frame, fixed to the object, in the tool frame, as
            simulated.
        """
        return (
            np.linalg.inv(
                self._simulated_pose(self.arm.end_effector.tool_frame.name.name)
            )
            @ self._world_T_object()
            @ self.grasp.grasp_pose.to_np()
        )

    def _world_T_object(self) -> np.ndarray:
        """
        :return: The pose of the body the grasp is placed on, as simulated.
        """
        return self._simulated_pose(self.grasp.graspable.root.name.name)

    def _simulated_pose(self, body_name: str) -> np.ndarray:
        """
        :param body_name: The body to look up.
        :return: Its pose in the simulation's world frame.
        """
        simulator = self.pacer.simulation.simulator
        pose = np.eye(4)
        pose[:3, 3] = simulator.get_body_position(body_name=body_name).result
        pose[:3, :3] = Rotation.from_quat(
            simulator.get_body_quaternion(body_name=body_name).result,
            scalar_first=True,
        ).as_matrix()
        return pose
