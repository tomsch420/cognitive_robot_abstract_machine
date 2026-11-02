"""
A video of an episode as the PR2 sees it, from the camera in its head.

The episode runs without MuJoCo. While it runs, the pose of every body that shows
something and may move, the robot's and the kitchen's doors, knobs and objects, is
recorded at every change of the world's state. Afterwards a forked process builds one
MuJoCo scene of the kitchen as it stood at the start, in which those bodies hang freely,
replays the recorded poses and renders them from the head camera.

The PR2's head stays still while it acts; the camera turns towards the gripper as the
head would, and looks ahead while the arms are parked.
"""

from __future__ import annotations

import logging
import multiprocessing
import time
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from typing_extensions import List

from semantic_digital_twin.callbacks.callback import StateChangeCallback
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    FixedConnection,
)
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

HEAD_CAMERA_FRAME = "wide_stereo_optical_frame"
"""
The PR2's frame of the camera in its head.
"""

HEAD_LINK = "head_pan_link"
"""
The PR2's link that its head, and everything on it, hangs from; left out of the video,
as the camera turns without it.
"""


@dataclass(eq=False)
class PoseRecorder(StateChangeCallback):
    """
    Keeps the global pose of some bodies at every change of the world's state that
    moves any of them.
    """

    bodies: List[Body] = field(kw_only=True)
    """
    The bodies whose poses are kept.
    """

    poses: List[np.ndarray] = field(kw_only=True, default_factory=list)
    """
    The poses, one stack of the bodies' 4x4 transforms per change.
    """

    def on_state_change(self, **kwargs) -> None:
        world = self._world
        poses = np.stack(
            [world.compute_forward_kinematics_np(world.root, body) for body in self.bodies]
        ).astype(np.float32)
        if self.poses and np.abs(poses - self.poses[-1]).max() < 1e-5:
            return
        self.poses.append(poses)


@dataclass
class HeadCameraVideo:
    """
    Records an episode as it runs, and renders it from the PR2's head camera.
    """

    world: World
    """
    The world the episode runs in.
    """

    robot: AbstractRobot
    """
    The robot whose head the video is seen from.
    """

    tool_frame: Body
    """
    The tool frame of the acting arm, which the camera turns towards.
    """

    width: int = 640
    """
    The width of the video, in pixels.
    """

    height: int = 480
    """
    Its height, in pixels.
    """

    frames_per_second: int = 25
    """
    The rate it plays back at.
    """

    maximum_frames: int = 1500
    """
    How many frames it has at most; a longer recording is played faster.
    """

    field_of_view: float = 75.0
    """
    The camera's vertical field of view, in degrees.
    """

    start_world: World = field(init=False, repr=False)
    """
    A copy of the world as it stood when recording began.
    """

    shown_bodies: List[Body] = field(init=False, repr=False)
    """
    The bodies that show something and may move.
    """

    hidden_bodies: List[Body] = field(init=False, repr=False)
    """
    The bodies of the head, which the camera would see from inside.
    """

    recorder: PoseRecorder = field(init=False, repr=False)
    """
    Keeps the poses of the shown bodies, of the camera and of the tool frame.
    """

    def __post_init__(self) -> None:
        world = self.world
        robot_entities = world.get_kinematic_structure_entities_of_branch(self.robot.root)
        movable = set(robot_entities)
        for connection in world.connections:
            if isinstance(connection, FixedConnection) or connection.child in movable:
                continue
            movable.add(connection.child)
            movable.update(
                world.compute_descendent_child_kinematic_structure_entities(
                    connection.child
                )
            )
        head = world.get_body_by_name(HEAD_LINK)
        hidden = {head} | set(
            world.compute_descendent_child_kinematic_structure_entities(head)
        )
        self.hidden_bodies = [
            body for body in hidden if isinstance(body, Body) and body.visual.shapes
        ]
        self.shown_bodies = [
            body
            for body in world.kinematic_structure_entities
            if body in movable
            and body not in hidden
            and isinstance(body, Body)
            and body.visual.shapes
        ]
        self.start_world = deepcopy(world)
        self.recorder = PoseRecorder(
            _world=world,
            bodies=[
                *self.shown_bodies,
                world.get_body_by_name(HEAD_CAMERA_FRAME),
                self.tool_frame,
                self.robot.root,
            ],
        )
        self.recorder.on_state_change()
        logger.info("Recording %d bodies", len(self.shown_bodies))

    def stop(self) -> None:
        """
        Stop recording; stopping again does nothing.
        """
        self.recorder.stop()

    def render(self, path: Path, timeout: float = 1800.0) -> bool:
        """
        Render the recording into a video file, in a process of its own: MuJoCo hangs
        when it shares one with a plan that ran.

        :param path: The video file written.
        :param timeout: How long rendering may take, in seconds.
        :return: Whether the video was written.
        """
        self.stop()
        child = multiprocessing.get_context("fork").Process(
            target=self._render_in_this_process, args=(path,)
        )
        child.start()
        child.join(timeout)
        if child.is_alive():
            child.kill()
            logger.warning("Rendering %s timed out", path)
            return False
        return child.exitcode == 0 and path.exists()

    def frame_poses(self) -> np.ndarray:
        """
        :return: The recorded poses of the frames shown, evenly picked if there are more
            than :attr:`maximum_frames`, with the last held for a second.
        """
        poses = np.stack(self.recorder.poses)
        if len(poses) > self.maximum_frames:
            poses = poses[np.linspace(0, len(poses) - 1, self.maximum_frames).astype(int)]
        return np.concatenate(
            [poses, np.repeat(poses[-1:], self.frames_per_second, axis=0)]
        )

    def gaze_points(self, poses: np.ndarray, smoothing: float = 0.12) -> np.ndarray:
        """
        :param poses: The recorded poses of each frame.
        :param smoothing: How far the gaze moves towards where it is drawn each frame.
        :return: The point the camera looks at in each frame: the gripper once it is
            away from the body, otherwise ahead of the robot at a counter's height.
        """
        tool, base = poses[:, -2, :3, 3], poses[:, -1]
        ahead = base[:, :3, 3] + 1.2 * base[:, :3, 0]
        ahead[:, 2] = 0.8
        reach = np.linalg.norm(tool[:, :2] - base[:, :2, 3], axis=1)
        drawn = np.where((reach > 0.55)[:, None], tool, ahead)
        gaze = np.empty_like(drawn)
        gaze[0] = drawn[0]
        for index in range(1, len(drawn)):
            gaze[index] = gaze[index - 1] + smoothing * (drawn[index] - gaze[index - 1])
        return gaze

    def _render_in_this_process(self, path: Path) -> None:
        import imageio.v2 as imageio
        import mujoco
        from scipy.spatial.transform import Rotation

        from semantic_digital_twin.adapters.multi_sim import MujocoSim

        install_rendering_patches()
        began = time.time()
        poses = self.frame_poses()
        gaze = self.gaze_points(poses)
        replay = self.start_world
        names = [body.name for body in self.shown_bodies]
        bodies = [replay.get_body_by_name(name) for name in names]
        with replay.modify_world():
            # nothing is driven in a replay, and the joints the actuators drive go
            for actuator in list(replay.actuators):
                replay.remove_actuator(actuator)
            for body in bodies:
                replay.remove_connection(body.parent_connection)
                replay.add_connection(
                    Connection6DoF.create_with_dofs(replay, replay.root, body)
                )
        # everything else stays where it stands; MuJoCo would start its joints at zero
        frozen = [
            connection
            for connection in replay.connections
            if not isinstance(connection, (FixedConnection, Connection6DoF))
        ]
        transforms = [
            replay.compute_forward_kinematics(connection.parent, connection.child)
            for connection in frozen
        ]
        with replay.modify_world():
            for connection, transform in zip(frozen, transforms):
                parent, child = connection.parent, connection.child
                replay.remove_connection(connection)
                replay.add_connection(
                    FixedConnection(
                        parent=parent,
                        child=child,
                        parent_T_connection_expression=transform,
                    )
                )
        # a picture needs no collision geometry, whose convex hulls MuJoCo takes long for
        for body in replay.bodies:
            body.collision.shapes.clear()
        for body in self.hidden_bodies:
            replay.get_body_by_name(body.name).visual.shapes.clear()
        simulation = MujocoSim(world=replay, headless=True)
        simulation.start_stepped_simulation()
        model, data = simulation.simulator._mj_model, simulation.simulator._mj_data
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, self.width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, self.height)
        model.vis.global_.fovy = self.field_of_view
        model.vis.headlight.ambient[:] = [0.45, 0.45, 0.45]
        model.vis.headlight.diffuse[:] = [0.7, 0.7, 0.7]
        addresses = []
        for name in names:
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name.name)
            addresses.append(int(model.jnt_qposadr[model.body_jntadr[body_id]]))
        renderer = mujoco.Renderer(model, self.height, self.width)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        logger.info("MuJoCo scene built in %.0f s", time.time() - began)
        frames = []
        for frame_poses, looked_at in zip(poses, gaze):
            for address, pose in zip(addresses, frame_poses):
                x, y, z, w = Rotation.from_matrix(pose[:3, :3]).as_quat()
                data.qpos[address : address + 7] = [*pose[:3, 3], w, x, y, z]
            mujoco.mj_kinematics(model, data)
            eye = frame_poses[-3, :3, 3]
            looking = looked_at - eye
            distance = max(float(np.linalg.norm(looking)), 0.05)
            looking = looking / distance
            camera.lookat[:] = eye + 0.3 * looking
            camera.distance = 0.3
            camera.azimuth = float(np.degrees(np.arctan2(looking[1], looking[0])))
            camera.elevation = float(np.degrees(np.arcsin(np.clip(looking[2], -1, 1))))
            renderer.update_scene(data, camera)
            frames.append(renderer.render())
        simulation.stop_simulation()
        imageio.mimwrite(path, frames, fps=self.frames_per_second, macro_block_size=8)
        logger.info(
            "Wrote %s, %d frames, in %.0f s", path, len(frames), time.time() - began
        )


def install_rendering_patches(
    converted_textures: Path = Path("/tmp/robocasa_png_textures"),
) -> None:
    """
    Make the MuJoCo scene buildable for rendering: let MuJoCo balance the inertias of
    the PR2's links that violate the triangle inequality, give it PNG copies of the
    JPEG textures some of RoboCasa's objects use, as it reads PNG only, and leave out
    the contacts collision rules exclude, which a picture has none of.

    :param converted_textures: Where the PNG copies are kept.
    """
    from PIL import Image

    from semantic_digital_twin.adapters.multi_sim import MujocoBuilder

    start_build = MujocoBuilder._start_build
    build_tendons = MujocoBuilder._build_tendons

    def start_build_for_rendering(self, file_path):
        start_build(self, file_path)
        self.spec.compiler.balanceinertia = True
        self.spec.compiler.usethread = False

    def build_tendons_then_convert_jpeg_textures(self):
        build_tendons(self)
        converted_textures.mkdir(parents=True, exist_ok=True)
        for texture in self.spec.textures:
            source = Path(texture.file)
            if source.suffix.lower() not in (".jpg", ".jpeg"):
                continue
            converted = converted_textures / f"{abs(hash(str(source)))}.png"
            if not converted.exists():
                Image.open(source).convert("RGB").save(converted)
            texture.file = str(converted)

    MujocoBuilder._build_contact_exclusions = lambda self, *args, **kwargs: None
    MujocoBuilder._start_build = start_build_for_rendering
    MujocoBuilder._build_tendons = build_tendons_then_convert_jpeg_textures
