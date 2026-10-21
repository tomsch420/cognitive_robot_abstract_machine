"""
Writing a video of a MuJoCo simulation while something else drives it.

The simulation is already running and being stepped by a plan, so frames are taken from
it rather than from a mirror of the world built for the purpose, and they are written
straight to the file instead of being kept until the run ends.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import imageio
import mujoco
import numpy as np
from typing_extensions import Optional

from semantic_digital_twin.adapters.mujoco_video_recording import VideoResolution


@dataclass
class ViewerCamera:
    """
    Where the viewer looks from, so the window opens on the table rather than on the
    middle of the apartment.
    """

    look_at: tuple = (4.6, 3.9, 0.9)
    """
    The point the camera orbits, in metres.
    """

    distance: float = 2.6
    """
    How far the camera stands from that point, in metres.
    """

    azimuth: float = 150.0
    """
    Which way the camera faces, in degrees.
    """

    elevation: float = -20.0
    """
    How far the camera looks down, in degrees.
    """

    def aim(self, camera: mujoco.MjvCamera) -> None:
        """
        Point a camera of MuJoCo's own at the table.

        :param camera: The camera to point.
        """
        camera.lookat[:] = self.look_at
        camera.distance = self.distance
        camera.azimuth = self.azimuth
        camera.elevation = self.elevation

    def free_camera(self) -> mujoco.MjvCamera:
        """
        :return: A camera that orbits the table, for rendering a frame outside the
            viewer's own window.
        """
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.aim(camera)
        return camera


# %% recording what the run looks like


@dataclass
class SceneRecording:
    """
    Where and how a run is written to a video file.
    """

    path: Path
    """
    The video file to write.
    """

    frames_per_second: int = 30
    """
    How many frames one second of the recording holds.
    """

    resolution: VideoResolution = field(
        default_factory=lambda: VideoResolution(width=1280, height=720)
    )
    """
    How big a frame is.
    """

    @property
    def simulated_seconds_per_frame(self) -> float:
        """
        :return: How much simulated time one frame stands for.
        """
        return 1.0 / self.frames_per_second


@dataclass
class SceneRecorder:
    """
    Writes the scene to a video file while the simulation runs.

    A frame is taken for every :attr:`SceneRecording.simulated_seconds_per_frame` of
    simulated time rather than of real time, so the recording plays back at the speed
    the robot moves and not at the speed the physics is computed at.

    Rendering happens on a thread of its own, which is the thread that holds the
    graphics context, and is used as a context manager around whatever is to be
    recorded. Finishing writes out whatever frames the simulated time already covers, so
    the recording is as long as the stretch it was open for.
    """

    simulation: object
    """
    The running simulation to render.
    """

    camera: ViewerCamera
    """
    Where the recording is shot from.
    """

    recording: SceneRecording
    """
    Where and how to write it.
    """

    polling_interval: float = 0.005
    """
    How long the writing thread waits before asking the clock again, in seconds.
    """

    frames_written: int = field(init=False, default=0)
    """
    How many frames have reached the file so far.
    """

    asked_to_finish: threading.Event = field(
        init=False, default_factory=threading.Event
    )
    """
    Set to tell the writing thread the recording is over.
    """

    ready: threading.Event = field(init=False, default_factory=threading.Event)
    """
    Set once the writing thread can draw, which it cannot until it holds a graphics
    context of its own.
    """

    writing_thread: Optional[threading.Thread] = field(init=False, default=None)
    """
    The thread that renders and writes.
    """

    def __enter__(self) -> SceneRecorder:
        """
        Start writing.

        The model carries the size of the buffer a frame is rendered into, which is too
        small for the asked-for frame until it is told otherwise.

        :return: The recorder, so the frame count can be read afterwards.
        """
        visual = self.simulation.simulator._mj_model.vis.global_
        visual.offwidth = self.recording.resolution.width
        visual.offheight = self.recording.resolution.height
        self.recording.path.parent.mkdir(parents=True, exist_ok=True)
        self.writing_thread = threading.Thread(target=self._write, daemon=True)
        self.writing_thread.start()
        self.ready.wait()
        return self

    def __exit__(self, *_) -> None:
        """
        Finish the file, so it is playable whether the plan ran to its end or raised.
        """
        self.asked_to_finish.set()
        self.writing_thread.join()

    def _write(self) -> None:
        """
        Render and write a frame whenever the simulation has run far enough for one.
        """
        renderer = mujoco.Renderer(
            self.simulation.simulator._mj_model,
            self.recording.resolution.height,
            self.recording.resolution.width,
        )
        camera = self.camera.free_camera()
        with imageio.get_writer(
            self.recording.path,
            fps=self.recording.frames_per_second,
            macro_block_size=None,
        ) as writer:
            due = self._simulated_time()
            self.ready.set()
            while True:
                if self._simulated_time() >= due:
                    writer.append_data(self._frame(renderer, camera))
                    self.frames_written += 1
                    due += self.recording.simulated_seconds_per_frame
                    continue
                if self.asked_to_finish.is_set():
                    break
                time.sleep(self.polling_interval)
        renderer.close()

    def _simulated_time(self) -> float:
        """
        :return: How far the simulation has run, in seconds.
        """
        return float(self.simulation.simulator._mj_data.time)

    def _frame(self, renderer: mujoco.Renderer, camera: mujoco.MjvCamera) -> np.ndarray:
        """
        The simulation steps on another thread, so the state a frame is drawn from is
        read under the lock that guards it.

        :param renderer: The renderer to draw with.
        :param camera: The camera to draw from.
        :return: What the camera sees now.
        """
        with self.simulation.simulator._model_lock:
            renderer.update_scene(self.simulation.simulator._mj_data, camera)
        return renderer.render()
