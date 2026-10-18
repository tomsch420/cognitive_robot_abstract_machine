"""
A robot picks an object up in MuJoCo, holding it by contact alone.

Run it with ``python -m experiments.physical_pick_up.pick_up_experiment``; see
``--help``.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import timedelta
from pathlib import Path

import mujoco
import numpy as np
from typing_extensions import List, Optional

from experiments.physical_pick_up.exceptions import NoDefaultGraspError
from experiments.physical_pick_up.pick_up import (
    FilmingSimulationPacer,
    PhysicalPickUp,
)
from experiments.physical_pick_up.robots import ObjectPlacement
from experiments.physical_pick_up.scene import PickUpScene, PickUpSceneChoice
from semantic_digital_twin.grasping.surface_grasp import GraspResult
from giskardpy.executor import SteppedSimulationPacer
from semantic_digital_twin.adapters.multi_sim import MujocoCamera, MujocoSim
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.grasp_trials import GraspPerformer

# %% the experiment

CAMERA_NAME = "pick_up_camera"
"""
Name of the camera the experiment is filmed with.
"""


@dataclass
class PickUpExperiment:
    """
    The robot of a scene picks up the object in it, in a MuJoCo simulation stepped in
    lockstep with Giskard's control loop.

    Unless told otherwise, the object is grasped by its description's default grasp
    where the scene first put it. Every attempt starts from the state the world was in
    before it, so attempts can follow one another, and all of them run in one
    simulation that is reset in between.
    """

    scene: PickUpScene = field(default_factory=PickUpScene)
    """
    The robot and the object.
    """

    headless: bool = True
    """
    Whether to run without MuJoCo's viewer window.
    """

    step_size: timedelta = timedelta(milliseconds=2)
    """
    The simulated time of one physics step.
    """

    settling_duration: timedelta = timedelta(seconds=1)
    """
    How long the object is given to come to rest on the table before the robot moves.
    """

    time_limit: timedelta = timedelta(seconds=60)
    """
    The simulated time after which an attempt that is still moving is given up on.
    """

    simulation: Optional[MujocoSim] = field(default=None, init=False)
    """
    The simulation the attempts run in, built from the world as it is before the first
    attempt; ``None`` until then.
    """

    def __post_init__(self):
        self._add_camera()

    def run(
        self,
        grasp: Optional[GraspCandidate] = None,
        video_path: Optional[Path] = None,
        placement: Optional[ObjectPlacement] = None,
    ) -> GraspResult:
        """
        Simulate one pick-up attempt.

        :param grasp: The grasp to take the object by; ``None`` takes the default grasp
            of the object's description.
        :param video_path: Where to write a video of the attempt; ``None`` films nothing.
        :param placement: Where the object stands for this attempt; ``None`` leaves it
            where the scene first put it.
        :return: What happened to the object.
        :raises NoDefaultGraspError: If no grasp is given and the object's description
            knows none that lifts it.
        """
        description = self.scene.object_description
        if grasp is None and description.default_grasp is None:
            raise NoDefaultGraspError(body_name=description.body_name)
        if grasp is None:
            grasp = description.default_grasp.grasp_candidate(self.scene.graspable)
        simulation = self._started_simulation()
        with self.scene.world.reset_state_context():
            simulation.reset_simulation()
            if placement is not None:
                self.scene.place_object(placement)
            simulation.synchronizer.command_actuators_from_world_state()
            simulation.step_simulation(self.settling_duration)
            pacer = self._pacer(simulation, video_path)
            result = PhysicalPickUp(
                robot=self.scene.robot,
                arm=self.scene.arm,
                grasp=grasp,
                pacer=pacer,
                time_limit=self.time_limit,
            ).perform()
            if isinstance(pacer, FilmingSimulationPacer):
                pacer.recorded_video.write(video_path)
        return result

    def _started_simulation(self) -> MujocoSim:
        """
        :return: :attr:`simulation`, built and started the first time it is asked for.
        """
        if self.simulation is not None:
            return self.simulation
        self.simulation = MujocoSim(
            world=self.scene.world,
            headless=self.headless,
            step_size=self.step_size.total_seconds(),
            # Bodies keep the mass and inertia they state; only those stating none
            # are weighed by their geometry.
            inertiafromgeom=mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_AUTO,
            # An implicit integrator keeps the stiff servos stable at this step size;
            # elliptic friction cones with a raised impedance ratio and a few no-slip
            # iterations are what MuJoCo's documentation recommends against objects
            # creeping out of a grasp.
            integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
            cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
            impratio=10,
            noslip_iterations=3,
        )
        self.simulation.start_stepped_simulation()
        return self.simulation

    def _pacer(
        self, simulation: MujocoSim, video_path: Optional[Path]
    ) -> SteppedSimulationPacer:
        """
        :param simulation: The running simulation.
        :param video_path: Where a video is to be written, if anywhere.
        :return: What steps the simulation between two control cycles, filming if a
            video was asked for.
        """
        if video_path is None:
            return SteppedSimulationPacer(simulation)
        return FilmingSimulationPacer(simulation, camera_name=CAMERA_NAME)

    def _add_camera(self) -> None:
        """
        Put a camera into the scene that looks at the object and the arm from the far
        side of the table.
        """
        world = self.scene.world
        object_position = world.compute_forward_kinematics_np(
            world.root, self.scene.graspable.root
        )[:3, 3]
        reach = np.array([0.35, 0.35, 0.35])
        pose = MujocoCamera.overview_pose(
            np.array([object_position - reach, object_position + reach]),
            distance_factor=1.2,
        )
        x, y, z, w = pose.quaternion.to_np().tolist()
        world.root.add_simulator_property(
            MujocoCamera(
                name=CAMERA_NAME,
                body=world.root,
                position=pose.position.to_np()[:3].tolist(),
                quaternion=[w, x, y, z],
            )
        )


# %% performing grasps with the experiment


@dataclass
class PickUpGraspPerformer(GraspPerformer):
    """
    Performs every grasp with the robot of a pick-up experiment, starting each attempt
    from the same state of the world, and with the object standing anywhere in its
    pick-up area if a source of randomness is given.
    """

    experiment: PickUpExperiment
    """
    The robot and the object the grasps are performed with.
    """

    generator: Optional[np.random.Generator] = None
    """
    Draws where the object stands for each attempt; ``None`` leaves it where the scene
    first put it.
    """

    maximum_yaw: float = 0.0
    """
    How far the object may be turned either way about the vertical axis, in radians.
    """

    video_directory: Optional[Path] = None
    """
    Where to film each attempt; ``None`` films nothing.
    """

    placements: List[ObjectPlacement] = field(default_factory=list, init=False)
    """
    Where the object stood in each attempt so far, in order.
    """

    def perform(self, grasp: GraspCandidate) -> GraspResult:
        placement = self._placement()
        video_path = (
            None
            if self.video_directory is None
            else self.video_directory / f"trial_{len(self.placements):03d}.mp4"
        )
        self.placements.append(placement)
        return self.experiment.run(grasp, video_path, placement)

    def _placement(self) -> ObjectPlacement:
        """
        :return: Where the object stands in the next attempt.
        """
        area = self.experiment.scene.pick_up_area
        if self.generator is None:
            return area.middle()
        return area.random_placement(self.generator, self.maximum_yaw)


# %% the command line


def main() -> None:
    """
    Run the experiment once from the command line and report the outcome.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    scene_choice = PickUpSceneChoice(parser)
    scene_choice.add_arguments()
    parser.add_argument("--video", type=Path, help="write a video of the attempt here")
    parser.add_argument(
        "--show", action="store_true", help="open MuJoCo's viewer window"
    )
    arguments = parser.parse_args()
    result = PickUpExperiment(
        scene=scene_choice.scene(arguments), headless=not arguments.show
    ).run(video_path=arguments.video)
    print(result)


if __name__ == "__main__":
    main()
