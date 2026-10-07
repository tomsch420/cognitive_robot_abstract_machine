"""
The PR2 picks an object up from a table in MuJoCo, holding it by contact alone.

Run it with ``python -m experiments.physical_pick_up.pr2_pick_up``; see ``--help``.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import dataclass, field
from datetime import timedelta
from pathlib import Path

import mujoco
import numpy as np
from typing_extensions import Dict, Optional
from uuid import UUID

from experiments.physical_pick_up.physical_simulation_preparation import (
    PR2PhysicalSimulationPreparation,
)
from experiments.physical_pick_up.pick_up import (
    FilmingSimulationPacer,
    PhysicalPickUp,
)
from experiments.physical_pick_up.objects import ObjectChoice
from experiments.physical_pick_up.scene import ObjectOnTableScene
from semantic_digital_twin.grasping.surface_grasp import GraspResult
from giskardpy.executor import SteppedSimulationPacer
from semantic_digital_twin.adapters.multi_sim import MujocoCamera, MujocoSim
from semantic_digital_twin.api import RobotSpecification
from semantic_digital_twin.datastructures.definitions import (
    StaticJointState,
    TorsoState,
)
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.robots.robot_parts import Arm

CAMERA_NAME = "pick_up_camera"
"""
Name of the camera the experiment is filmed with.
"""


@dataclass
class PR2PickUpExperiment:
    """
    A PR2 standing in front of a table picks up the object on it with its left arm, in a
    MuJoCo simulation stepped in lockstep with Giskard's control loop.

    Unless told otherwise, the object is grasped by its description's default grasp.
    Every attempt starts from the same state of the world, so attempts can follow one
    another.
    """

    scene: ObjectOnTableScene = field(default_factory=ObjectOnTableScene)
    """
    The table and the object.
    """

    grip_torque: float = 8.0
    """
    The torque the finger servos press the object with, in newton meters.
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

    robot: PR2 = field(init=False)
    """
    The PR2, spawned into the scene's world and prepared for physical simulation.
    """

    _start_state: Dict[UUID, np.ndarray] = field(init=False)
    """
    The state of every degree of freedom of the world before the first attempt.
    """

    def __post_init__(self):
        self.robot = RobotSpecification(PR2).spawn(self.scene.world)
        PR2PhysicalSimulationPreparation(
            robot=self.robot, grip_torque=self.grip_torque
        ).apply()
        self._move_to_start_configuration()
        self._add_camera()
        self._start_state = dict(self.scene.world.state.items())

    @property
    def arm(self) -> Arm:
        """
        :return: The arm the object is picked up with.
        """
        return self.robot.left_arm

    def run(
        self,
        grasp: Optional[GraspCandidate] = None,
        video_path: Optional[Path] = None,
    ) -> GraspResult:
        """
        Simulate one pick-up attempt.

        :param grasp: The grasp to take the object by; ``None`` takes the default grasp
            of the object's description.
        :param video_path: Where to write a video of the attempt; ``None`` films nothing.
        :return: What happened to the object.
        """
        self._restore_start_state()
        simulation = MujocoSim(
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
        simulation.start_stepped_simulation()
        with ExitStack() as cleanup:
            cleanup.callback(simulation.stop_simulation)
            simulation.step_simulation(self.settling_duration)
            pacer = self._pacer(simulation, video_path)
            result = PhysicalPickUp(
                robot=self.robot,
                arm=self.arm,
                grasp=(
                    grasp
                    if grasp is not None
                    else self.scene.object_description.default_grasp.grasp_candidate(
                        self.scene.graspable
                    )
                ),
                pacer=pacer,
                time_limit=self.time_limit,
            ).perform()
            if isinstance(pacer, FilmingSimulationPacer):
                pacer.recorded_video.write(video_path)
        return result

    def _restore_start_state(self) -> None:
        """
        Put the robot and the object back where they were before the first attempt.
        """
        world = self.scene.world
        for degree_of_freedom_id, values in self._start_state.items():
            world.state[degree_of_freedom_id] = values
        world.notify_state_change()

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

    def _move_to_start_configuration(self) -> None:
        """
        Park both arms and raise the torso, so that the left arm reaches the table from
        above.
        """
        world = self.scene.world
        for arm in self.robot.all_arms:
            arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)
        self.robot.mobile_base.torso.get_joint_state_by_type(TorsoState.HIGH).apply_to(
            world
        )
        world.notify_state_change()

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


def main() -> None:
    """
    Run the experiment once from the command line and report the outcome.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    object_choice = ObjectChoice(parser)
    object_choice.add_arguments()
    parser.add_argument("--video", type=Path, help="write a video of the attempt here")
    parser.add_argument(
        "--grip-torque",
        type=float,
        default=PR2PickUpExperiment.grip_torque,
        help="torque the fingers press with, in newton meters",
    )
    parser.add_argument(
        "--show", action="store_true", help="open MuJoCo's viewer window"
    )
    arguments = parser.parse_args()
    result = PR2PickUpExperiment(
        scene=ObjectOnTableScene(
            object_description=object_choice.description(arguments)
        ),
        grip_torque=arguments.grip_torque,
        headless=not arguments.show,
    ).run(video_path=arguments.video)
    print(result)


if __name__ == "__main__":
    main()
