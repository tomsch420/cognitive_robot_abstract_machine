"""
Record a video of the Stage-1 fixed-candidate grasp using MujocoVideoRecorder.

Reuses build_world()/default_grasp_pose()/solve_approach_joints() from
stage1_fixed_sample.py so the scene and the Giskard-solved approach pose are identical
to the verified physics run; only the execution loop changes, to drive the recorder's
own internal MujocoSim mirror (via advance_simulation) instead of a bare MujocoSimulator.
"""
import sys
sys.path.insert(0, "experiments/src")

from pathlib import Path
import numpy as np

from experiments.graspability_learning.domain_model import Cube, GraspCandidate
from experiments.graspability_learning.stage1_fixed_sample import (
    build_world, default_grasp_pose, solve_approach_joints, HOVER_ARM,
)
from semantic_digital_twin.adapters.mujoco_video_recording import (
    MujocoVideoRecorder, VideoResolution,
)
from semantic_digital_twin.adapters.multi_sim import MujocoCamera
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.spatial_types.spatial_types import HomogeneousTransformationMatrix

# This MJCF fixture's arm links (link0..link7, hand) carry joints/inertials but zero
# geoms at all -- confirmed by inspection (mj_model.ngeom only ever covers box/floor/our
# own added finger pads). What rendered as blocky, striped shapes before this fix was
# MuJoCo's own default inertia-box visualization, not the arm. Add simple box "visual"
# stand-ins per link purely for this video -- rough proportions, not a real robot mesh.
_ARM_LINK_BOX_SIZES = {
    "link0": (0.18, 0.18, 0.14),
    "link1": (0.09, 0.09, 0.2),
    "link2": (0.09, 0.09, 0.16),
    "link3": (0.08, 0.08, 0.2),
    "link4": (0.08, 0.08, 0.16),
    "link5": (0.07, 0.07, 0.22),
    "link6": (0.07, 0.07, 0.1),
    "link7": (0.06, 0.06, 0.08),
    "hand": (0.09, 0.09, 0.08),
}


def add_arm_visual_boxes(world) -> None:
    for name, size in _ARM_LINK_BOX_SIZES.items():
        body = world.get_body_by_name(name)
        box = Box(
            scale=Scale(*size),
            color=Color(0.75, 0.55, 0.1, 1.0),
            origin=HomogeneousTransformationMatrix.from_xyz_rpy(reference_frame=body),
        )
        body.visual = ShapeCollection([box], reference_frame=body)

OUT_PATH = Path("/tmp/grasp_video.mp4")


def main():
    world, cube = build_world()
    add_arm_visual_boxes(world)
    approach_pose = default_grasp_pose(world)
    candidate = GraspCandidate(
        graspable=cube, end_effector=None,
        approach_pose=approach_pose, aperture=0.04, closing_effort=40.0,
    )
    arm_angles = solve_approach_joints(world, candidate.approach_pose)

    # MujocoVideoRecorder's auto-attached overview camera frames the *whole* world's
    # bounding box, which includes this fixture's huge floor plane (-50..50 m in x/y) --
    # that puts the camera ~170 m away from the 5 cm cube, rendering nothing but black
    # background. MujocoCamera.overview_pose's distance/orientation convention also
    # turned out not to frame an off-center, non-cubic box well (tested: produced a
    # blurry, near-grazing-angle shot). Build the camera pose manually instead: a plain
    # look-at from a fixed eye point down at the grasp region.
    from scipy.spatial.transform import Rotation

    eye = np.array([1.4, -1.15, 0.95])
    target = np.array([0.5, 0.0, 0.15])
    forward = (target - eye) / np.linalg.norm(target - eye)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    true_up = np.cross(right, forward)
    # MuJoCo camera convention: local +X right, +Y up, looks down local -Z.
    rotation_matrix = np.column_stack([right, true_up, -forward])
    quat_xyzw = Rotation.from_matrix(rotation_matrix).as_quat()

    camera = MujocoCamera(
        name="grasp_overview_camera",
        body=world.root,
        position=eye.tolist(),
        quaternion=[quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
        resolution=[640.0, 480.0],
    )
    world.root.simulator_additional_properties.append(camera)

    recorder = MujocoVideoRecorder(
        world=world, frames_per_second=30, resolution=VideoResolution(width=640, height=480),
        camera=camera,
    )
    recorder.start()
    sim = recorder._multi_sim.simulator

    def set_ctrl(arm_angles, finger_target):
        for i, angle in enumerate(arm_angles):
            sim.get_actuator(f"actuator{i + 1}").result.ctrl[0] = angle
        sim.get_actuator("actuator8").result.ctrl[0] = finger_target

    open_width = candidate.aperture

    print("Phase A: settle at hover, then move to grasp configuration")
    set_ctrl(HOVER_ARM, open_width)
    recorder.advance_simulation(0.8)
    set_ctrl(arm_angles, open_width)
    recorder.advance_simulation(2.0)

    print("Teleporting cube into the open gripper")
    left_finger_pos = np.array(sim.get_body_position("left_finger").result)
    right_finger_pos = np.array(sim.get_body_position("right_finger").result)
    grasp_point = ((left_finger_pos + right_finger_pos) / 2).tolist()
    sim.set_body_position("box", grasp_point)
    sim.set_body_quaternion("box", [1, 0, 0, 0])
    recorder.advance_simulation(0.6)

    print("Phase B: closing fingers")
    finger_actuator_id = sim.get_all_actuator_names().result.index("actuator8")
    sim._mj_model.actuator_forcerange[finger_actuator_id] = [
        -candidate.closing_effort, candidate.closing_effort,
    ]
    close_target = 0.005
    set_ctrl(arm_angles, close_target)
    recorder.advance_simulation(1.2)

    print("Phase C: lifting")
    lift_arm = list(arm_angles)
    lift_arm[1] -= 0.35
    set_ctrl(lift_arm, close_target)
    recorder.advance_simulation(3.0)

    print(f"Captured {recorder.captured_frame_count} frames")
    video = recorder.stop()
    video.write(OUT_PATH)
    print(f"WROTE VIDEO: {OUT_PATH}")


if __name__ == "__main__":
    main()
