import sys
sys.path.insert(0, "experiments/src")
import numpy as np

from experiments.graspability_learning.stage1_fixed_sample import build_world
from semantic_digital_twin.spatial_computations.raytracer import RayTracer
from semantic_digital_twin.adapters.multi_sim import MujocoCamera, MujocoSim, MujocoSynchronizer

world, cube = build_world()
bounds = RayTracer(world).scene.bounds
print("BOUNDS:", bounds)

pose = MujocoCamera.overview_pose(np.asarray(bounds))
print("CAMERA POSE pos:", pose.to_position().to_np(), "quat:", pose.to_quaternion().to_np())

quaternion_xyzw = pose.to_quaternion().to_np().tolist()
camera = MujocoCamera(
    name="diag_camera",
    body=world.root,
    position=pose.to_position().to_np()[:3].tolist(),
    quaternion=[quaternion_xyzw[3]] + quaternion_xyzw[:3],
    resolution=[640.0, 480.0],
)
world.root.simulator_additional_properties.append(camera)

sim_wrapper = MujocoSim(world=world, headless=True)
sim_wrapper.synchronizer.sync_rate_hz = MujocoSynchronizer.UNTHROTTLED_SYNC_RATE_HZ
sim_wrapper.simulator.start(simulate_in_thread=False, render_in_thread=False)
import mujoco
sim = sim_wrapper.simulator
with sim._model_lock:
    mujoco.mj_forward(sim._mj_model, sim._mj_data)
    result = sim.capture_rgb(camera_name="diag_camera", height=480, width=640)
img = result.result
print("IMG shape", img.shape, "min/max/mean", img.min(), img.max(), img.mean())

import imageio.v2 as imageio
imageio.imwrite("/tmp/diag_frame.png", img)

print("all body names:", sim.get_all_body_names().result)
print("all camera names:", sim._mj_model.camera(0).name if sim._mj_model.ncam else "NO CAMERAS IN MODEL")
print("ncam:", sim._mj_model.ncam)
print("nlight:", sim._mj_model.nlight)
sim_wrapper.simulator.stop()
