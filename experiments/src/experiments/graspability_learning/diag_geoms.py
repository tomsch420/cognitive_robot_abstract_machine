import sys
sys.path.insert(0, "experiments/src")

from experiments.graspability_learning.stage1_fixed_sample import build_world
from semantic_digital_twin.adapters.multi_sim import MujocoBuilder

world, cube = build_world()
MujocoBuilder().build_world(world=world, file_path="/tmp/diag_scene.xml")

import mujoco
model = mujoco.MjModel.from_xml_path("/tmp/diag_scene.xml")
print("ngeom:", model.ngeom, "nbody:", model.nbody)
for i in range(model.nbody):
    print("body", i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i))
for i in range(model.ngeom):
    body_id = model.geom_bodyid[i]
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    print(
        i, "body=", body_name, "type=", model.geom_type[i], "group=", model.geom_group[i],
        "size=", model.geom_size[i], "rgba=", model.geom_rgba[i],
    )
