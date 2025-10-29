#!/usr/bin/env python3
# control_arm_your_robot.py
# ------------------------------------------------------------
# Isaac Sim 5.1.0 – Your Robot + Fixed Base + Direct Control
# ------------------------------------------------------------

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.prims import Articulation
from pxr import UsdPhysics
import sys

# ------------------------------------------------------------------
# 1. CONFIG
# ------------------------------------------------------------------
USD_PATH = "/home/kheyal/dev/robotics/smart-arm-v2/Models/armv1.usd"
ROBOT_PRIM_PATH = "/World/so101_new_calib"  # Parent of base_link
BASE_LINK_PATH = "/World/so101_new_calib/base_link"

# Joint order must match USD
JOINT_POSITIONS_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 joints
JOINT_POSITIONS_REACH = [0.5, -0.3, 1.0, 0.2, 0.0, 0.04]

# ------------------------------------------------------------------
# 2. Setup World
# ------------------------------------------------------------------
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

print(f"Loading robot: {USD_PATH}")
add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT_PRIM_PATH)

# ------------------------------------------------------------------
# 3. Fix Base (ONLY if not fixed in USD)
# ------------------------------------------------------------------
stage = my_world.scene.stage
base_prim = stage.GetPrimAtPath(BASE_LINK_PATH)

# ------------------------------------------------------------------
# 4. Create Articulation
# ------------------------------------------------------------------
print("Creating Articulation...")
arm = Articulation(prim_paths_expr=f"{ROBOT_PRIM_PATH}/.*", name="my_arm")  # Match all under parent
# OR: arm = Articulation(prim_paths_expr=BASE_LINK_PATH, name="my_arm")

# ------------------------------------------------------------------
# 5. Initialize Physics
# ------------------------------------------------------------------
print("Resetting world...")
my_world.reset()  # Critical: starts physics

# Wait one frame for DOFs
my_world.step(render=True)

# ------------------------------------------------------------------
# 6. Validate DOFs
# ------------------------------------------------------------------
print(f"DOFs: {arm.num_dof}")
print("Joint names:", arm.dof_names)

if arm.num_dof != 6:
    print("ERROR: Expected 6 DOFs!")
    simulation_app.close()
    sys.exit(1)

# ------------------------------------------------------------------
# 7. RUN DEMO
# ------------------------------------------------------------------
poses = [
    JOINT_POSITIONS_HOME,
    JOINT_POSITIONS_REACH,
    JOINT_POSITIONS_HOME,
]

print("\n=== RUNNING DEMO ===")
for i, pos in enumerate(poses):
    print(f"\n[Step {i+1}] Setting joint positions: {pos}")
    arm.set_joint_positions(np.array(pos))
    
    for _ in range(120):
        my_world.step(render=True)

print("\nDemo complete!")
simulation_app.close()