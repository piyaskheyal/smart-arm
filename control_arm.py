#!/usr/bin/env python3
# control_arm_live_smooth.py
# ------------------------------------------------------------
# Isaac Sim 5.1.0 – Live Terminal Input + Smooth Motion
# ------------------------------------------------------------

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api.objects.ground_plane import GroundPlane
from pxr import UsdLux, Gf, Sdf
import threading
import queue
import signal
import sys

# ------------------------------------------------------------------
# CONFIG
# ------------------------------------------------------------------
USD_PATH = "/home/kheyal/dev/robotics/smart-arm-v2/Models/armv1.usd"
ROBOT_PRIM_PATH = "/World/so101_new_calib"

# ------------------------------------------------------------------
# SETUP: GROUND + LIGHT + CAMERA
# ------------------------------------------------------------------
world = World(stage_units_in_meters=1.0)
GroundPlane(prim_path="/World/Ground", z_position=0.0)

# === LIGHT ===
stage = world.scene.stage
light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Light"))
light.CreateIntensityAttr(1000)
light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

# === CAMERA ===
set_camera_view(
    eye=[5.0, 0.0, 2.0],
    target=[0.0, 0.0, 0.5],
    camera_prim_path="/OmniverseKit_Persp"
)

# === LOAD ROBOT ===
print(f"Loading robot: {USD_PATH}")
add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT_PRIM_PATH)

# === ARTICULATION ===
print("Creating Articulation...")
arm = Articulation(prim_paths_expr=f"{ROBOT_PRIM_PATH}/.*", name="arm")

# === INITIALIZE ===
print("Starting physics...")
world.reset()
arm.initialize()

# Wait for joint states
for _ in range(30):
    world.step(render=True)
    if arm.get_joint_velocities() is not None:
        break

print(f"DOFs: {arm.num_dof}")
print("Joints:", arm.dof_names)

# ------------------------------------------------------------------
# INPUT THREAD
# ------------------------------------------------------------------
input_queue = queue.Queue()

def input_thread():
    print("\n" + "="*60)
    print("=== LIVE JOINT CONTROL ===")
    print("Enter 6 numbers: shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper")
    print("Example: 0.5 -0.3 1.0 0.2 0.0 0.04")
    print("Type 'quit' to exit")
    print("="*60)
    print(">>> ", end="", flush=True)
    while True:
        try:
            line = sys.stdin.readline().strip()
            if line:
                input_queue.put(line)
        except:
            break

threading.Thread(target=input_thread, daemon=True).start()

# ------------------------------------------------------------------
# CTRL+C
# ------------------------------------------------------------------
def signal_handler(sig, frame):
    print("\n\nShutting down...")
    simulation_app.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# ------------------------------------------------------------------
# MOTION CONTROL
# ------------------------------------------------------------------
current_pose = arm.get_joint_positions().copy()
target_pose = current_pose.copy()
motion_steps = 60
motion_counter = 0
is_moving = False

def start_motion(new_target):
    global target_pose, current_pose, motion_counter, is_moving
    target_pose = np.array(new_target, dtype=float)
    current_pose = arm.get_joint_positions().copy()
    motion_counter = 0
    is_moving = True

# ------------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------------
print("\nSimulation ready. Type joint values below.\n")

try:
    while True:
        # === INPUT ===
        try:
            line = input_queue.get_nowait()
            if line.lower() == "quit":
                print("Quit received.")
                break

            values = [float(x) for x in line.split()]
            if len(values) != 6:
                print("ERROR: Need exactly 6 numbers!")
                print(">>> ", end="", flush=True)
                continue

            print(f"Moving to: {values}")
            start_motion(values)

        except queue.Empty:
            pass
        except ValueError:
            print("ERROR: Invalid numbers!")
            print(">>> ", end="", flush=True)

        # === SMOOTH MOTION ===
        if is_moving:
            motion_counter += 1
            t = min(motion_counter / motion_steps, 1.0)
            t = t * t * (3 - 2 * t)  # Smoothstep
            interp = current_pose * (1 - t) + target_pose * t
            arm.set_joint_positions(interp)
            if t >= 1.0:
                is_moving = False
                current_pose = interp.copy()
        else:
            # HOLD via USD drives
            arm.set_joint_positions(target_pose)

        world.step(render=True)

finally:
    print("Closing simulation...")
    simulation_app.close()