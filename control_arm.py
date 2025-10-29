#!/usr/bin/env python3
# control_arm_live_perfect.py
# ------------------------------------------------------------
# Isaac Sim 5.1.0 – Live Control + Tutorial-Style Scene
# ------------------------------------------------------------

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api.objects.ground_plane import GroundPlane
from pxr import Sdf, UsdLux, Gf
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

# === GRID GROUND PLANE (LIKE TUTORIALS) ===
GroundPlane(prim_path="/World/GridGround", z_position=0.0)

# === DISTANT LIGHT (SUN) ===
stage = world.scene.stage
distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
distant_light.CreateIntensityAttr(500)  # Bright sun
distant_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
distant_light.CreateAngleAttr(0.5)  # Soft shadows

# === CAMERA VIEW (LIKE getting_started_robot.py) ===
set_camera_view(
    eye=[5.0, 0.0, 1.5],
    target=[0.0, 0.0, 1.0],
    camera_prim_path="/OmniverseKit_Persp"
)

# === LOAD ROBOT ===
print(f"Loading robot: {USD_PATH}")
add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT_PRIM_PATH)

# === ARTICULATION ===
print("Creating Articulation...")
arm = Articulation(prim_paths_expr=f"{ROBOT_PRIM_PATH}/.*", name="arm")

# === PHYSICS START ===
print("Starting physics...")
world.reset()
world.step(render=True)  # Initialize DOFs

print(f"DOFs: {arm.num_dof}")
print("Joints:", arm.dof_names)

# ------------------------------------------------------------------
# INPUT THREAD
# ------------------------------------------------------------------
input_queue = queue.Queue()

def input_thread():
    print("\n=== LIVE CONTROL ===")
    print("Enter 6 numbers (e.g., 0.5 -0.3 1.0 0.2 0.0 0.04) → press Enter")
    print("Type 'quit' to exit\n")
    print(">>> ", end="", flush=True)
    while True:
        try:
            line = sys.stdin.readline().strip()
            if not line:
                continue
            input_queue.put(line)
        except:
            break

threading.Thread(target=input_thread, daemon=True).start()

# ------------------------------------------------------------------
# CTRL+C HANDLER
# ------------------------------------------------------------------
def signal_handler(sig, frame):
    print("\n\nShutting down...")
    simulation_app.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# ------------------------------------------------------------------
# MOTION CONTROL (SMOOTH)
# ------------------------------------------------------------------
current_pose = np.zeros(6)
target_pose = np.zeros(6)
motion_steps = 60  # frames to reach target
motion_counter = 0
is_moving = False

def start_motion(new_target):
    global target_pose, motion_counter, is_moving, current_pose
    target_pose = np.array(new_target)
    current_pose = arm.get_joint_positions()
    motion_counter = 0
    is_moving = True

# ------------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------------
print("Simulation running. Type poses below.\n")

try:
    while True:
        # === PROCESS INPUT ===
        try:
            line = input_queue.get_nowait()
            if line.lower() == "quit":
                print("Quit command received.")
                break

            values = [float(x) for x in line.split()]
            if len(values) != 6:
                print("ERROR: Need exactly 6 numbers!")
                print(">>> ", end="", flush=True)
                continue

            print(f"→ Smooth move to: {values}")
            start_motion(values)

        except queue.Empty:
            pass
        except ValueError:
            print("ERROR: Invalid numbers!")
            print(">>> ", end="", flush=True)

        # === INTERPOLATE IF MOVING ===
        if is_moving:
            motion_counter += 1
            t = motion_counter / motion_steps
            if t >= 1.0:
                t = 1.0
                is_moving = False
            interp_pose = current_pose * (1 - t) + target_pose * t
            arm.set_joint_positions(interp_pose)
        else:
            # Hold last position
            arm.set_joint_positions(arm.get_joint_positions())

        # === STEP SIMULATION ===
        world.step(render=True)

finally:
    simulation_app.close()