#!/usr/bin/env python3
# control_arm_live_input.py
# ------------------------------------------------------------
# Isaac Sim 5.1.0 – Live Terminal Control (Type & Move)
# ------------------------------------------------------------

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
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
# SETUP
# ------------------------------------------------------------------
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

print(f"Loading: {USD_PATH}")
add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT_PRIM_PATH)

print("Creating Articulation...")
arm = Articulation(prim_paths_expr=f"{ROBOT_PRIM_PATH}/.*", name="arm")

print("Starting physics...")
world.reset()
world.step(render=True)

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
# SIGNAL HANDLER (Ctrl+C)
# ------------------------------------------------------------------
def signal_handler(sig, frame):
    print("\n\nShutting down...")
    simulation_app.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# ------------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------------
print("Simulation running. Type poses below.\n")

try:
    while True:
        # Process input
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

            print(f"→ Moving to: {values}")
            arm.set_joint_positions(np.array(values))

        except queue.Empty:
            pass
        except ValueError:
            print("ERROR: Invalid numbers!")
            print(">>> ", end="", flush=True)

        # Step simulation
        world.step(render=True)

finally:
    simulation_app.close()