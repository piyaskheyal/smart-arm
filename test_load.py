# test_load.py
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})  # GUI ON

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

usd_path = "/home/kheyal/dev/robotics/smart-arm-v2/Models/armv1.usd"
add_reference_to_stage(usd_path=usd_path, prim_path="/World/so101")

# Let it load
for _ in range(120):
    simulation_app.update()

print("Robot loaded! Check GUI. Close window to exit.")
simulation_app.close()