#!/usr/bin/env python3
"""Phase 3: Verify OceanSim sensors on Isaac Sim 6."""
import sys, os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni.kit.app
mgr = omni.kit.app.get_app().get_extension_manager()
mgr.set_extension_enabled_immediate("OceanSim", True)

import numpy as np
from isaacsim.oceansim.utils.assets_utils import get_oceansim_assets_path
from isaacsim.core.utils.stage import create_new_stage, add_reference_to_stage
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.prims import SingleRigidPrim, SingleGeometryPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from pxr import PhysxSchema

results = []

# Create stage and load robot
create_new_stage()
assets_path = get_oceansim_assets_path()
robot_prim_path = "/World/rob"
robot_usd_path = assets_path + "/Bluerov/BROV_low.usd"
rob = add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)
rob_rigidBody_API = PhysxSchema.PhysxRigidBodyAPI.Apply(get_prim_at_path(robot_prim_path))
rob_rigidBody_API.CreateDisableGravityAttr(True)
rob_rigidBody_API.GetLinearDampingAttr().Set(10.0)
rob_rigidBody_API.GetAngularDampingAttr().Set(10.0)
SingleGeometryPrim(prim_path=robot_prim_path, collision=True)
SingleRigidPrim(prim_path=robot_prim_path, mass=5.0, translation=np.array([-2.0, 0.0, -0.8]))
results.append(f"[PASS] BlueROV robot loaded from {robot_usd_path}")

# BarometerSensor
try:
    from isaacsim.oceansim.sensors.BarometerSensor import BarometerSensor
    baro = BarometerSensor(prim_path=robot_prim_path + '/Baro', water_surface_z=1.43389)
    pressure = baro.get_pressure()
    results.append(f"[PASS] BarometerSensor: pressure={pressure:.1f} Pa")
except Exception as e:
    results.append(f"[FAIL] BarometerSensor: {e}")

# DVLsensor
try:
    from isaacsim.oceansim.sensors.DVLsensor import DVLsensor
    dvl = DVLsensor(max_range=10)
    dvl.attachDVL(rigid_body_path=robot_prim_path, translation=np.array([0, 0, -0.1]))
    results.append(f"[PASS] DVLsensor: {len(dvl.get_beam_paths())} beams attached")
except Exception as e:
    results.append(f"[FAIL] DVLsensor: {e}")

# UW_Camera
try:
    from isaacsim.oceansim.sensors.UW_Camera import UW_Camera
    cam = UW_Camera(prim_path=robot_prim_path + '/UW_camera',
                    resolution=[640, 480],
                    translation=np.array([0.3, 0.0, 0.1]))
    cam.set_focal_length(2.1)
    cam.set_clipping_range(0.1, 100)
    results.append(f"[PASS] UW_Camera: resolution={cam.get_resolution()}")
except Exception as e:
    results.append(f"[FAIL] UW_Camera: {e}")

# ImagingSonarSensor
try:
    from isaacsim.oceansim.sensors.ImagingSonarSensor import ImagingSonarSensor
    sonar = ImagingSonarSensor(
        prim_path=robot_prim_path + '/sonar',
        translation=np.array([0.3, 0.0, 0.3]),
        orientation=euler_angles_to_quat(np.array([0.0, 45, 0.0]), degrees=True),
        range_res=0.005, angular_res=0.25, hori_res=4000
    )
    results.append(f"[PASS] ImagingSonarSensor: range={sonar.get_range()}, fov={sonar.get_fov()}")
except Exception as e:
    results.append(f"[FAIL] ImagingSonarSensor: {e}")

# Scenario loop
try:
    from isaacsim.oceansim.modules.SensorExample_python.scenario import MHL_Sensor_Example_Scenario
    scenario = MHL_Sensor_Example_Scenario()
    scenario.setup_scenario(rob=rob, sonar=None, cam=None, DVL=None, baro=baro, ctrl_mode="No control")
    for i in range(5):
        scenario.update_scenario(1.0 / 60.0)
    baro_reading = scenario._baro_reading
    scenario.teardown_scenario()
    results.append(f"[PASS] Sensor Example scenario: baro={baro_reading:.1f} Pa")
except Exception as e:
    results.append(f"[FAIL] Sensor Example scenario: {e}")

# Waypoint following
try:
    scenario2 = MHL_Sensor_Example_Scenario()
    scenario2.setup_scenario(rob=rob, sonar=None, cam=None, DVL=None, baro=None, ctrl_mode="Waypoints")
    scenario2.setup_waypoints(waypoint_path="demo/demo_waypoints.txt", default_waypoint_path="demo/demo_waypoints.txt")
    initial_pos = rob.GetAttribute('xformOp:translate').Get()
    for i in range(10):
        scenario2.update_scenario(1.0 / 60.0)
    final_pos = rob.GetAttribute('xformOp:translate').Get()
    scenario2.teardown_scenario()
    moved = abs(final_pos[0] - initial_pos[0]) > 0.01 or abs(final_pos[1] - initial_pos[1]) > 0.01 or abs(final_pos[2] - initial_pos[2]) > 0.01
    tag = "PASS" if moved else "FAIL"
    results.append(f"[{tag}] Waypoint following: moved={moved}, pos={final_pos}")
except Exception as e:
    results.append(f"[FAIL] Waypoint following: {e}")

with open('/tmp/phase3_results.txt', 'w') as f:
    for r in results:
        f.write(r + '\n')

app.close()

failed_checks = sum(not result.startswith("[PASS]") for result in results)
sys.exit(1 if failed_checks else 0)
