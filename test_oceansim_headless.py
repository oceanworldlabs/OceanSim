#!/usr/bin/env python3
"""Headless test of OceanSim sensors on Isaac Sim 6.

Tests: UW_Camera, BarometerSensor, DVLsensor, ImagingSonarSensor
"""

import sys

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import numpy as np
import omni.kit.app

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.set_extension_enabled_immediate("OceanSim", True)

# Import sensors
from isaacsim.oceansim.sensors.UW_Camera import UW_Camera
from isaacsim.oceansim.sensors.BarometerSensor import BarometerSensor
from isaacsim.oceansim.sensors.DVLsensor import DVLsensor
from isaacsim.oceansim.sensors.ImagingSonarSensor import ImagingSonarSensor
from isaacsim.oceansim.utils.assets_utils import get_oceansim_assets_path

from isaacsim.core.utils.stage import create_new_stage, add_reference_to_stage
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.prims import SingleRigidPrim, SingleGeometryPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from pxr import PhysxSchema

print(f"OceanSim assets: {get_oceansim_assets_path()}")

# Create stage
create_new_stage()

# Load robot
robot_prim_path = "/World/rob"
robot_usd_path = get_oceansim_assets_path() + "/Bluerov/BROV_low.usd"
rob = add_reference_to_stage(usd_path=robot_usd_path, prim_path=robot_prim_path)

# Setup robot physics
rob_rigidBody_API = PhysxSchema.PhysxRigidBodyAPI.Apply(get_prim_at_path(robot_prim_path))
rob_rigidBody_API.CreateDisableGravityAttr(True)
rob_rigidBody_API.GetLinearDampingAttr().Set(10.0)
rob_rigidBody_API.GetAngularDampingAttr().Set(10.0)

SingleGeometryPrim(prim_path=robot_prim_path, collision=True)
SingleRigidPrim(prim_path=robot_prim_path, mass=5.0, translation=np.array([-2.0, 0.0, -0.8]))

print("Robot loaded successfully")

# Test BarometerSensor
failed_checks = 0
try:
    baro = BarometerSensor(prim_path=robot_prim_path + '/Baro', water_surface_z=1.43389)
    pressure = baro.get_pressure()
    print(f"OK: BarometerSensor - pressure = {pressure:.1f} Pa")
except Exception as e:
    failed_checks += 1
    print(f"FAIL: BarometerSensor - {e}")

# Test DVLsensor
try:
    dvl = DVLsensor(max_range=10)
    dvl.attachDVL(rigid_body_path=robot_prim_path, translation=np.array([0, 0, -0.1]))
    dvl.add_debug_lines()
    print("OK: DVLsensor - attached and debug lines added")
except Exception as e:
    failed_checks += 1
    print(f"FAIL: DVLsensor - {e}")

# Test UW_Camera (just creation, rendering needs timeline)
try:
    cam = UW_Camera(prim_path=robot_prim_path + '/UW_camera',
                    resolution=[640, 480],
                    translation=np.array([0.3, 0.0, 0.1]))
    cam.set_focal_length(2.1)
    cam.set_clipping_range(0.1, 100)
    print("OK: UW_Camera - created")
except Exception as e:
    failed_checks += 1
    print(f"FAIL: UW_Camera - {e}")

# Test ImagingSonarSensor (just creation)
try:
    sonar = ImagingSonarSensor(prim_path=robot_prim_path + '/sonar',
                               translation=np.array([0.3, 0.0, 0.3]),
                               orientation=euler_angles_to_quat(np.array([0.0, 45, 0.0]), degrees=True),
                               range_res=0.005,
                               angular_res=0.25,
                               hori_res=4000)
    print("OK: ImagingSonarSensor - created")
except Exception as e:
    failed_checks += 1
    print(f"FAIL: ImagingSonarSensor - {e}")

print(f"\nOceanSim sensor checks completed: {4 - failed_checks} passed, {failed_checks} failed")
app.close()
sys.exit(1 if failed_checks else 0)
