#!/usr/bin/env python3
"""Regression tests for the 80004 flying-car flight physics."""

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
AIRFRAME = (
    ROOT
    / "ROMFS/px4fmu_common/init.d-posix/airframes"
    / "80004_gazebo-classic_fs_fasmv8_flying_car"
)
MODEL = ROOT / (
    "Tools/simulation/gazebo-classic/flying_car/models/"
    "FS-FASMv8_octo/FS-FASMv8_octo.sdf"
)
GRAVITY = 9.8066


def airframe_float_parameter(name):
    pattern = re.compile(
        rf"^\s*param\s+set-default\s+{re.escape(name)}\s+"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
    )

    for line in AIRFRAME.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            return float(match.group(1))

    raise AssertionError(f"{name} is not set by {AIRFRAME}")


class FlyingCarFlightPhysicsTest(unittest.TestCase):
    def test_maximum_rotor_thrust_matches_configured_hover_thrust(self):
        """80004 must have TWR 2.0 and a matching inverse-quadratic curve."""
        model = ET.parse(MODEL).getroot().find("model")
        mass = sum(
            float(link.findtext("inertial/mass"))
            for link in model.findall("link")
            if link.find("inertial") is not None
        )
        motors = [
            plugin for plugin in model.findall("plugin")
            if plugin.attrib.get("filename") == "libgazebo_motor_model.so"
        ]

        self.assertEqual(len(motors), 8)
        motor_constants = [
            float(motor.findtext("motorConstant")) for motor in motors
        ]
        self.assertEqual(motor_constants, [0.017296235] * 8)
        max_thrust = sum(
            motor_constant
            * float(motor.findtext("maxRotVelocity")) ** 2
            for motor_constant, motor in zip(motor_constants, motors)
        )
        max_twr = max_thrust / (mass * GRAVITY)
        hover_thrust = airframe_float_parameter("MPC_THR_HOVER")

        self.assertAlmostEqual(max_twr, 2.0, places=2)
        self.assertAlmostEqual(1.0 / max_twr, hover_thrust, places=2)
        self.assertEqual(airframe_float_parameter("THR_MDL_FAC"), 1.0)



    def test_xy_velocity_loop_gains_are_tuned(self):
        """80004 XY velocity loop must use the validated smooth-control gains."""
        self.assertEqual(airframe_float_parameter("MPC_XY_P"), 0.95)
        self.assertEqual(airframe_float_parameter("MPC_XY_VEL_P_ACC"), 0.50)
        self.assertEqual(airframe_float_parameter("MPC_XY_VEL_I_ACC"), 0.05)
        self.assertEqual(airframe_float_parameter("MPC_XY_VEL_D_ACC"), 0.01)
if __name__ == "__main__":
    unittest.main()
