#!/usr/bin/env python3
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
DEFAULTS = ROOT / "ROMFS/px4fmu_common/init.d/rc.flying_car_sitl_defaults"
APPS = ROOT / "ROMFS/px4fmu_common/init.d/rc.flying_car_apps"


def read_parameters():
    parameters = {}
    pattern = re.compile(
        r"^\s*param\s+set-default\s+(\S+)\s+"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
    )

    for line in AIRFRAME.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)

        if match:
            parameters[match.group(1)] = float(match.group(2))

    return parameters


class FlyingCarAirframeTest(unittest.TestCase):
    def test_output_layout_and_idle_parameters(self):
        parameters = read_parameters()
        self.assertEqual(parameters["MAV_TYPE"], 14)
        self.assertEqual(parameters["CA_ROTOR_COUNT"], 8)
        self.assertEqual(parameters["RD_MOT_OFFSET"], 8)
        self.assertEqual(parameters["CA_R_REV"], 3840)

        for channel in range(1, 13):
            self.assertEqual(
                parameters[f"PWM_MAIN_FUNC{channel}"], 100 + channel
            )

        for channel in range(1, 9):
            self.assertNotIn(f"PWM_MAIN_MIN{channel}", parameters)

        # The flying-car SITL airframe idles rotors at zero so a zero rotor
        # command means fully stopped rotors during rover ground phases.
        self.assertEqual(parameters["SIM_MOT_IDLE"], 0.0)

    def test_rotors_match_sdf_in_px4_frd_coordinates(self):
        parameters = read_parameters()
        root = ET.parse(MODEL).getroot()
        joints = {
            joint.attrib["name"]: [
                float(value) for value in joint.findtext("pose").split()[:3]
            ]
            for joint in root.findall(".//joint")
            if joint.find("pose") is not None
        }
        motor_plugins = sorted(
            (
                int(plugin.findtext("motorNumber")),
                plugin.findtext("jointName"),
                plugin.findtext("turningDirection"),
            )
            for plugin in root.findall(".//plugin")
            if plugin.attrib.get("filename") == "libgazebo_motor_model.so"
        )
        self.assertEqual([motor for motor, _, _ in motor_plugins], list(range(8)))

        for motor, joint_name, direction in motor_plugins:
            gazebo_x, gazebo_y, gazebo_z = joints[joint_name]
            expected = {
                "PX": gazebo_x,
                "PY": -gazebo_y,
                "PZ": -gazebo_z,
                "KM": 0.05 if direction == "ccw" else -0.05,
            }

            for field, expected_value in expected.items():
                self.assertAlmostEqual(
                    parameters[f"CA_ROTOR{motor}_{field}"],
                    expected_value,
                    places=6,
                )

    def test_startup_uses_current_parameter_names(self):
        startup = AIRFRAME.read_text(encoding="utf-8")
        startup += DEFAULTS.read_text(encoding="utf-8")
        obsolete = {
            "COM_ARM_EKF_POS",
            "COM_ARM_EKF_VEL",
            "MPC_XY_VEL_P",
            "MPC_XY_VEL_I",
            "MPC_XY_VEL_D",
            "MPC_Z_VEL_P",
            "MPC_Z_VEL_I",
            "MPC_Z_VEL_D",
        }

        for parameter in obsolete:
            self.assertNotRegex(
                startup, rf"(?m)^\s*param\s+set-default\s+{parameter}\s"
            )

        parameters = read_parameters()
        self.assertEqual(parameters["MPC_XY_VEL_P_ACC"], 0.50)
        self.assertEqual(parameters["MPC_XY_VEL_I_ACC"], 0.05)
        self.assertEqual(parameters["MPC_XY_VEL_D_ACC"], 0.01)
        self.assertEqual(parameters["MPC_Z_VEL_P_ACC"], 0.50)
        self.assertEqual(parameters["MPC_Z_VEL_I_ACC"], 0.20)
        # SITL vertical damping: the EKF altitude estimate lags the truth
        # during climb, so an undamped z-velocity loop overshoots the takeoff
        # target by ~2 m and trips the AGL safety limit.
        self.assertEqual(parameters["MPC_Z_VEL_D_ACC"], 0.25)

    def test_both_control_stacks_start_and_allocator_stays_multirotor(self):
        airframe = AIRFRAME.read_text(encoding="utf-8")
        apps = APPS.read_text(encoding="utf-8")
        self.assertRegex(airframe, r"(?m)^param set CA_AIRFRAME 0$")
        self.assertRegex(apps, r"(?m)^control_allocator start$")
        self.assertRegex(apps, r"(?m)^rover_differential start$")


if __name__ == "__main__":
    unittest.main()
