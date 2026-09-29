#!/usr/bin/env python3

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PX4_ROOT = Path(__file__).resolve().parents[1]
AIRFRAME = (
    PX4_ROOT
    / "ROMFS/px4fmu_common/init.d-posix/airframes"
    / "80004_gazebo-classic_fs_fasmv8_flying_car"
)
MODEL_ROOT = PX4_ROOT / "Tools/simulation/gazebo-classic/flying_car/models"
MODEL = MODEL_ROOT / "FS-FASMv8_octo/FS-FASMv8_octo.sdf"
AIRFRAME_CMAKE = (
    PX4_ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
)
TARGET_CMAKE = (
    PX4_ROOT
    / "src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
)


def read_rotor_parameters():
    parameters = {}
    pattern = re.compile(
        r"^\s*param\s+set-default\s+"
        r"(CA_ROTOR\d+_(?:PX|PY|PZ|KM))\s+"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
    )

    for line in AIRFRAME.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)

        if match:
            parameters[match.group(1)] = float(match.group(2))

    return parameters


class FsFasmv8AirframeMappingTest(unittest.TestCase):

    def test_direct_make_targets_are_registered(self):
        airframe_entries = {
            line.strip() for line in AIRFRAME_CMAKE.read_text(encoding="utf-8").splitlines()
        }
        model_entries = {
            line.strip() for line in TARGET_CMAKE.read_text(encoding="utf-8").splitlines()
        }

        for airframe_id, model_name in (
            (80004, "fs_fasmv8_flying_car"),
            (80005, "fs_fasmv8_flying_car_light"),
        ):
            self.assertIn(
                f"{airframe_id}_gazebo-classic_{model_name}", airframe_entries
            )
            self.assertIn(model_name, model_entries)
            self.assertTrue((MODEL_ROOT / model_name / f"{model_name}.sdf").is_file())

    def test_airframe_rotors_match_sdf_in_px4_frd_coordinates(self):
        parameters = read_rotor_parameters()
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
                parameter = f"CA_ROTOR{motor}_{field}"
                self.assertIn(parameter, parameters)
                self.assertAlmostEqual(
                    parameters[parameter],
                    expected_value,
                    places=6,
                    msg=(
                        f"{parameter} does not map {joint_name} from Gazebo FLU "
                        "coordinates to PX4 FRD coordinates"
                    ),
                )


if __name__ == "__main__":
    unittest.main()
