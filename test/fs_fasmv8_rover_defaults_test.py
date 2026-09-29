#!/usr/bin/env python3
"""Regression tests for flying-car rover defaults on PX4 v1.16.1."""

import re
import unittest
from pathlib import Path


DEFAULTS = (
    Path(__file__).resolve().parents[1]
    / "ROMFS/px4fmu_common/init.d/rc.flying_car_sitl_defaults"
)


def default_parameters():
    parameters = {}
    pattern = re.compile(
        r"^\s*param\s+set-default\s+(\S+)\s+"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
    )

    for line in DEFAULTS.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            parameters[match.group(1)] = float(match.group(2))

    return parameters


class FlyingCarRoverDefaultsTest(unittest.TestCase):
    def test_differential_rover_has_valid_yaw_control_defaults(self):
        """Starting rover_differential must not reject zero-valued RO_* controls."""
        parameters = default_parameters()

        self.assertIn("RO_YAW_P", parameters)
        self.assertIn("RO_YAW_RATE_LIM", parameters)
        self.assertIn("RO_YAW_RATE_P", parameters)
        self.assertEqual(parameters["RO_YAW_P"], 5.0)
        self.assertEqual(parameters["RO_YAW_RATE_LIM"], 180.0)
        self.assertEqual(parameters["RO_YAW_RATE_P"], 0.25)


if __name__ == "__main__":
    unittest.main()
