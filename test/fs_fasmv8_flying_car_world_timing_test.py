#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / (
    "Tools/simulation/gazebo-classic/flying_car/models/"
    "FS-FASMv8_octo/FS-FASMv8_octo.sdf"
)
WORLD = ROOT / (
    "Tools/simulation/gazebo-classic/flying_car/worlds/"
    "FS_flying_car.world"
)


class FlyingCarWorldTimingTest(unittest.TestCase):
    def test_lockstep_uses_bounded_px4_physics_budget(self):
        model = ET.parse(MODEL).getroot().find("model")
        mavlink = next(
            plugin for plugin in model.findall("plugin")
            if plugin.attrib.get("name") == "mavlink_interface"
        )
        self.assertEqual(mavlink.findtext("enable_lockstep"), "false")

        physics = ET.parse(WORLD).getroot().find("world/physics")
        self.assertEqual(
            int(physics.findtext("real_time_update_rate")), 250
        )
        self.assertEqual(float(physics.findtext("max_step_size")), 0.004)
        self.assertEqual(int(physics.findtext("ode/solver/iters")), 20)
        self.assertEqual(
            int(physics.findtext("ode/solver/use_dynamic_moi_rescaling")), 0
        )


if __name__ == "__main__":
    unittest.main()
