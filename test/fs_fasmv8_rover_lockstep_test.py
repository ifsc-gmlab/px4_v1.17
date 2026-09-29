#!/usr/bin/env python3
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "src/modules/rover_differential/RoverDifferential.hpp"
SOURCE = ROOT / "src/modules/rover_differential/RoverDifferential.cpp"
MAPPING = ROOT / (
    "src/modules/rover_differential/"
    "RoverDifferentialActuatorMapping.hpp"
)
ACT_SOURCE = ROOT / (
    "src/modules/rover_differential/DifferentialActControl/"
    "DifferentialActControl.cpp"
)
COMMANDER = ROOT / "src/modules/commander/Commander.cpp"


class FlyingCarRoverLockstepTest(unittest.TestCase):
    def test_rover_output_is_driven_by_each_angular_velocity_update(self):
        header = HEADER.read_text()
        source = SOURCE.read_text()

        self.assertIn(
            "SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub",
            header,
        )
        self.assertIn(
            "_vehicle_angular_velocity_sub.registerCallback()",
            source,
        )
        # v1.17.0 port: callback drive is POSIX-only; NuttX keeps the
        # interval schedule so real-hardware behavior is unchanged.
        self.assertIn("#if defined(__PX4_POSIX)", source)
        self.assertIn("ScheduleOnInterval(10_ms)", source)

    def test_ground_mode_disables_the_flight_control_allocator(self):
        commander = COMMANDER.read_text()

        self.assertIn(
            "flyingCarControlAllocationEnabled("
            "_flying_car_enabled,"
            "_flying_car_rover_mode,"
            "_vehicle_control_mode.flag_control_allocation_enabled)",
            "".join(commander.split()),
        )

    def test_disarmed_mode_switch_keeps_a_neutral_actuator_heartbeat(self):
        mapping = "".join(MAPPING.read_text().split())
        source = "".join(SOURCE.read_text().split())
        act = "".join(ACT_SOURCE.read_text().split())

        self.assertIn(
            "shouldPublishActuatorFrame(boolarmed,int32_tmav_type)",
            mapping,
        )
        self.assertIn("return!armed||mav_type==10;", mapping)
        # v1.17.0 port: heartbeat is published from DifferentialActControl
        # while disarmed in rover mode; wheel commands go through the shared
        # mapping helper that also zeroes the frame.
        self.assertIn("_vehicle_status.system_type==10", source)
        self.assertIn("publishHeartbeat()", source)
        self.assertIn("writeWheelCommands(actuator_motors,", act)


if __name__ == "__main__":
    unittest.main()
