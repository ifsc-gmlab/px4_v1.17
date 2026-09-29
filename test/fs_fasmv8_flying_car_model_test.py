#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / (
    "Tools/simulation/gazebo-classic/flying_car/models/"
    "FS-FASMv8_octo/FS-FASMv8_octo.sdf"
)
LAUNCH = ROOT / "launch/FS_flying_car.launch"
WORLD = ROOT / (
    "Tools/simulation/gazebo-classic/flying_car/worlds/"
    "FS_flying_car.world"
)
ORIGINAL_INERTIALS = {
    "base_link": (2014.6, 199.86, 0.0387763, -0.0162954,
                  523.22, -0.0464263, 711.879),
    "/link_imu": (0.055, 5e-05, 0.0, 0.0, 5e-05, 0.0, 5e-05),
    "link1": (12.4796, 0.782573, -0.728518, 9.05168e-08,
              0.692515, 5.28706e-08, 1.474),
    "link2": (12.4795, 0.782576, 0.728526, -7.31367e-08,
              0.692527, 3.45378e-08, 1.47401),
    "link3": (12.4796, 1.46607, 0.0450295, -2.53057e-08,
              0.0090258, -1.08433e-07, 1.47401),
    "link4": (12.4795, 1.46609, -0.0450249, -2.27117e-08,
              0.00902529, 1.18391e-07, 1.47402),
    "link5": (12.48, 0.78257, -0.72852, 9.313e-08,
              0.69252, 5.5556e-08, 1.474),
    "link6": (12.4795, 0.782576, 0.728526, 8.243e-08,
              0.692527, -4.44141e-08, 1.47401),
    "link7": (12.4796, 0.00902577, -0.0450294, 1.28014e-07,
              1.46607, -2.26937e-08, 1.47401),
    "link8": (12.4795, 0.00902529, 0.0450249, 1.06812e-07,
              1.46609, 2.31e-08, 1.47402),
    "link9": (82.971, 1.5735, 0.0, 0.0, 1.5735, 0.0, 2.5937),
    "link10": (82.9705, 1.5735, 0.0, 0.0, 1.5735, 0.0, 2.5937),
    "link11": (82.9705, 1.5735, 0.0, 0.0, 1.5735, 0.0, 2.5937),
    "link12": (82.9705, 1.5735, 0.0, 0.0, 1.5735, 0.0, 2.5937),
}
ORIGINAL_TOTAL_MASS = 2446.3743
INERTIAL_SCALE = 1000.0 / ORIGINAL_TOTAL_MASS


class FlyingCarModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ET.parse(MODEL).getroot().find("model")

    def test_ackermann_plugin_is_removed_and_front_wheels_are_direct(self):
        plugins = {
            plugin.attrib["filename"]
            for plugin in self.model.findall("plugin")
        }
        self.assertNotIn("libgazebo_ros_ackermann_drive_plugin.so", plugins)
        joints = {
            joint.attrib["name"]: joint
            for joint in self.model.findall("joint")
        }
        links = {link.attrib["name"] for link in self.model.findall("link")}
        self.assertNotIn("front_left_steer_joint", joints)
        self.assertNotIn("front_right_steer_joint", joints)
        self.assertNotIn("front_left_steer_link", links)
        self.assertNotIn("front_right_steer_link", links)
        self.assertEqual(joints["joint10"].findtext("parent"), "base_link")
        self.assertEqual(joints["joint9"].findtext("parent"), "base_link")

    def test_all_wheel_contacts_are_damped_and_allow_small_slip(self):
        links = {
            link.attrib["name"]: link
            for link in self.model.findall("link")
        }

        for wheel in ("link9", "link10", "link11", "link12"):
            surface = links[wheel].find("collision/surface")
            friction = surface.find("friction/ode")
            contact = surface.find("contact/ode")
            self.assertEqual(
                tuple(float(friction.findtext(key)) for key in
                      ("mu1", "mu2", "slip1", "slip2")),
                (1.0, 1.0, 0.01, 0.01),
            )
            self.assertEqual(
                tuple(float(contact.findtext(key)) for key in ("kp", "kd")),
                (1000000.0, 20000.0),
            )

    def test_launch_uses_stable_single_ground_world_with_spawn_clearance(self):
        launch = ET.parse(LAUNCH).getroot()
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in launch.findall("arg")
        }
        self.assertEqual(
            arguments["world"],
            "$(find mavlink_sitl_gazebo)/worlds/FS_flying_car.world",
        )
        self.assertTrue(WORLD.is_file(), f"missing dedicated world: {WORLD}")

        world = ET.parse(WORLD).getroot().find("world")
        ground_uris = [
            include.findtext("uri")
            for include in world.findall("include")
            if include.findtext("uri") in
            {"model://ground_plane", "model://asphalt_plane"}
        ]
        self.assertEqual(ground_uris, ["model://ground_plane"])

        mavlink = next(
            plugin for plugin in self.model.findall("plugin")
            if plugin.attrib.get("name") == "mavlink_interface"
        )
        self.assertEqual(mavlink.findtext("enable_lockstep"), "false")

        physics = world.find("physics")
        self.assertEqual(
            int(physics.findtext("real_time_update_rate")), 250
        )
        self.assertEqual(float(physics.findtext("max_step_size")), 0.004)
        self.assertEqual(int(physics.findtext("ode/solver/iters")), 20)
        self.assertEqual(
            int(physics.findtext("ode/solver/use_dynamic_moi_rescaling")), 0
        )
        self.assertLessEqual(
            float(physics.findtext(
                "ode/constraints/contact_max_correcting_vel"
            )),
            2.0,
        )

        joints = {
            joint.attrib["name"]: joint
            for joint in self.model.findall("joint")
        }
        wheel_joint_z = float(
            joints["joint10"].findtext("pose").split()[2]
        )
        wheel_radius = float(
            self.model.find(
                "link[@name='link10']/collision/geometry/cylinder/radius"
            ).text
        )
        clearance = float(arguments["z"]) + wheel_joint_z - wheel_radius
        self.assertGreater(clearance, 0.0)
        self.assertLess(clearance, 0.05)

    def test_mavlink_channels_map_eight_rotors_and_four_wheels(self):
        mavlink = next(
            plugin for plugin in self.model.findall("plugin")
            if plugin.attrib.get("name") == "mavlink_interface"
        )
        channels = {
            int(channel.findtext("input_index")): channel
            for channel in mavlink.find("control_channels").findall("channel")
        }
        self.assertEqual(set(range(12)), set(channels))
        self.assertEqual(
            {i: channels[i].findtext("joint_name") for i in range(8, 12)},
            {8: "joint10", 9: "joint9", 10: "joint11", 11: "joint12"},
        )

        for index in range(8):
            self.assertAlmostEqual(
                float(channels[index].findtext("input_scaling")), 376.49
            )
            self.assertEqual(
                float(channels[index].findtext("zero_position_armed")), 0.0
            )

        for index in range(8, 12):
            self.assertEqual(channels[index].findtext("joint_control_type"), "velocity")
            self.assertAlmostEqual(
                float(channels[index].findtext("input_scaling")), 8.0
            )
            pid = channels[index].find("joint_control_pid")
            self.assertEqual(
                tuple(float(pid.findtext(k)) for k in ("p", "i", "d")),
                (80.0, 10.0, 1.0),
            )

    def test_all_mass_and_inertia_values_are_uniformly_scaled_to_1000kg(self):
        inertial_fields = ("mass", "ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        links = {
            link.attrib["name"]: link
            for link in self.model.findall("link")
            if link.find("inertial") is not None
        }
        self.assertEqual(set(links), set(ORIGINAL_INERTIALS))

        total_mass = 0.0

        for link_name, original_values in ORIGINAL_INERTIALS.items():
            inertial = links[link_name].find("inertial")
            actual_values = (
                float(inertial.findtext("mass")),
                *(float(inertial.findtext(f"inertia/{field}"))
                  for field in inertial_fields[1:]),
            )

            for field, actual, original in zip(
                    inertial_fields, actual_values, original_values):
                self.assertAlmostEqual(
                    actual,
                    original * INERTIAL_SCALE,
                    places=9,
                    msg=f"{link_name}.{field} was not uniformly scaled",
                )

            total_mass += actual_values[0]

        self.assertAlmostEqual(total_mass, 1000.0, places=8)

    def test_motor_thrust_parameters_match_hover_calibration(self):
        motors = [
            plugin for plugin in self.model.findall("plugin")
            if plugin.attrib.get("filename") == "libgazebo_motor_model.so"
        ]
        self.assertEqual(len(motors), 8)

        motor_constants = [
            float(motor.findtext("motorConstant")) for motor in motors
        ]
        self.assertEqual(motor_constants, [0.017296235] * 8)

        for motor, motor_constant in zip(motors, motor_constants):
            self.assertAlmostEqual(
                float(motor.findtext("maxRotVelocity")), 376.49
            )
            self.assertAlmostEqual(motor_constant, 0.017296235, places=9)


if __name__ == "__main__":
    unittest.main()
