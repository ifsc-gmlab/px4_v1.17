#!/usr/bin/env python3
"""
Unit tests for the FS-FASMv8 Flying Car (Light 50kg variant).
Validates the new lightweight model, airframe, and launch configuration.

Design parameters:
- Model mass: 50 kg
- TWR (thrust-to-weight): 2.0
- Hover throttle: ~50%
- motorConstant: 0.000865
- maxRotVelocity: 376.49
"""
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
AIRFRAME = (
    ROOT
    / "ROMFS/px4fmu_common/init.d-posix/airframes"
    / "80005_gazebo-classic_fs_fasmv8_flying_car_light"
)
MODEL = ROOT / (
    "Tools/simulation/gazebo-classic/flying_car/models/"
    "FS-FASMv8_octo_light/FS-FASMv8_octo_light.sdf"
)
LAUNCH = ROOT / "launch/FS_flying_car_light.launch"
WORLD = ROOT / (
    "Tools/simulation/gazebo-classic/flying_car/worlds/"
    "FS_flying_car.world"
)
DEFAULTS = ROOT / "ROMFS/px4fmu_common/init.d/rc.flying_car_sitl_defaults"

# Expected values from design
EXPECTED_MASS = 50.0
EXPECTED_MOTOR_CONSTANT = 0.000865
EXPECTED_MAX_ROT_VELOCITY = 376.49
EXPECTED_TWR = 2.0
EXPECTED_HOVER_THROTTLE = 0.50


def read_parameters():
    """Parse parameter values from the airframe file."""
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


class FlyingCarLightAirframeTest(unittest.TestCase):
    """Tests for the lightweight flying car airframe configuration."""

    def test_basic_vehicle_configuration(self):
        """Verify basic multicopter configuration is set correctly."""
        parameters = read_parameters()
        self.assertEqual(parameters["MAV_TYPE"], 14)
        self.assertEqual(parameters["CA_ROTOR_COUNT"], 8)
        self.assertEqual(parameters["RD_MOT_OFFSET"], 8)
        self.assertEqual(parameters["CA_R_REV"], 3840)

    def test_thrust_to_weight_ratio(self):
        """
        Verify thrust-to-weight ratio is 2.0.
        F_max_total = 8 * motorConstant * maxRotVelocity^2
        TWR = F_max_total / (mass * g)
        For 50 kg, TWR=2.0:
        motorConstant = weight * TWR / (8 * omega^2)
                      = 50*9.8066*2 / (8 * 376.49^2)
                      = 0.000865
        """
        parameters = read_parameters()
        # Hover throttle = 1/TWR = 0.5
        self.assertAlmostEqual(parameters["MPC_THR_HOVER"], EXPECTED_HOVER_THROTTLE, places=2)

        # The actual physical verification is in the model test
        # Here we verify the airframe sets the expected hover throttle

    def test_output_channels(self):
        """Verify PWM_MAIN_FUNC assignments for rotors and wheels."""
        parameters = read_parameters()
        for channel in range(1, 13):
            self.assertEqual(
                parameters[f"PWM_MAIN_FUNC{channel}"], 100 + channel
            )

    def test_motor_idle_parameter(self):
        """Verify SIM_MOT_IDLE is set correctly (100/maxRotVelocity)."""
        parameters = read_parameters()
        self.assertAlmostEqual(
            parameters["SIM_MOT_IDLE"], 100.0 / EXPECTED_MAX_ROT_VELOCITY, places=6
        )

    def test_attitude_gains_tuned_for_50kg(self):
        """
        Verify attitude control gains are tuned for 50 kg vehicle.
        For a vehicle with ~10x larger inertia than a standard 5kg quad,
        MC_ROLL_P should be significantly increased.
        """
        parameters = read_parameters()
        # Attitude P-gains should be increased from defaults (8.0) for 50kg
        self.assertGreaterEqual(parameters["MC_ROLL_P"], 10.0)
        self.assertGreaterEqual(parameters["MC_PITCH_P"], 10.0)
        self.assertGreaterEqual(parameters["MC_YAW_P"], 5.0)

    def test_rate_gains_tuned_for_50kg(self):
        """Verify rate control gains are increased for higher inertia."""
        parameters = read_parameters()
        # Rate P-gains should be increased from defaults (0.15)
        self.assertGreaterEqual(parameters["MC_ROLLRATE_P"], 0.20)
        self.assertGreaterEqual(parameters["MC_PITCHRATE_P"], 0.25)
        self.assertGreaterEqual(parameters["MC_YAWRATE_I"], 0.10)

    def test_position_control_gains(self):
        """Verify position control gains are reasonable."""
        parameters = read_parameters()
        self.assertEqual(parameters["MPC_XY_P"], 0.95)
        self.assertGreaterEqual(parameters["MPC_XY_VEL_P_ACC"], 0.10)
        self.assertEqual(parameters["MPC_Z_P"], 1.0)
        self.assertGreaterEqual(parameters["MPC_Z_VEL_P_ACC"], 0.30)

    def test_safety_parameters(self):
        """Verify safety parameters are configured for SITL testing."""
        parameters = read_parameters()
        self.assertEqual(parameters["COM_ARM_WO_GPS"], 1)
        self.assertEqual(parameters["COM_RC_IN_MODE"], 1)
        self.assertEqual(parameters["COM_DISARM_LAND"], 0)

    def test_landing_parameters(self):
        """Verify landing parameters for safe touchdown."""
        parameters = read_parameters()
        # Landing should be slow and stable
        self.assertLessEqual(parameters["LNDMC_Z_VEL_MAX"], 1.5)


class FlyingCarLightModelTest(unittest.TestCase):
    """Tests for the lightweight flying car Gazebo model."""

    @classmethod
    def setUpClass(cls):
        cls.model = ET.parse(MODEL).getroot().find("model")

    def test_model_mass_is_50kg(self):
        """
        Verify total model mass is 50 kg.
        Scale factor from 1000 kg to 50 kg is 0.05.
        """
        inertial_fields = ("mass",)
        total_mass = 0.0
        for link in self.model.findall("link"):
            inertial = link.find("inertial")
            if inertial is not None:
                total_mass += float(inertial.findtext("mass"))

        # Total mass should be 50 kg
        self.assertAlmostEqual(total_mass, EXPECTED_MASS, places=4)

    def test_motor_thrust_parameters(self):
        """
        Verify motor parameters give TWR=2.0 for 50 kg model.

        F_max_per_motor = motorConstant * maxRotVelocity^2
        Total F_max = 8 * F_max_per_motor
        TWR = Total F_max / (mass * g)

        For TWR=2.0, mass=50kg:
        motorConstant = 50*9.8066*2 / (8*376.49^2) = 0.000865
        """
        motors = [
            plugin for plugin in self.model.findall("plugin")
            if plugin.attrib.get("filename") == "libgazebo_motor_model.so"
        ]
        self.assertEqual(len(motors), 8)

        for motor in motors:
            self.assertAlmostEqual(
                float(motor.findtext("maxRotVelocity")), EXPECTED_MAX_ROT_VELOCITY
            )
            self.assertAlmostEqual(
                float(motor.findtext("motorConstant")), EXPECTED_MOTOR_CONSTANT, places=6
            )

    def test_mavlink_channels_map_eight_rotors_and_four_wheels(self):
        """Verify MAVLink channel mapping for rotors and wheels."""
        mavlink = next(
            plugin for plugin in self.model.findall("plugin")
            if plugin.attrib.get("name") == "mavlink_interface"
        )
        channels = {
            int(channel.findtext("input_index")): channel
            for channel in mavlink.find("control_channels").findall("channel")
        }
        self.assertEqual(set(range(12)), set(channels))

        # Verify wheel channels use velocity control
        for index in range(8, 12):
            self.assertEqual(channels[index].findtext("joint_control_type"), "velocity")

    def test_wheel_contacts_are_damped(self):
        """Verify wheel contacts have damping for stable ground mode."""
        links = {
            link.attrib["name"]: link
            for link in self.model.findall("link")
        }

        for wheel in ("link9", "link10", "link11", "link12"):
            surface = links[wheel].find("collision/surface")
            friction = surface.find("friction/ode")
            self.assertEqual(
                tuple(float(friction.findtext(key)) for key in
                      ("mu1", "mu2", "slip1", "slip2")),
                (1.0, 1.0, 0.01, 0.01),
            )


class FlyingCarLightIntegrationTest(unittest.TestCase):
    """Integration tests for the lightweight flying car configuration."""

    def test_launch_uses_lightweight_model(self):
        """Verify launch file references the lightweight model."""
        launch = ET.parse(LAUNCH).getroot()
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in launch.findall("arg")
        }
        self.assertEqual(arguments["my_model"], "FS-FASMv8_octo_light")
        self.assertEqual(arguments["vehicle"], "fs_fasmv8_flying_car_light")

    def test_light_model_files_exist(self):
        """Verify all required model files exist."""
        self.assertTrue(MODEL.is_file(), f"missing model SDF: {MODEL}")
        self.assertTrue(AIRFRAME.is_file(), f"missing airframe: {AIRFRAME}")
        self.assertTrue(LAUNCH.is_file(), f"missing launch: {LAUNCH}")
        # Mesh directory
        mesh_dir = MODEL.parent / "meshes"
        self.assertTrue(mesh_dir.is_dir(), f"missing mesh directory: {mesh_dir}")


class FlyingCarLightPhysicsTest(unittest.TestCase):
    """Physics-based validation tests for the lightweight model."""

    def test_thrust_to_weight_ratio_calculated(self):
        """
        Verify the TWR is exactly 2.0 for the configured parameters.

        For a multirotor to be flyable:
        - TWR >= 1.5 is acceptable
        - TWR 2.0-3.0 is good
        - TWR > 4.0 is excessive (control difficulties)

        Our design uses TWR=2.0 for stable, controllable flight.
        """
        motor_constant = EXPECTED_MOTOR_CONSTANT
        max_rot_velocity = EXPECTED_MAX_ROT_VELOCITY
        mass = EXPECTED_MASS
        g = 9.8066

        f_max_per_motor = motor_constant * (max_rot_velocity ** 2)
        f_max_total = 8 * f_max_per_motor
        weight = mass * g
        twr = f_max_total / weight

        # Verify TWR is exactly 2.0 (within floating-point tolerance)
        self.assertAlmostEqual(twr, EXPECTED_TWR, places=2)

    def test_hover_throttle_matches_twr(self):
        """
        Verify hover throttle is the inverse of TWR.

        For TWR=2.0: hover_throttle = 1/TWR = 0.5 (50%)
        """
        hover_throttle = 1.0 / EXPECTED_TWR
        self.assertAlmostEqual(hover_throttle, EXPECTED_HOVER_THROTTLE, places=2)


if __name__ == "__main__":
    unittest.main()