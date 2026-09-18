#include <gtest/gtest.h>

#include "RoverDifferentialActuatorMapping.hpp"

TEST(RoverDifferentialActuatorMapping, WritesFourWheelsAtOffsetEight)
{
	actuator_motors_s motors{};
	ASSERT_TRUE(RoverDifferentialActuatorMapping::writeWheelCommands(
			    motors, 8, -0.25f, 0.75f));

	for (int i = 0; i < 8; ++i) {
		// SITL port: non-wheel channels are zero-filled so the simulation
		// output pipeline never sees NaN rotor/wheel commands.
		EXPECT_EQ(motors.control[i], 0.0f);
	}

	EXPECT_FLOAT_EQ(motors.control[8], -0.25f);
	EXPECT_FLOAT_EQ(motors.control[9], 0.75f);
	EXPECT_FLOAT_EQ(motors.control[10], -0.25f);
	EXPECT_FLOAT_EQ(motors.control[11], 0.75f);
}

TEST(RoverDifferentialActuatorMapping, RejectsOffsetOutsideTwelveMotorArray)
{
	actuator_motors_s motors{};
	EXPECT_FALSE(RoverDifferentialActuatorMapping::writeWheelCommands(
			     motors, 9, 0.1f, 0.2f));
}
