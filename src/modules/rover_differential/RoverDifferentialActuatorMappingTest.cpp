#include <gtest/gtest.h>

#include <cmath>

#include "RoverDifferentialActuatorMapping.hpp"

TEST(RoverDifferentialActuatorMapping, WritesFourWheelsAtOffsetEight)
{
	actuator_motors_s motors{};
	ASSERT_TRUE(RoverDifferentialActuatorMapping::writeWheelCommands(
			    motors, 8, -0.25f, 0.75f));

	for (int i = 0; i < 8; ++i) {
		// NaN leaves non-wheel channels untouched by the rover publisher.
		EXPECT_TRUE(std::isnan(motors.control[i]));
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

TEST(RoverDifferentialActuatorMapping, PublishesGroundCommandsAndDisarmedHeartbeat)
{
	EXPECT_TRUE(RoverDifferentialActuatorMapping::shouldPublishActuatorFrame(true, 10));
	EXPECT_TRUE(RoverDifferentialActuatorMapping::shouldPublishActuatorFrame(false, 10));
	EXPECT_TRUE(RoverDifferentialActuatorMapping::shouldPublishActuatorFrame(false, 14));
	EXPECT_FALSE(RoverDifferentialActuatorMapping::shouldPublishActuatorFrame(true, 14));
}
