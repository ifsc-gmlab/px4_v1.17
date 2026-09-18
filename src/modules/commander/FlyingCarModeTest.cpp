#include <gtest/gtest.h>

#include "FlyingCarMode.hpp"


TEST(FlyingCarMode, RequestsRoverFromFlight)
{
	EXPECT_EQ(requestedStack(false, false, 1.f, 14, 14),
		  FlyingCarTargetStack::Rover);
	EXPECT_EQ(requestedStack(false, false, 0.f, 10, 14),
		  FlyingCarTargetStack::Rover);
}

TEST(FlyingCarMode, RequestsFlightFromRover)
{
	EXPECT_EQ(requestedStack(false, true, -1.f, 10, 14),
		  FlyingCarTargetStack::Flight);
	EXPECT_EQ(requestedStack(false, true, 0.f, 14, 14),
		  FlyingCarTargetStack::Flight);
}

TEST(FlyingCarMode, NeverSwitchesWhileArmed)
{
	EXPECT_EQ(requestedStack(true, false, 1.f, 14, 14),
		  FlyingCarTargetStack::None);
}

TEST(FlyingCarMode, RejectsMavTypeChangesWhileArmed)
{
	EXPECT_EQ(guardedMavType(true, false, 10, 14), 14);
	EXPECT_EQ(guardedMavType(true, true, 14, 14), 10);
	EXPECT_EQ(guardedMavType(false, false, 10, 14), 10);
}

TEST(FlyingCarMode, RestoresFlightTypeAfterBootingWithSavedRoverType)
{
	EXPECT_EQ(flyingCarFlightMavType(80004, 10), 14);
	EXPECT_EQ(flyingCarFlightMavType(80005, 10), 14);
	EXPECT_EQ(flyingCarFlightMavType(80003, 10), 2);
	EXPECT_EQ(flyingCarFlightMavType(0, 1), 1);
}

TEST(FlyingCarMode, GivesActuatorOwnershipToOnlyOneControlStack)
{
	EXPECT_FALSE(flyingCarControlAllocationEnabled(true, true, true));
	EXPECT_TRUE(flyingCarControlAllocationEnabled(true, false, true));
	EXPECT_TRUE(flyingCarControlAllocationEnabled(false, true, true));
	EXPECT_FALSE(flyingCarControlAllocationEnabled(true, false, false));
}
