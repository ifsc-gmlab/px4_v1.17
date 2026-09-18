#pragma once

#include <uORB/topics/actuator_motors.h>

namespace RoverDifferentialActuatorMapping
{

inline bool writeWheelCommands(actuator_motors_s &motors, int offset,
			       float left, float right)
{
	for (float &control : motors.control) {
		control = 0.0f;
	}

	if (offset < 0 || offset + 3 >= actuator_motors_s::NUM_CONTROLS) {
		return false;
	}

	motors.control[offset] = left;
	motors.control[offset + 1] = right;
	motors.control[offset + 2] = left;
	motors.control[offset + 3] = right;
	return true;
}

} // namespace RoverDifferentialActuatorMapping
