#pragma once

#include <cstdint>


enum class FlyingCarTargetStack {
	None,
	Flight,
	Rover
};

inline int32_t flyingCarFlightMavType(int32_t autostart_id, int32_t current_mav_type)
{
	if (autostart_id == 80004 || autostart_id == 80005) {
		return 14;
	}

	if (autostart_id == 80003) {
		return 2;
	}

	return current_mav_type;
}

inline int32_t guardedMavType(bool armed, bool currently_rover,
			     int32_t requested_mav_type, int32_t flight_mav_type)
{
	if (armed) {
		return currently_rover ? 10 : flight_mav_type;
	}

	return requested_mav_type;
}

inline bool flyingCarControlAllocationEnabled(bool flying_car_enabled,
		bool rover_mode, bool requested_enabled)
{
	return requested_enabled && !(flying_car_enabled && rover_mode);
}

inline FlyingCarTargetStack requestedStack(bool armed, bool currently_rover,
		float aux1, int32_t mav_type, int32_t flight_mav_type)
{
	// MC -> ROVER: only when disarmed (flight->ground is a high-risk switch).
	if (!currently_rover && !armed && (aux1 > 0.5f || mav_type == 10)) {
		return FlyingCarTargetStack::Rover;
	}

	// ROVER -> MC: always allowed (emergency return; Commander disarms first).
	if (currently_rover && (aux1 < -0.5f || mav_type == flight_mav_type)) {
		return FlyingCarTargetStack::Flight;
	}

	return FlyingCarTargetStack::None;
}
