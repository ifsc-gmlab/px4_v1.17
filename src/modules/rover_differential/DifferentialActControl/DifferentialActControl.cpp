/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "DifferentialActControl.hpp"
#include "../RoverDifferentialActuatorMapping.hpp"

using namespace time_literals;

DifferentialActControl::DifferentialActControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
}

void DifferentialActControl::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ro_accel_limit.get() > FLT_EPSILON && _param_ro_max_thr_speed.get() > FLT_EPSILON) {
		_adjusted_throttle_setpoint.setSlewRate(_param_ro_accel_limit.get() / _param_ro_max_thr_speed.get());
	}
}

void DifferentialActControl::updateActControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 10_ms) * 1e-6f;

	if (_rover_throttle_setpoint_sub.updated()) {
		rover_throttle_setpoint_s rover_throttle_setpoint{};
		_rover_throttle_setpoint_sub.copy(&rover_throttle_setpoint);
		_throttle_setpoint = rover_throttle_setpoint.throttle_body_x;
	}

	if (_rover_steering_setpoint_sub.updated()) {
		rover_steering_setpoint_s rover_steering_setpoint{};
		_rover_steering_setpoint_sub.copy(&rover_steering_setpoint);
		_speed_diff_setpoint = rover_steering_setpoint.normalized_steering_setpoint;
	}

	if (PX4_ISFINITE(_throttle_setpoint) && PX4_ISFINITE(_speed_diff_setpoint)) {
		actuator_motors_s actuator_motors_sub{};
		_actuator_motors_sub.copy(&actuator_motors_sub);

		const int motor_offset = _param_rd_mot_offset.get();
		float wheel_command_sum = 0.f;
		int valid_wheel_commands = 0;

		for (int i = 0; i < 4 && motor_offset + i < actuator_motors_s::NUM_CONTROLS; ++i) {
			const float wheel_command = actuator_motors_sub.control[motor_offset + i];

			if (PX4_ISFINITE(wheel_command)) {
				wheel_command_sum += wheel_command;
				++valid_wheel_commands;
			}
		}

		const float current_throttle = valid_wheel_commands > 0 ? wheel_command_sum / valid_wheel_commands : 0.f;
		const float adjusted_throttle_setpoint = RoverControl::throttleControl(_adjusted_throttle_setpoint,
				_throttle_setpoint, current_throttle, _param_ro_accel_limit.get(),
				_param_ro_decel_limit.get(), _param_ro_max_thr_speed.get(), dt);
		actuator_motors_s actuator_motors{};
		_actuator_motors_sub.copy(&actuator_motors);
		actuator_motors.reversible_flags = _param_r_rev.get();

		const Vector2f wheel_commands = computeInverseKinematics(adjusted_throttle_setpoint, _speed_diff_setpoint);

		if (!RoverDifferentialActuatorMapping::writeWheelCommands(
			    actuator_motors, motor_offset, wheel_commands(0), wheel_commands(1))) {
			PX4_ERR("RD_MOT_OFFSET out of range");
			return;
		}

		actuator_motors.timestamp = _timestamp;
		_actuator_motors_pub.publish(actuator_motors);

	}

}

Vector2f DifferentialActControl::computeInverseKinematics(float throttle, const float speed_diff_normalized)
{
	float max_motor_command = fabsf(throttle) + fabsf(speed_diff_normalized);

	if (max_motor_command > 1.0f) { // Prioritize yaw rate if a normalized motor command exceeds limit of 1
		float excess = fabsf(max_motor_command - 1.0f);
		throttle -= sign(throttle) * excess;
	}

	// Calculate the left and right wheel speeds
	return Vector2f(throttle - speed_diff_normalized,
			throttle + speed_diff_normalized);
}

void DifferentialActControl::stopVehicle()
{
	actuator_motors_s actuator_motors{};
	_actuator_motors_sub.copy(&actuator_motors);
	actuator_motors.reversible_flags = _param_r_rev.get();
	for (float &control : actuator_motors.control) {
		control = NAN;
	}
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);
}

void DifferentialActControl::publishHeartbeat()
{
	actuator_motors_s actuator_motors{};
	_actuator_motors_sub.copy(&actuator_motors);
	actuator_motors.reversible_flags = _param_r_rev.get();

	if (!RoverDifferentialActuatorMapping::writeWheelCommands(
		    actuator_motors, _param_rd_mot_offset.get(), 0.f, 0.f)) {
		return;
	}

	actuator_motors.timestamp = hrt_absolute_time();
	_actuator_motors_pub.publish(actuator_motors);
}
