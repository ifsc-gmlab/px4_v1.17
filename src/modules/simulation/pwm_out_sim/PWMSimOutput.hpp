#pragma once

#include <cstdint>


inline float normalizeSimOutput(uint16_t output, bool non_reversible_motor,
				float motor_idle)
{
	static constexpr float pwm_min = 1000.f;
	static constexpr float pwm_max = 2000.f;
	static constexpr uint16_t pwm_failsafe = 600;

	if (non_reversible_motor) {
		const float normalized = (static_cast<float>(output) - pwm_min) / (pwm_max - pwm_min);
		return fmaxf(normalized, motor_idle);
	}

	// The SITL output limiter uses 600 as a sentinel, not as a real PWM
	// command. Mapping it as a reversible output would produce -1.8 and can
	// violently drive rover wheels during failsafe.
	if (output == pwm_failsafe) {
		return 0.f;
	}

	const float pwm_center = (pwm_max + pwm_min) / 2.f;
	const float pwm_delta = (pwm_max - pwm_min) / 2.f;
	return (static_cast<float>(output) - pwm_center) / pwm_delta;
}
