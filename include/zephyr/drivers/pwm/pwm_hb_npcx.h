/*
 * Copyright (c) 2026 Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PWM_HB_NPCX_H_
#define ZEPHYR_INCLUDE_DRIVERS_PWM_HB_NPCX_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Heartbeat mode values used by NPCX hardware. */
#define NPCX_PWM_HB_MODE_VISUAL   1u
#define NPCX_PWM_HB_MODE_STANDARD 2u

/**
 * @brief NPCX heartbeat waveform profile.
 */
struct pwm_npcx_hb_profile {
	uint16_t rise_time;
	uint16_t on_time;
	uint16_t fall_time;
	uint16_t off_time;
	uint8_t hb_mode;
	uint8_t max_duty_per;
	uint8_t min_duty_per;
};

/**
 * @brief NPCX heartbeat device API.
 */
__subsystem struct pwm_npcx_hb_driver_api {
	uint16_t (*configure)(const struct device *dev,
			      const struct pwm_npcx_hb_profile *profile);
};

/**
 * @brief Configure a heartbeat instance with a caller-provided profile.
 *
 * @param dev Heartbeat device instance.
 * @param profile Configuration profile.
 *
 * @return Achieved heartbeat frequency in Hz, or 0 on error.
 */
static inline uint16_t pwm_npcx_hb_configure(const struct device *dev,
				      const struct pwm_npcx_hb_profile *profile)
{
	const struct pwm_npcx_hb_driver_api *api =
		DEVICE_API_GET(pwm_npcx_hb, dev);

	return api->configure(dev, profile);
}

/*
 * Configure NPCX PWM heartbeat profile and return achieved PWM frequency (Hz).
 * Return 0 on invalid parameters.
 *
 * Parameters:
 * - pwm_dev: Zephyr PWM device instance.
 * - rise_time: Rise transition duration in milliseconds.
 * - on_time: Full-on hold duration in milliseconds.
 * - fall_time: Fall transition duration in milliseconds.
 * - off_time: Full-off hold duration in milliseconds.
 *   The PWM polarity is preserved from the controller's existing setting.
 * - hb_mode: Heartbeat mode (use NPCX_PWM_HB_MODE_* values).
 * - max_duty_per: Maximum duty cycle percentage (1..100).
 * - min_duty_per: Minimum duty cycle percentage (0..99, less than max).
 */
uint16_t pwm_npcx_config_heartbeat(
	const struct device *pwm_dev,
	uint16_t rise_time,
	uint16_t on_time,
	uint16_t fall_time,
	uint16_t off_time,
	uint8_t hb_mode,
	uint8_t max_duty_per,
	uint8_t min_duty_per);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PWM_HB_NPCX_H_ */