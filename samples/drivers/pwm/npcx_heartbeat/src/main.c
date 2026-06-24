/*
 * Copyright (c) 2026 Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm/pwm_hb_npcx.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#if !DT_HAS_COMPAT_STATUS_OKAY(nuvoton_npcx_pwm_heartbeat)
#error "No nuvoton,npcx-pwm-heartbeat node found in devicetree"
#endif

#define HB_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nuvoton_npcx_pwm_heartbeat)

static const struct pwm_npcx_hb_profile hb_profile_standard = {
	.rise_time = 1000u,
	.on_time = 500u,
	.fall_time = 2000u,
	.off_time = 500u,
	.hb_mode = NPCX_PWM_HB_MODE_STANDARD,
	.max_duty_per = 100u,
	.min_duty_per = 0u,
};

static const struct pwm_npcx_hb_profile hb_profile_visual = {
	.rise_time = 800u,
	.on_time = 400u,
	.fall_time = 800u,
	.off_time = 700u,
	.hb_mode = NPCX_PWM_HB_MODE_STANDARD,
	.max_duty_per = 80u,
	.min_duty_per = 5u,
};

int main(void)
{
	const struct device *const hb_dev = DEVICE_DT_GET(HB_NODE);
	uint16_t hz;

	if (!device_is_ready(hb_dev)) {
		printk("NPCX heartbeat device is not ready\n");
		return 0;
	}

	printk("NPCX PWM heartbeat sample start\n");

	while (1) {
		hz = pwm_npcx_hb_configure(hb_dev, &hb_profile_standard);
		if (hz == 0u) {
			printk("Failed to apply heartbeat profile\n");
			return 0;
		}
	
		k_sleep(K_SECONDS(11));
	
		hz = pwm_npcx_hb_configure(hb_dev, &hb_profile_visual);
		if (hz == 0u) {
			printk("Failed to apply heartbeat profile\n");
			return 0;
		}
	
//	while (1) {
		k_sleep(K_SECONDS(9));
	}

	return 0;
}
