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

static const struct pwm_npcx_hb_profile hb_profile1 = {
	.rise_time    = 1000u,     /* 1 second rise time  */
	.on_time      = 0,         /* no on time          */
	.fall_time    = 2000u,     /* 2 seconds fall time */
	.off_time     = 0,         /* no off time         */
	.hb_mode      = NPCX_PWM_HB_MODE_STANDARD,
	.max_duty_per = 100u,      /* 100% max duty cycle */
	.min_duty_per = 0u,        /* 0% min duty cycle   */
};

static const struct pwm_npcx_hb_profile hb_profile2 = {
	.rise_time    = 800u,     /* 0.8 second rise time */
	.on_time      = 400u,     /* 0.4 second on time   */
	.fall_time    = 800u,     /* 0.8 second fall time */
	.off_time     = 700u,     /* 0.7 second off time  */
	.hb_mode      = NPCX_PWM_HB_MODE_STANDARD,
	.max_duty_per = 80u,     /* 80% max duty cycle    */
	.min_duty_per = 5u,      /* 5% min duty cycle     */
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
		hz = pwm_npcx_hb_configure(hb_dev, &hb_profile1);
		if (hz == 0u) {
			printk("Failed to apply heartbeat profile\n");
			return 0;
		}
	
		k_sleep(K_SECONDS(9));
	
		hz = pwm_npcx_hb_configure(hb_dev, &hb_profile2);
		if (hz == 0u) {
			printk("Failed to apply heartbeat profile\n");
			return 0;
		}
	
		k_sleep(K_SECONDS(9));
	}

	return 0;
}
