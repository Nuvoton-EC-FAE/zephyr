/*
 * Copyright (c) 2026 Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcx_pwm_heartbeat

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pwm/pwm_hb_npcx.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <soc.h>

#define NPCX_PWM_HB_ONOFF_PRESCALER 819u
#define NPCX_PWM_MAX_CTR            0x0FFFu
#define NPCX_PWM_HB_FRCLK_HZ        930000u

#define NPCX_PWM_HB_CTR_RS_OFFSET      0x02u
#define NPCX_PWM_HB_N_STEP_RS_OFFSET   0x05u
#define NPCX_PWM_HB_MAX_DC_RS_OFFSET   0x06u
#define NPCX_PWM_HB_CTR_FL_OFFSET      0x08u
#define NPCX_PWM_HB_MAX_DC_FL_OFFSET   0x0Au
#define NPCX_PWM_HB_N_STEP_FL_OFFSET   0x0Du
#define NPCX_PWM_HB_EXT_ON_OFFSET      0x0Eu
#define NPCX_PWM_HB_EXT_OFF_OFFSET     0x0Fu
#define NPCX_PWM_HB_MIN_DC_RSFL_OFFSET 0x10u

#ifndef NPCX_PWMCTL_HBNK_SEL
#define NPCX_PWMCTL_HBNK_SEL        6
#endif

struct pwm_npcx_hb_dev_cfg {
	/* pwm controller base address */
	struct pwm_reg *base;
};

struct pwm_npcx_hb_cfg {
	const struct device *pwm_dev;
	uint8_t polarity;
};

static inline void pwm_npcx_hb_write8(struct pwm_reg *inst,
				      uint32_t offset,
				      uint8_t val)
{
	sys_write8(val, ((uintptr_t)inst) + offset);
}

static inline void pwm_npcx_hb_write16(struct pwm_reg *inst,
				       uint32_t offset,
				       uint16_t val)
{
	sys_write16(val, ((uintptr_t)inst) + offset);
}

static uint32_t pwm_npcx_hb_calc_sqrt(uint32_t n)
{
	uint32_t sqrt_n = n;
	uint32_t xk;

	if (n == 0u) {
		return 0u;
	}

	if (n <= 3u) {
		return 1u;
	}

	do {
		xk = sqrt_n;
		sqrt_n = (n / xk + xk) / 2u;
	} while (sqrt_n < xk);

	return sqrt_n;
}

/*
 * Configure NPCX PWM heartbeat profile and return achieved PWM frequency (Hz).
 * Return 0 on invalid parameters.
 */
uint16_t pwm_npcx_config_heartbeat(
	const struct device *pwm_dev,
	uint16_t rise_time,
	uint16_t on_time,
	uint16_t fall_time,
	uint16_t off_time,
	uint8_t hb_mode,
	uint8_t max_duty_per,
	uint8_t min_duty_per)
{
	const struct pwm_npcx_hb_dev_cfg *cfg;
	struct pwm_reg *inst;
	uint8_t ext_off, ext_on;
	uint16_t prescaler;
	uint16_t max_dc = 0u;
	uint16_t ctr = 0u;
	uint16_t hz = 0u;
	uint16_t steps = 0u;
	uint16_t steps_multiplier = (hb_mode == NPCX_PWM_HB_MODE_STANDARD) ? 4u : 23u;
	uint16_t max_on_off_time = MAX(on_time, off_time);
	uint16_t max_rise_fall_time = MAX(rise_time, fall_time);
	uint32_t clock;
	uint32_t max_clock;
	uint32_t time;
	uint8_t delta_duty_cycle;

	if ((pwm_dev == NULL) || !device_is_ready(pwm_dev)) {
		return 0u;
	}

	cfg = (const struct pwm_npcx_hb_dev_cfg *)pwm_dev->config;
	if ((cfg == NULL) || (cfg->base == NULL)) {
		return 0u;
	}

	inst = cfg->base;

	if ((max_duty_per == 0u) || (min_duty_per >= max_duty_per)) {
		inst->PWMCTL &= ~BIT(NPCX_PWMCTL_PWR);
		return 0u;
	}

	if (max_duty_per > 100u) {
		max_duty_per = 100u;
	}

	delta_duty_cycle = max_duty_per - min_duty_per;

	clock = NPCX_PWM_HB_FRCLK_HZ;
	max_clock = NPCX_PWM_HB_FRCLK_HZ;

	if (max_on_off_time > 0u) {
		max_clock = ((uint32_t)0xFFu * NPCX_PWM_HB_ONOFF_PRESCALER * 1000u) /
				    max_on_off_time;
		if (clock > max_clock) {
			clock = max_clock;
		}
	}

	/* Disable PWM while configuring. */
	inst->PWMCTL &= ~BIT(NPCX_PWMCTL_PWR);

	/* Force FR clock source for heartbeat calculation/operation. */
	inst->PWMCTL &= ~BIT(NPCX_PWMCTL_CKSEL);
	SET_FIELD(inst->PWMCTLEX, NPCX_PWMCTLEX_FCK_SEL_FIELD, 2u);

	prescaler = (uint16_t)MAX(1u, (NPCX_PWM_HB_FRCLK_HZ / MAX(clock, 1u)));
	clock = NPCX_PWM_HB_FRCLK_HZ / prescaler;
	if (clock > max_clock) {
		prescaler++;
		if (prescaler == 0u) {
			return 0u;
		}
		clock = NPCX_PWM_HB_FRCLK_HZ / prescaler;
	}

	/* Select heartbeat bank and mode. */
	inst->PWMCTL |= BIT(NPCX_PWMCTL_HBNK_SEL);
	SET_FIELD(inst->PWMCTL, NPCX_PWMCTL_HB_DC_CTL_FIELD, hb_mode);

	/*
	 * Start with the longest ramp (rise/fall) and derive a feasible
	 * ctr/steps combination under the hardware counter limit.
	 */
	do {
		steps = 0u;
		do {
			steps++;
			max_dc = (uint16_t)(steps * steps_multiplier);
			ctr = (uint16_t)(((uint32_t)max_dc * 100u) / delta_duty_cycle);
			ctr = MAX(ctr, 1u);
			hz = (uint16_t)(clock / ctr);
			hz = MAX(hz, 1u);
			time = ((uint32_t)steps * 4u * 1000u) / hz;
		} while ((time < max_rise_fall_time) && (steps <= 0xFF));

		if ((ctr > NPCX_PWM_MAX_CTR) || (steps > 0xFF)) {
			prescaler++;
			if (prescaler == 0u) {
				return 0u;
			}
			clock = NPCX_PWM_HB_FRCLK_HZ / prescaler;
		}
	} while ((ctr > NPCX_PWM_MAX_CTR) || (steps > 0xFF));

	inst->PRSC = (uint16_t)(prescaler - 1u);

	if (rise_time >= fall_time) {
		if (rise_time == 0u) {
			steps = 0u;
		}

		pwm_npcx_hb_write16(inst, NPCX_PWM_HB_CTR_RS_OFFSET, (uint16_t)(ctr - 1u));
		pwm_npcx_hb_write16(inst, NPCX_PWM_HB_MAX_DC_RS_OFFSET,
				   (uint16_t)(((uint32_t)max_dc * max_duty_per) / delta_duty_cycle));
		pwm_npcx_hb_write8(inst, NPCX_PWM_HB_N_STEP_RS_OFFSET, (uint8_t)steps);

		steps = 0u;
		if (fall_time > 0u) {
			ctr = (uint16_t)pwm_npcx_hb_calc_sqrt(((uint32_t)fall_time * clock *
						      steps_multiplier) /
						     ((uint32_t)delta_duty_cycle * 10u * 4u));
			ctr = MAX(ctr, 1u);
			max_dc = (uint16_t)(((uint32_t)ctr * delta_duty_cycle) / 100u);
			steps = (uint16_t)(max_dc / steps_multiplier);
		}
		pwm_npcx_hb_write16(inst, NPCX_PWM_HB_CTR_FL_OFFSET, (uint16_t)(ctr - 1u));
		pwm_npcx_hb_write16(inst, NPCX_PWM_HB_MAX_DC_FL_OFFSET,
				   (uint16_t)(((uint32_t)max_dc * max_duty_per) /
					      delta_duty_cycle));
		pwm_npcx_hb_write8(inst, NPCX_PWM_HB_N_STEP_FL_OFFSET, (uint8_t)steps);
	} else {
		if (fall_time == 0u) {
			steps = 0u;
		}

		pwm_npcx_hb_write16(inst, NPCX_PWM_HB_CTR_FL_OFFSET, (uint16_t)(ctr - 1u));
		pwm_npcx_hb_write16(inst, NPCX_PWM_HB_MAX_DC_FL_OFFSET, max_dc);
		pwm_npcx_hb_write8(inst, NPCX_PWM_HB_N_STEP_FL_OFFSET, (uint8_t)steps);

		steps = 0u;
		if (rise_time > 0u) {
			ctr = (uint16_t)pwm_npcx_hb_calc_sqrt(((uint32_t)rise_time * clock *
						      steps_multiplier) /
						     ((uint32_t)delta_duty_cycle * 10u * 4u));
			ctr = MAX(ctr, 1u);
			max_dc = (uint16_t)(((uint32_t)ctr * delta_duty_cycle) / 100u);
			steps = (uint16_t)(max_dc / steps_multiplier);
		}
		pwm_npcx_hb_write16(inst, NPCX_PWM_HB_CTR_RS_OFFSET, (uint16_t)(ctr - 1u));
		pwm_npcx_hb_write16(inst, NPCX_PWM_HB_MAX_DC_RS_OFFSET, max_dc);
		pwm_npcx_hb_write8(inst, NPCX_PWM_HB_N_STEP_RS_OFFSET, (uint8_t)steps);
	}

	if ((ctr > 0u) && (hz > (clock / ctr))) {
		hz = (uint16_t)(clock / ctr);
	}

	pwm_npcx_hb_write16(inst, NPCX_PWM_HB_MIN_DC_RSFL_OFFSET,
			   (uint16_t)(((uint32_t)ctr * min_duty_per) / 100u));

	ext_on = (uint8_t)MIN(0xFFu, ((uint32_t)on_time * clock) /
			 (NPCX_PWM_HB_ONOFF_PRESCALER * 1000u));
	pwm_npcx_hb_write8(inst, NPCX_PWM_HB_EXT_ON_OFFSET, ext_on);

	ext_off = (uint8_t)MIN(0xFFu, ((uint32_t)off_time * clock) /
			  (NPCX_PWM_HB_ONOFF_PRESCALER * 1000u));
	pwm_npcx_hb_write8(inst, NPCX_PWM_HB_EXT_OFF_OFFSET, ext_off);

	/* Return to bank 0 and enable PWM output. */
	inst->PWMCTL &= ~BIT(NPCX_PWMCTL_HBNK_SEL);
	inst->PWMCTL |= BIT(NPCX_PWMCTL_PWR);

	return hz;
}

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
static uint16_t pwm_npcx_hb_dev_configure(
	const struct device *dev,
	const struct pwm_npcx_hb_profile *profile)
{
	const struct pwm_npcx_hb_cfg *cfg = dev->config;

	if (profile == NULL) {
		return 0u;
	}

	return pwm_npcx_config_heartbeat(cfg->pwm_dev,
					 profile->rise_time,
					 profile->on_time,
					 profile->fall_time,
					 profile->off_time,
					 profile->hb_mode,
					 profile->max_duty_per,
					 profile->min_duty_per);
}

static int pwm_npcx_hb_apply_polarity(const struct device *pwm_dev, uint8_t polarity)
{
	const struct pwm_npcx_hb_dev_cfg *cfg;
	struct pwm_reg *inst;

	if ((pwm_dev == NULL) || !device_is_ready(pwm_dev)) {
		return -ENODEV;
	}

	cfg = (const struct pwm_npcx_hb_dev_cfg *)pwm_dev->config;
	if ((cfg == NULL) || (cfg->base == NULL)) {
		return -EINVAL;
	}

	inst = cfg->base;
	if (polarity == 0u) {
		inst->PWMCTL &= ~BIT(NPCX_PWMCTL_INVP);
	} else {
		inst->PWMCTL |= BIT(NPCX_PWMCTL_INVP);
	}

	return 0;
}

static int pwm_npcx_hb_init(const struct device *dev)
{
	const struct pwm_npcx_hb_cfg *cfg = dev->config;

	return pwm_npcx_hb_apply_polarity(cfg->pwm_dev, cfg->polarity);
}

static DEVICE_API(pwm_npcx_hb, pwm_npcx_hb_driver_api) = {
	.configure = pwm_npcx_hb_dev_configure,
};

#define NPCX_PWM_HB_POLARITY_FROM_DT(inst) \
	(((DT_INST_PWMS_FLAGS(inst) & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED) ? 1u : 0u)

#define NPCX_PWM_HB_INIT(inst) \
	static const struct pwm_npcx_hb_cfg pwm_npcx_hb_cfg_##inst = { \
		.pwm_dev = DEVICE_DT_GET(DT_INST_PWMS_CTLR(inst)), \
		.polarity = NPCX_PWM_HB_POLARITY_FROM_DT(inst), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, \
			      pwm_npcx_hb_init, NULL, \
			      NULL, &pwm_npcx_hb_cfg_##inst, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
			      &pwm_npcx_hb_driver_api)

DT_INST_FOREACH_STATUS_OKAY(NPCX_PWM_HB_INIT)
#endif
