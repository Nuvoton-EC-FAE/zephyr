/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2C_I2C_NPCX_H_
#define ZEPHYR_DRIVERS_I2C_I2C_NPCX_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Activate or deactivate the npcx i2c controller.
 * 
 * @param dev Pointer to the device structure for i2c controller instance.
 * @param enable True to activate the controller, false to deactivate.
 * @return int 0 if successful, negative error code if failed.
 */
int npcx_i2c_activate(const struct device *dev, bool enable);

/**
 * @brief Enable or disable the wakeup feature for the npcx i2c controller.
 *
 * @param dev Pointer to the device structure for i2c controller instance.
 * @param enable True to enable the wakeup feature, false to disable.
 */
void npcx_i2c_wakeup_enable(const struct device *dev, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I2C_I2C_NPCX_H_ */