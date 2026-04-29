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
 * @brief Function to fill rx data to release clock strech in i2c target mode.
 * 
 * @param i2c_ctrl_dev Pointer to the device structure for i2c controller instance.
 * @param val The byte of data to be transferred to the i2c target device.
 */
void i2c_rx_fill_n_release_clock_strech(const struct device *i2c_ctrl_dev, uint8_t val);

/**
 * @brief Check if the npcx i2c controller is activated.
 * 
 * @param dev Pointer to the device structure for i2c controller instance.
 * @return true if the controller is activated, false otherwise.
 */
bool is_npcx_i2c_activated(const struct device *dev);

/**
 * @brief Activate or deactivate the npcx i2c controller.
 * 
 * @param dev Pointer to the device structure for i2c controller instance.
 * @param enable True to activate the controller, false to deactivate.
 * @return int 0 if successful, negative error code if failed.
 */
int npcx_i2c_activate(const struct device *dev, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I2C_I2C_NPCX_H_ */