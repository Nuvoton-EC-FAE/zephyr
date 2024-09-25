/*
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I3C_I3C_NPCX_H_
#define ZEPHYR_DRIVERS_I3C_I3C_NPCX_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_I3C_NPCX_DMA

/**
 * @brief Program memory of the virtual EEPROM
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param eeprom_data Pointer of data to program into the virtual eeprom memory
 * @param length Length of data to program into the virtual eeprom memory
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid data size
 */

/**
 * @brief Read single byte of virtual EEPROM memory
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param eeprom_data Pointer of byte where to store the virtual eeprom memory
 * @param offset Offset into EEPROM memory where to read the byte
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid data pointer or offset
 */

/**
 * @brief Function to configure the application's buffer the will be used for
 *        MDMA mode.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mdma_rd_buf Pointer to storage for read data
 * @param mdma_rd_buf_size Length of the buffer to storage for read data
 * @param mdma_wr_buf Pointer to the data to be written
 * @param mdma_wr_buf_size Length of the buffer to be written
 */
void npcx_i3c_target_set_mdma_buff(const struct device *dev,
									uint8_t *mdma_rd_buf,
									uint16_t mdma_rd_buf_size,
									uint8_t *mdma_wr_buf,
									uint16_t mdma_wr_buf_size);

/**
 * @brief Function to get the received data count by MDMA
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @return  the count of data the read from bus
 */
uint16_t npcx_i3c_target_get_mdmafb_count(const struct device *dev);

/**
 * @brief Function to get the data count of write to bus
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @return  the count of data the has write to bus
 */
uint16_t npcx_i3c_target_get_mdmatb_count(const struct device *dev);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I3C_I3C_NPCX_H_ */