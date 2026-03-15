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

/**
 * @brief Activate or deactivate the npcx i3c controller.
 *
 * @param dev Pointer to the device structure for i3c controller instance.
 * @param enable True to activate the controller, false to deactivate.
 * @return int 0 if successful, negative error code if failed.
 */
int npcx_i3c_activate(const struct device *dev, bool enable);

/* MIPI I3C MDB definition: see https://www.mipi.org/MIPI_I3C_mandatory_data_byte_values_public */
#define IBI_MDB_ID(grp, id)		((((grp) << 5) & GENMASK(7, 5)) | ((id) & GENMASK(4, 0)))
#define IBI_MDB_GET_GRP(m)		(((m) & GENMASK(7, 5)) >> 5)
#define IBI_MDB_GET_ID(m)		((m) & GENMASK(4, 0))

#define IBI_MDB_GRP_PENDING_READ_NOTIF	0x5
#define IS_MDB_PENDING_READ_NOTIFY(m)	(IBI_MDB_GET_GRP(m) == IBI_MDB_GRP_PENDING_READ_NOTIF)
#define IBI_MDB_MIPI_DBGDATAREADY	IBI_MDB_ID(IBI_MDB_GRP_PENDING_READ_NOTIF, 0xd)
#define IBI_MDB_MCTP			IBI_MDB_ID(IBI_MDB_GRP_PENDING_READ_NOTIF, 0xe)
/* Interrupt ID 0x10 to 0x1F are for vendor specific */
#define IBI_MDB_ASPEED			I

/* slave events */
#define I3C_SLAVE_EVENT_SIR		BIT(0)
#define I3C_SLAVE_EVENT_MR		BIT(1)
#define I3C_SLAVE_EVENT_HJ		BIT(2)

struct i3c_dev_attached_list *npcx_i3c_get_device_attached_list(const struct device *dev);
int npcx_i3c_slave_set_static_addr(const struct device *dev, uint8_t static_addr);
int npcx_i3c_slave_get_dynamic_addr(const struct device *dev, uint8_t *dynamic_addr);
int npcx_i3c_slave_get_event_enabling(const struct device *dev, uint32_t *event_en);

#define i3c_get_device_attached_list	npcx_i3c_get_device_attached_list
#define i3c_slave_set_static_addr	npcx_i3c_slave_set_static_addr
#define i3c_slave_get_dynamic_addr	npcx_i3c_slave_get_dynamic_addr
#define i3c_slave_get_event_enabling	npcx_i3c_slave_get_event_enabling
#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_I3C_I3C_NPCX_H_ */