/*
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCX_SOC_ESPI_H_
#define _NUVOTON_NPCX_SOC_ESPI_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

enum espi_host_subs_peripheral {
	ESPI_PERIPHERAL_SHARED_MEMORY = 0x80,
	ESPI_PERIPHERAL_MAILBOX,
};

#define SHM_EVENT_CHANNEL_MASK 0xFF000000u
#define SHM_EVENT_CHANNEL_POS  24u
#define SHM_EVENT_CHANNEL(x)   ((x & SHM_EVENT_CHANNEL_MASK) >> SHM_EVENT_CHANNEL_POS)
#define SHM_EVENT_TYPE_MASK 0x00FF0000u
#define SHM_EVENT_TYPE_POS  16u
#define SHM_EVENT_TYPE(x)   ((x & SHM_EVENT_TYPE_MASK) >> SHM_EVENT_TYPE_POS)


/* SHM window number */
typedef enum {
	SHM_WINDOW_ONE      = 0,
	SHM_WINDOW_TWO      = 1,
    SHM_WINDOW_THREE    = 2,
    SHM_WINDOW_FOUR     = 3,
	SHM_WINDOW_NUM_MAX
} shm_window_t;

/* SHM Event Type */
typedef enum {
	SHM_HOST_ACCESS = 0,
	SHM_HOST_SEMAPHORE_WRITTEN,
	SHM_HOST_WRITE_TO_OFFSET,
	SHM_HOST_READ_FROM_OFFSET,
} shm_event_type_t;

/**
 * @brief Turn on all interrupts of eSPI host interface module.
 *
 * @param dev Pointer to structure device of eSPI module
 */
void npcx_espi_enable_interrupts(const struct device *dev);

/**
 * @brief Turn off all interrupts of eSPI host interface module.
 *
 * @param dev Pointer to structure device of eSPI module
 */
void npcx_espi_disable_interrupts(const struct device *dev);

#if defined(CONFIG_SOC_SERIES_NPCX4)
/**
 * @brief Send a software interrupt via Virtual Wire.
 *
 * @param dev Pointer to structure device of eSPI module
 * @param swirq_num Software interrupt number to send
 * @param edge 
 * @param level 
 *
 * @return int 
 */
int npcx_espi_vw_send_swirq(const struct device *dev, uint8_t swirq_num, bool edge, uint8_t level);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _NUVOTON_NPCX_SOC_ESPI_H_ */
