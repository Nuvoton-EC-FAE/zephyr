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
