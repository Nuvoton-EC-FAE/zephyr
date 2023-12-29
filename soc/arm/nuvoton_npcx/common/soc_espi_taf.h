/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCX_SOC_ESPI_TAF_H_
#define _NUVOTON_NPCX_SOC_ESPI_TAF_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* data operation for eSPI TAF packet */
#define MSN(var8)        ((uint8_t)((uint8_t)(var8) >> 4))
#define LSN(var8)        ((uint8_t)((uint8_t)(var8) & 0x0F))

#define MSB0(var32)      ((uint8_t)(((uint32_t)(var32) & 0xFF000000) >> 24))
#define MSB1(var32)      ((uint8_t)(((uint32_t)(var32) & 0xFF0000) >> 16))
#define MSB2(var32)      ((uint8_t)(((uint16_t)(var32) & 0xFF00) >> 8))
#define MSB3(var32)      ((uint8_t)((var32) & 0xFF))

#define MAKE8(nlo, nhi)  ((uint8_t)(((uint8_t)(nlo)) | (((uint8_t)(nhi)) << 4)))

/* Successful Completion Without Data     */
#define CYC_SCS_CMP_WITHOUT_DATA                   0x06
/* Successful middle Completion With Data */
#define CYC_SCS_CMP_WITH_DATA_MIDDLE               0x09
/* Successful first Completion With Data  */
#define CYC_SCS_CMP_WITH_DATA_FIRST                0x0B
/* Successful last Completion With Data   */
#define CYC_SCS_CMP_WITH_DATA_LAST                 0x0D
/* Successful only Completion With Data   */
#define CYC_SCS_CMP_WITH_DATA_ONLY                 0x0F
/* Unsuccessful Completion Without Data   */
#define CYC_UNSCS_CMP_WITHOUT_DATA                 0x08
/* Unsuccessful Last Completion Without Data */
#define CYC_UNSCS_CMP_WITHOUT_DATA_LAST            0x0C
/* Unsuccessful Only Completion Without Data */
#define CYC_UNSCS_CMP_WITHOUT_DATA_ONLY            0x0E

/* TAF EC Portal read/write flash access limited to 1-64 bytes*/
#define MAX_FLASH_REQUEST                          64u

/* Clear RSTBUFHEADS, FLASH_ACC_TX_AVAIL, and FLASH_ACC_NP_FREE */
#define FLASHCTL_ACCESS_MASK                       (~0x00002003)

/* Flash Sharing Capability Support */
#define ESPI_FLASH_SHARING_CAP_SUPP_CAF_DEF        0
#define ESPI_FLASH_SHARING_CAP_SUPP_CAF            1
#define ESPI_FLASH_SHARING_CAP_SUPP_TAF            2
#define ESPI_FLASH_SHARING_CAP_SUPP_TAF_AND_CAF    3

#define _4KB_                                      (4 * 1024)
#define _32KB_                                     (32 * 1024)
#define _64KB_                                     (64 * 1024)
#define _128KB_                                    (128 * 1024)

enum ESPI_TAF_MODE {
	ESPI_TAF_STANDARD_MODE                     = 0,
	ESPI_TAF_AUTO_MODE                         = 1,
};

enum ESPI_FLASH_TAF_REQ {
	ESPI_FLASH_TAF_REQ_READ                    = 0,
	ESPI_FLASH_TAF_REQ_WRITE                   = 1,
	ESPI_FLASH_TAF_REQ_ERASE                   = 2,
	ESPI_FLASH_TAF_REQ_RPMC_OP1                = 3,
	ESPI_FLASH_TAF_REQ_RPMC_OP2                = 4,
	ESPI_FLASH_TAF_REQ_UNKNOWN                 = 5,
};

/* ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_4KB is default */
enum ESPI_FLASH_TAF_ERASE_BLOCK_SIZE {
	ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_1KB        = 0,
	ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_2KB        = 1,
	ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_4KB        = 2,
	ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_8KB        = 3,
	ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_16KB       = 4,
	ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_32KB       = 5,
	ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_64KB       = 6,
	ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_128KB      = 7,
};

/* ESPI_FLASH_TAF_MAX_READ_REQ_64B is default */
enum ESPI_FLASH_TAF_MAX_READ_REQ {
	ESPI_FLASH_TAF_MAX_READ_REQ_64B            = 1,
	ESPI_FLASH_TAF_MAX_READ_REQ_128B           = 2,
	ESPI_FLASH_TAF_MAX_READ_REQ_256B           = 3,
	ESPI_FLASH_TAF_MAX_READ_REQ_512B           = 4,
	ESPI_FLASH_TAF_MAX_READ_REQ_1024B          = 5,
	ESPI_FLASH_TAF_MAX_READ_REQ_2048B          = 6,
	ESPI_FLASH_TAF_MAX_READ_REQ_4096B          = 7,
};

struct espi_saf_hw_cfg {
	uint8_t  version;
	enum ESPI_TAF_MODE mode;
};

struct espi_saf_pr {
	uint32_t start;
	uint32_t end;
	uint16_t override_r;
	uint16_t override_w;
	uint8_t  master_bm_we;
	uint8_t  master_bm_rd;
	uint8_t  pr_num;
	uint8_t  flags;
};

struct espi_saf_protection {
	size_t nregions;
	const struct espi_saf_pr *pregions;
};

struct espi_taf_npcx_pckt {
	uint8_t tag;
	uint8_t *data;
};

struct espi_saf_packet;

struct espi_taf_pckt {
	uint8_t  type;
	uint8_t  tag;
	uint32_t addr;
	uint16_t len;
	uint32_t src[16];
};

#ifdef __cplusplus
}
#endif

#endif
