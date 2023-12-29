/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcx_espi_taf

#include <soc.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_saf.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(espi_taf, CONFIG_ESPI_LOG_LEVEL);

static const struct device *const spi_dev = DEVICE_DT_GET(DT_ALIAS(taf_flash));

struct espi_taf_npcx_config {
	uintptr_t base;
	uintptr_t mapped_addr;
	enum ESPI_FLASH_TAF_ERASE_BLOCK_SIZE erase_sz;
	enum ESPI_FLASH_TAF_MAX_READ_REQ max_rd_sz;
};

struct espi_taf_npcx_data {
	sys_slist_t callbacks;
};

#define HAL_INSTANCE(dev)						\
	((struct espi_reg *)((const struct espi_taf_npcx_config *)	\
	(dev)->config)->base)

#define DT_INST_ERASE_SZ_PROP_OR(inst)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, erase_sz),		\
		    (_CONCAT(ESPI_FLASH_TAF_ERASE_BLOCK_,		\
		     DT_INST_STRING_TOKEN(inst, erase_sz))),		\
		    ((ESPI_FLASH_TAF_ERASE_BLOCK_SIZE_4KB)))

#define DT_INST_MAX_READ_SZ_PROP_OR(inst)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, max_read_sz),		\
		    (_CONCAT(ESPI_FLASH_TAF_MAX_READ_REQ_,		\
		     DT_INST_STRING_TOKEN(inst, max_read_sz))),		\
		    ((ESPI_FLASH_TAF_MAX_READ_REQ_64B)))

/* Check access region of read request is protected or not */
bool espi_taf_check_read_protect(const struct device *dev, uint32_t addr, uint32_t len,
				 uint8_t tag)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint8_t i = 0;
	uint16_t override_rd;
	uint32_t flash_addr = addr;
	uint32_t base, high;
	bool rdpr;

	flash_addr +=
		GET_FIELD(inst->FLASHBASE, NPCX_FLASHBASE_FLBASE_ADDR)
		<< GET_POS(NPCX_FLASHBASE_FLBASE_ADDR);

	for (i = 0; i < CONFIG_ESPI_TAF_PR_NUM; i++) {
		base = GET_FIELD(inst->FLASH_PRTR_BADDR[i],
				 NPCX_FLASH_PRTR_BADDR)
				 << GET_POS(NPCX_FLASH_PRTR_BADDR);
		high = GET_FIELD(inst->FLASH_PRTR_HADDR[i],
				 NPCX_FLASH_PRTR_HADDR)
				 << GET_POS(NPCX_FLASH_PRTR_HADDR);
		high |= 0xFFF;

		rdpr = IS_BIT_SET(inst->FLASH_PRTR_BADDR[i], NPCX_FRGN_RPR) ? true : false;
		override_rd = GET_FIELD(inst->FLASH_RGN_TAG_OVR[i], NPCX_FLASH_TAG_OVR_RPR);

		if (rdpr && !IS_BIT_SET(override_rd, tag) &&
		    (base <= flash_addr + len - 1 && flash_addr <= high)) {
			return true;
		}
	}

	return false;
}

/* Check access region of write request is protected or not */
bool espi_taf_check_write_protect(const struct device *dev, uint32_t addr,
				  uint32_t len, uint8_t tag)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint8_t i = 0;
	uint16_t override_wr;
	uint32_t base, high;
	uint32_t flash_addr = addr;
	bool wrpr;

	flash_addr +=
		GET_FIELD(inst->FLASHBASE, NPCX_FLASHBASE_FLBASE_ADDR)
		<< GET_POS(NPCX_FLASHBASE_FLBASE_ADDR);

	for (i = 0; i < CONFIG_ESPI_TAF_PR_NUM; i++) {
		base = GET_FIELD(inst->FLASH_PRTR_BADDR[i], NPCX_FLASH_PRTR_BADDR)
				 << GET_POS(NPCX_FLASH_PRTR_BADDR);
		high = GET_FIELD(inst->FLASH_PRTR_HADDR[i], NPCX_FLASH_PRTR_HADDR)
				 << GET_POS(NPCX_FLASH_PRTR_HADDR);
		high |= 0xFFF;

		wrpr = IS_BIT_SET(inst->FLASH_PRTR_BADDR[i], NPCX_FRGN_WPR) ? true : false;
		override_wr = GET_FIELD(inst->FLASH_RGN_TAG_OVR[i], NPCX_FLASH_TAG_OVR_WPR);

		if (wrpr && !IS_BIT_SET(override_wr, tag) &&
		    (base <= flash_addr + len - 1 && flash_addr <= high)) {
			return true;
		}
	}

	return false;
}

int espi_taf_npcx_configuration(const struct device *dev, const struct espi_saf_cfg *cfg)
{
	LOG_INF("%s", __func__);
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	const struct espi_saf_hw_cfg *hwcfg = &cfg->hwcfg;

	if (hwcfg->mode == ESPI_TAF_STANDARD_MODE) {
		inst->FLASHCTL &= ~BIT(NPCX_FLASHCTL_SAF_AUTO_READ);
	} else {
		inst->FLASHCTL |= BIT(NPCX_FLASHCTL_SAF_AUTO_READ);
	}

	return 0;
}

int espi_taf_npcx_set_pr(const struct device *dev, const struct espi_saf_protection *pr)
{
	LOG_DBG("%s", __func__);
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	const struct espi_saf_pr *preg = pr->pregions;
	size_t n = pr->nregions;
	uint8_t regnum;
	uint32_t rw_pr, override_rw;

	if ((dev == NULL) || (pr == NULL)) {
		return -EINVAL;
	}

	if (pr->nregions >= CONFIG_ESPI_TAF_PR_NUM) {
		return -EINVAL;
	}

	while (n--) {
		regnum = preg->pr_num;

		if (regnum >= CONFIG_ESPI_TAF_PR_NUM) {
			return -EINVAL;
		}

		rw_pr = preg->master_bm_we << NPCX_FRGN_WPR;
		rw_pr = rw_pr | (preg->master_bm_rd << NPCX_FRGN_RPR);

		if (preg->flags) {
			inst->FLASH_PRTR_BADDR[regnum] = (preg->start << 12U) | rw_pr;
			inst->FLASH_PRTR_HADDR[regnum] = preg->end << 12U;
		}

		override_rw = (preg->override_r << 16) | preg->override_w;
		inst->FLASH_RGN_TAG_OVR[regnum] = override_rw;
		preg++;
	}

	return 0;
}

static int espi_taf_npcx_activate(const struct device *dev)
{
	LOG_DBG("%s", __func__);
	struct espi_reg *const inst = HAL_INSTANCE(dev);

	inst->FLASHCTL &= ~BIT(NPCX_FLASHCTL_AUTO_RD_DIS_CTL);
	inst->FLASHCTL &= ~BIT(NPCX_FLASHCTL_BLK_FLASH_NP_FREE);

	return 0;
}

bool espi_taf_npcx_channel_ready(const struct device *dev)
{
	LOG_DBG("%s", __func__);
	struct espi_reg *const inst = HAL_INSTANCE(dev);

	if (!IS_BIT_SET(inst->ESPICFG, NPCX_ESPICFG_FLCHANMODE)) {
		return false;
	}
	return true;
}

/* This routine set FLASH_C_AVAIL for standard request */
void taf_set_flash_c_avail(const struct device *dev)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint32_t tmp = inst->FLASHCTL;

	/*
	 * Clear FLASHCTL_FLASH_NP_FREE to avoid host puts a flash
	 * standard request command at here.
	 */
	tmp &= FLASHCTL_ACCESS_MASK;

	/* set FLASHCTL_FLASH_TX_AVAIL */
	tmp |= BIT(NPCX_FLASHCTL_FLASH_TX_AVAIL);
	inst->FLASHCTL = tmp;
}

/* This routine release FLASH_NP_FREE for standard request */
void taf_release_flash_np_free(const struct device *dev)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint32_t tmp = inst->FLASHCTL;

	/*
	 * Clear FLASHCTL_FLASH_TX_AVAIL to avoid host puts a
	 * GET_FLASH_C command at here.
	 */
	tmp &= FLASHCTL_ACCESS_MASK;

	/* release FLASH_NP_FREE */
	tmp |= BIT(NPCX_FLASHCTL_FLASH_NP_FREE);
	inst->FLASHCTL = tmp;
}

/* This routine resets eSPI flash channel indirect buffer head */
void reset_indirect_buffer_head(const struct device *dev)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint32_t tmp;

	tmp = inst->FLASHCTL;

	/*
	 * Clear FLASHCTL_FLASH_NP_FREE and FLASHCTL_FLASH_TX_AVAIL
	 * to avoid host puts a flash standard request or a GET_FLASH_C
	 * command at here.
	 */
	tmp &= FLASHCTL_ACCESS_MASK;

	/* release FLASHCTL_RSTBUFHEADS */
	tmp |= BIT(NPCX_FLASHCTL_RSTBUFHEADS);
	inst->FLASHCTL = tmp;
}

void taf_npcx_completion_handler(const struct device *dev, uint32_t *buffer)
{
	LOG_DBG("%s", __func__);
	uint8_t i;
	uint16_t size = DIV_ROUND_UP((uint8_t)(buffer[0]) + 1, sizeof(uint32_t));
	struct espi_reg *const inst = HAL_INSTANCE(dev);

	/* Check the Flash Access TX Queue is empty by polling
	 * FLASH_TX_AVAIL.
	 */
	while (IS_BIT_SET(inst->FLASHCTL, NPCX_FLASHCTL_FLASH_TX_AVAIL)) {
		;
	}

	/* check ESPISTS.FLNACS is clear (no slave completion is detected) */
	while (IS_BIT_SET(inst->ESPISTS, NPCX_ESPISTS_FLNACS)) {
		;
	}

	/* Write packet to FLASHTXBUF */
	if (IS_ENABLED(CONFIG_ESPI_TAF_DIRECT_ACCESS)) { /* direct mode */
		for (i = 0; i < size; i++) {
			inst->FLASHTXBUF[i] = buffer[i];
		}
	} else { /* indirect mode */
		reset_indirect_buffer_head(dev);
		for (i = 0; i < size; i++) {
			inst->FLASHTXWRHEAD = buffer[i];
		}
	}

	/* Set the FLASHCTL.FLASH_TX_AVAIL bit to 1 to enqueue the packet */
	taf_set_flash_c_avail(dev);

	/* release FLASH_NP_FREE here to ready get next TAF request*/
	if ((MSB2(buffer[0]) != CYC_SCS_CMP_WITH_DATA_FIRST) &&
	    (MSB2(buffer[0]) != CYC_SCS_CMP_WITH_DATA_MIDDLE)) {
		taf_release_flash_np_free(dev);
	}
}

int espi_taf_npcx_flash_read(const struct device *dev, struct espi_saf_packet *pckt)
{
	LOG_DBG("%s", __func__);
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	struct espi_taf_npcx_pckt *taf_data_ptr = (struct espi_taf_npcx_pckt *)pckt->buf;
	uint8_t *data_ptr = (uint8_t *)taf_data_ptr->data;
	uint8_t cycle_type = CYC_SCS_CMP_WITH_DATA_ONLY;
	uint32_t total_len = pckt->len;
	uint32_t len = total_len;
	uint32_t addr = pckt->flash_addr;
	uint8_t flashreqsize = GET_FIELD(inst->FLASHCFG, NPCX_FLASHCFG_FLASHREQSIZE);
	uint8_t targetmaxsize = GET_FIELD(inst->FLASHCFG, NPCX_FLASHCFG_FLREQSUP);
	uint16_t max_read_req = 32 << flashreqsize;
	int rc;

	if (flashreqsize > targetmaxsize) {
		LOG_DBG("Exceeded the maximum supported length");
		if (targetmaxsize == 0) {
			targetmaxsize = 1;
		}
		max_read_req = 32 << targetmaxsize;
	}

	if (total_len > max_read_req) {
		LOG_ERR("Exceeded the limitation of read length");
		return -EINVAL;
	}

	if (espi_taf_check_read_protect(dev, addr, len, taf_data_ptr->tag)) {
		LOG_ERR("Access protect region");
		return -EINVAL;
	}

	if (total_len <= MAX_FLASH_REQUEST) {
		cycle_type = CYC_SCS_CMP_WITH_DATA_ONLY;
		len = total_len;
	} else {
		cycle_type = CYC_SCS_CMP_WITH_DATA_FIRST;
		len = MAX_FLASH_REQUEST;
	}

	do {
		data_ptr = (uint8_t *)taf_data_ptr->data;
		*data_ptr++ = len + 3;
		*data_ptr++ = cycle_type;
		*data_ptr++ = MAKE8(LSN(MSB2(len)), taf_data_ptr->tag);
		*data_ptr++ = MSB3(len);

		rc = flash_read(spi_dev, addr, data_ptr, len);
		if (rc) {
			LOG_ERR("flash read fail 0x%x", rc);
			return -EIO;
		}

		taf_npcx_completion_handler(dev, (uint32_t *)taf_data_ptr->data);

		total_len -= len;
		addr += len;

		if (total_len <= MAX_FLASH_REQUEST) {
			cycle_type = CYC_SCS_CMP_WITH_DATA_LAST;
			len = total_len;
		} else {
			cycle_type = CYC_SCS_CMP_WITH_DATA_MIDDLE;
		}
	} while (total_len);

	return 0;
}

int espi_taf_npcx_flash_write(const struct device *dev, struct espi_saf_packet *pckt)
{
	LOG_DBG("%s", __func__);
	struct espi_taf_npcx_pckt *taf_data_ptr = (struct espi_taf_npcx_pckt *)pckt->buf;
	uint8_t *data_ptr = (uint8_t *)(taf_data_ptr->data);
	uint16_t len;
	int rc;

	if (espi_taf_check_write_protect(dev, pckt->flash_addr,
					 pckt->len, taf_data_ptr->tag)) {
		LOG_ERR("Access protection region");
		return -EINVAL;
	}

	rc = flash_write(spi_dev, pckt->flash_addr, data_ptr, pckt->len);
	if (rc) {
		LOG_ERR("flash write fail 0x%x", rc);
		return -EIO;
	}

	/* Completion without Data the length field must be driven to zeros */
	len = 0;

	*data_ptr++ = len + 3;
	*data_ptr++ = CYC_SCS_CMP_WITHOUT_DATA;
	*data_ptr++ = MAKE8(LSN(MSB2(len)), taf_data_ptr->tag);
	*data_ptr++ = MSB3(len);

	taf_npcx_completion_handler(dev, (uint32_t *)taf_data_ptr->data);
	return 0;
}

int espi_taf_npcx_flash_erase(const struct device *dev, struct espi_saf_packet *pckt)
{
	LOG_DBG("%s", __func__);
	struct espi_taf_npcx_pckt *taf_data_ptr = (struct espi_taf_npcx_pckt *)pckt->buf;
	uint8_t *data_ptr = (uint8_t *)taf_data_ptr->data;
	uint32_t addr = pckt->flash_addr;
	uint32_t len = pckt->len;
	int rc;

	if (espi_taf_check_write_protect(dev, addr, len, taf_data_ptr->tag)) {
		LOG_ERR("Access protect region");
		return -EINVAL;
	}

	rc = flash_erase(spi_dev, addr, len);
	if (rc) {
		LOG_ERR("flash erase fail");
		return -EIO;
	}

	/* Completion without Data the length field must be driven to zeros */
	len = 0;

	*data_ptr++ = len + 3;
	*data_ptr++ = CYC_SCS_CMP_WITHOUT_DATA;
	*data_ptr++ = MAKE8(LSN(MSB2(len)), taf_data_ptr->tag);
	*data_ptr++ = MSB3(len);

	taf_npcx_completion_handler(dev, (uint32_t *)taf_data_ptr->data);
	return 0;
}

int espi_taf_npcx_flash_unsuccess(const struct device *dev, struct espi_saf_packet *pckt)
{
	LOG_DBG("%s", __func__);
	uint16_t len;
	struct espi_taf_npcx_pckt *taf_data_ptr
			= (struct espi_taf_npcx_pckt *)pckt->buf;
	uint8_t *data_ptr = (uint8_t *)taf_data_ptr->data;

	/* Completion without Data the length field must be driven to zeros */
	len = 0;

	*data_ptr++ = len + 3;
	*data_ptr++ = CYC_UNSCS_CMP_WITHOUT_DATA_ONLY;
	*data_ptr++ = MAKE8(LSN(MSB2(len)), taf_data_ptr->tag);
	*data_ptr++ = MSB3(len);

	taf_npcx_completion_handler(dev, (uint32_t *)taf_data_ptr->data);
	return 0;
}

static int espi_taf_npcx_manage_callback(const struct device *dev, struct espi_callback *callback,
					 bool set)
{
	LOG_DBG("%s", __func__);
	return 0;
}

int espi_taf_npcx_init(const struct device *dev)
{
	LOG_DBG("%s", __func__);
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	const struct espi_taf_npcx_config *config = dev->config;

	SET_FIELD(inst->FLASHCFG, NPCX_FLASHCFG_FLCAPA,
		  ESPI_FLASH_SHARING_CAP_SUPP_TAF_AND_CAF);
	SET_FIELD(inst->FLASHCFG, NPCX_FLASHCFG_TRGFLEBLKSIZE,
		  BIT(config->erase_sz));
	SET_FIELD(inst->FLASHCFG, NPCX_FLASHCFG_FLREQSUP,
		  config->max_rd_sz);
	inst->FLASHBASE = config->mapped_addr;

	return 0;
}

static const struct espi_saf_driver_api espi_taf_npcx_driver_api = {
	.config = espi_taf_npcx_configuration,
	.set_protection_regions = espi_taf_npcx_set_pr,
	.activate = espi_taf_npcx_activate,
	.get_channel_status = espi_taf_npcx_channel_ready,
	.flash_read = espi_taf_npcx_flash_read,
	.flash_write = espi_taf_npcx_flash_write,
	.flash_erase = espi_taf_npcx_flash_erase,
	.flash_unsuccess = espi_taf_npcx_flash_unsuccess,
	.manage_callback = espi_taf_npcx_manage_callback,
};

static struct espi_taf_npcx_data espi_taf_npcx_data;

static const struct espi_taf_npcx_config espi_taf_npcx_config = {
	.base = DT_INST_REG_ADDR(0),
	.mapped_addr = DT_INST_PROP(0, mapped_addr),
	.erase_sz = DT_INST_ERASE_SZ_PROP_OR(0),
	.max_rd_sz = DT_INST_MAX_READ_SZ_PROP_OR(0),
};

DEVICE_DT_INST_DEFINE(0, &espi_taf_npcx_init, NULL,
			&espi_taf_npcx_data, &espi_taf_npcx_config,
			PRE_KERNEL_2, CONFIG_ESPI_INIT_PRIORITY,
			&espi_taf_npcx_driver_api);
