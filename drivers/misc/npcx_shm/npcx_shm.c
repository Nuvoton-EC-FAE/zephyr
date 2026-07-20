/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcx_shm_window

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/misc/npcx_shm/npcx_shm.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <soc.h>

#define NPCX_SHM_CFG_IDX_SEM_CTS 0x0A
#define NPCX_SHM_CFG_IDX_SEME_CTS 0x0A
#define NPCX_SHM_CFG_IDX_HOST_COFSE_CTL 0x0F
#define NPCX_SHM_CFG_IDX_HOST_COFS_CTL 0x0F
#define NPCX_SHM_CFG_IDX_HOST_HOFS2L 0x10
#define NPCX_SHM_CFG_IDX_HOST_HOFS2H 0x11
#define NPCX_SHM_CFG_IDX_HOST_HOFS1L 0x12
#define NPCX_SHM_CFG_IDX_HOST_HOFS1H 0x13
#define NPCX_SHM_CFG_IDX_HOST_HOFS4L 0x10
#define NPCX_SHM_CFG_IDX_HOST_HOFS4H 0x11
#define NPCX_SHM_CFG_IDX_HOST_HOFS3L 0x12
#define NPCX_SHM_CFG_IDX_HOST_HOFS3H 0x13

#define NPCX_C2H_TRANSACTION_TIMEOUT_US 200U

/* Extended SHM interrupt bits in SMCE_CTL. */
#define NPCX_SMCE_CTL_HSEM3_IE 3
#define NPCX_SMCE_CTL_HSEM4_IE 4

/* CRSMAE access bits for SHM/ESHM semaphore control space. */
#define NPCX_CRSMAE_SHM_REGS 10U
#define NPCX_CRSMAE_ESHM_REGS 3U
#define NPCX_CRSMAE_SHM_REGS_MASK BIT(NPCX_CRSMAE_SHM_REGS)
#define NPCX_CRSMAE_ESHM_REGS_MASK BIT(NPCX_CRSMAE_ESHM_REGS)

#define NPCX_SHM_CLOCK_COUNT 3

struct npcx_shm_config {
	struct shm_reg *const shm;
	struct c2h_reg *const c2h;
	struct npcx_clk_cfg clk_cfg[NPCX_SHM_CLOCK_COUNT];
};

struct npcx_shm_data {
	struct k_spinlock lock;
	uint8_t window_id;
};

npcx_shm_callback_t npcx_shm_callback[NPCX_SHM_MAX_WINDOWS];

/*
 * Check if a window ID is valid.
 *
 * Parameters:
 * - window_id: The window ID to check.
 *
 * Returns:
 * - true if the window ID is valid, false otherwise.
 */
static inline bool npcx_shm_valid_window_id(uint8_t window_id)
{
	return (window_id >= NPCX_SHM_WINDOW_1) &&
	       (window_id <= NPCX_SHM_MAX_WINDOWS);
}

/* 
 * Set or clear a specific bit in an 8-bit register.
 *
 * Parameters:
 * - reg: Pointer to the register.
 * - bit: Bit position to modify.
 * - enable: true to set the bit, false to clear it.
 */
static void npcx_shm_set_bit_u8(volatile uint8_t *reg, uint8_t bit, bool enable)
{
	if (enable) {
		*reg |= BIT(bit);
	} else {
		*reg &= (uint8_t)~BIT(bit);
	}
}

/* 
 * Convert a window size to the hardware encoding.
 *
 * Parameters:
 * - size: Window size in bytes.
 *
 * Returns:
 * - Hardware encoding of the window size.
 */
static inline uint8_t npcx_shm_covert_wnd_size(uint32_t size)
{
	/* Hardware encoding is log2(window_size) with the valid range clamped. */
	if (size <= 8U) {
		size = 8U;
	}

	if (size >= 4096U) {
		size = 4096U;
	}

	return (uint8_t)(32 - __builtin_clz(size - 1U));
}

/*
 * Wait for a C2H write transaction to complete.
 *
 * Parameters:
 * - c2h: Pointer to the C2H register structure.
 */
static void npcx_shm_wait_c2h_write_done(struct c2h_reg *c2h)
{
	uint32_t start_cycles = k_cycle_get_32();
	uint32_t max_wait_cycles = k_us_to_cyc_ceil32(NPCX_C2H_TRANSACTION_TIMEOUT_US);

	/* Poll until the C2H write transaction bit is cleared or timeout expires. */
	while ((c2h->SIBCTRL & BIT(NPCX_SIBCTRL_CSWR)) != 0U) {
		if ((k_cycle_get_32() - start_cycles) > max_wait_cycles) {
			break;
		}
	}
}

/* 
 * Wait for a C2H read transaction to complete.
 *
 * Parameters:
 * - c2h: Pointer to the C2H register structure.
 */
static void npcx_shm_wait_c2h_read_done(struct c2h_reg *c2h)
{
	uint32_t start_cycles = k_cycle_get_32();
	uint32_t max_wait_cycles = k_us_to_cyc_ceil32(NPCX_C2H_TRANSACTION_TIMEOUT_US);

	/* Poll until the C2H read transaction bit is cleared or timeout expires. */
	while ((c2h->SIBCTRL & BIT(NPCX_SIBCTRL_CSRD)) != 0U) {
		if ((k_cycle_get_32() - start_cycles) > max_wait_cycles) {
			break;
		}
	}
}

/* 
 * Read a value from a Host register for the selected LDN.
 *
 * Parameters:
 * - config: SHM configuration structure.
 * - data: SHM data structure.
 * - device_mask: Mask representing the selected LDN.
 * - offset: Register offset.
 *
 * Returns:
 * - The value read from the register.
 */
static uint8_t npcx_c2h_read_reg(const struct npcx_shm_config *config,
				 struct npcx_shm_data *data,
				 uint16_t device_mask,
				 uint8_t offset)
{
	struct c2h_reg *c2h = config->c2h;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	bool locked = (c2h->LKSIOHA & device_mask) != 0U;
	uint16_t crsmae_restore = c2h->CRSMAE;
	uint8_t value;

	/* Acquire temporary lock only when this client does not already hold it. */
	if (!locked) {
		c2h->LKSIOHA |= device_mask;
	}

	c2h->SIBCTRL |= BIT(NPCX_SIBCTRL_CSAE);
	npcx_shm_wait_c2h_read_done(c2h);
	npcx_shm_wait_c2h_write_done(c2h);
	c2h->CRSMAE = device_mask;
	c2h->IHIOA = offset;
	c2h->SIBCTRL |= BIT(NPCX_SIBCTRL_CSRD);
	npcx_shm_wait_c2h_read_done(c2h);
	value = c2h->IHD;

	c2h->CRSMAE = crsmae_restore;

	if (!locked) {
		c2h->LKSIOHA &= (uint16_t)~device_mask;
	}

	k_spin_unlock(&data->lock, key);

	return value;
}

/* 
 * Write a value to a Host register for the selected LDN.
 *
 * Parameters:
 * - config: SHM configuration structure.
 * - data: SHM data structure.
 * - device_mask: Mask representing the selected LDN.
 * - offset: Register offset.
 * - value: Value to write.
 */
static void npcx_c2h_write_reg(const struct npcx_shm_config *config,
			       struct npcx_shm_data *data,
			       uint16_t device_mask,
			       uint8_t offset,
			       uint8_t value)
{
	struct c2h_reg *c2h = config->c2h;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	bool locked = (c2h->LKSIOHA & device_mask) != 0U;
	uint16_t crsmae_restore = c2h->CRSMAE;

	/* Acquire temporary lock only when this client does not already hold it. */
	if (!locked) {
		c2h->LKSIOHA |= device_mask;
	}

	c2h->SIBCTRL |= BIT(NPCX_SIBCTRL_CSAE);
	npcx_shm_wait_c2h_read_done(c2h);
	npcx_shm_wait_c2h_write_done(c2h);
	c2h->CRSMAE = device_mask;
	c2h->IHIOA = offset;
	c2h->IHD = value;
	npcx_shm_wait_c2h_write_done(c2h);

	c2h->CRSMAE = crsmae_restore;

	if (!locked) {
		c2h->LKSIOHA &= (uint16_t)~device_mask;
	}

	k_spin_unlock(&data->lock, key);
}

/* 
 * Return the offset for the HOFSx registers for the selected SHM window.
 *
 * Parameters:
 * - window_id: SHM window identifier.
 * - idx_l: Pointer to HOFSxL register offset.
 * - idx_h: Pointer to HOFSxH register offset.
 */
static void npcx_shm_hofs_cfg_index(uint8_t window_id, uint8_t *idx_l,
				    uint8_t *idx_h)
{
	/* Each window maps to a different pair of host-offset config indexes. */
	switch (window_id) {
	case NPCX_SHM_WINDOW_1:
		*idx_l = NPCX_SHM_CFG_IDX_HOST_HOFS1L;
		*idx_h = NPCX_SHM_CFG_IDX_HOST_HOFS1H;
		break;
	case NPCX_SHM_WINDOW_2:
		*idx_l = NPCX_SHM_CFG_IDX_HOST_HOFS2L;
		*idx_h = NPCX_SHM_CFG_IDX_HOST_HOFS2H;
		break;
	case NPCX_SHM_WINDOW_3:
		*idx_l = NPCX_SHM_CFG_IDX_HOST_HOFS3L;
		*idx_h = NPCX_SHM_CFG_IDX_HOST_HOFS3H;
		break;
	default:
		*idx_l = NPCX_SHM_CFG_IDX_HOST_HOFS4L;
		*idx_h = NPCX_SHM_CFG_IDX_HOST_HOFS4H;
		break;
	}
}

#define NPCX_SHM_SEM_1_2_IE (BIT(NPCX_SMC_CTL_HSEM1_IE) | BIT(NPCX_SMC_CTL_HSEM2_IE))
#define NPCX_ESHM_SEM_3_4_IE (BIT(NPCX_SMCE_CTL_HSEM3_IE) | BIT(NPCX_SMCE_CTL_HSEM4_IE))

/* 
 * SHM interrupt service routine.
 *
 * Parameters:
 * - dev: SHM window device instance.
 */
static void npcx_shm_isr(const struct device *dev)
{
	const struct npcx_shm_config *config = dev->config;
	uint8_t smc_ctl   = config->shm->SMC_CTL;
	uint8_t smc_sts   = config->shm->SMC_STS;
	uint8_t hofs_ctl  = config->shm->HOFS_CTL;
	uint8_t hofs_sts  = config->shm->HOFS_STS;
	uint8_t smce_ctl  = config->shm->SMCE_CTL;
	uint8_t smce_sts  = config->shm->SMCE_STS;
	uint8_t hofse_ctl = config->shm->HOFSE_CTL;
	uint8_t hofse_sts = config->shm->HOFSE_STS;

	/* clear status bits  */
	config->shm->SMC_STS   = smc_sts;
	config->shm->SMCE_STS  = smce_sts;
	config->shm->HOFS_STS  = hofs_sts;
	config->shm->HOFSE_STS = hofse_sts;

	/* clear none enable bits  */
	smc_sts   = smc_sts   & ((smc_ctl & NPCX_SHM_SEM_1_2_IE) << 1);
	smce_sts  = smce_sts  & ((smce_ctl & NPCX_ESHM_SEM_3_4_IE) << 1);
	hofs_sts  = hofs_sts  & hofs_ctl;
	hofse_sts = hofse_sts & hofse_ctl;

	if (npcx_shm_callback[0] != NULL) {
		/* check WIN1 bits */
		uint8_t sts = (hofs_sts & 0x3) | ((smc_sts & 0x10) >> 2);
		if(sts) {
			npcx_shm_callback[0](sts);
		}
	}
	if (npcx_shm_callback[1] != NULL) {
		/* check WIN2 bits */
		uint8_t sts = ((hofs_sts >> 2) & 0x3) | ((smc_sts & 0x20) >> 3);
		if(sts) {
			npcx_shm_callback[1](sts);
		}
	}
	if (npcx_shm_callback[2] != NULL) {
		/* check WIN3 bits */
		uint8_t sts = (hofse_sts & 0x3) | ((smce_sts & 0x10) >> 2);
		if(sts) {
			npcx_shm_callback[2](sts);
		}
	}
	if (npcx_shm_callback[3] != NULL) {
		/* check WIN4 bits */
		uint8_t sts = ((hofse_sts >> 2) & 0x3) | ((smce_sts & 0x20) >> 3);
		if(sts) {
			npcx_shm_callback[3](sts);
		}
	}
}

/* 
 * Enable or disable semaphore interrupts for the selected SHM window.
 *
 * Parameters:
 * - dev: SHM window device instance.
 * - enable: true to enable interrupts, false to disable.
 */
static void npcx_shm_enable_semaphore_interrupt(const struct device *dev,
				    bool enable)
{
	const struct npcx_shm_config *config = dev->config;
	const struct npcx_shm_data *data = dev->data;
	uint8_t window_id = data->window_id;

	if (window_id == NPCX_SHM_WINDOW_1) {
		npcx_shm_set_bit_u8(&config->shm->SMC_CTL, NPCX_SMC_CTL_HSEM1_IE,
				 enable);
	} else if (window_id == NPCX_SHM_WINDOW_2) {
		npcx_shm_set_bit_u8(&config->shm->SMC_CTL, NPCX_SMC_CTL_HSEM2_IE,
				 enable);
	} else if (window_id == NPCX_SHM_WINDOW_3) {
		npcx_shm_set_bit_u8(&config->shm->SMCE_CTL, NPCX_SMCE_CTL_HSEM3_IE,
				 enable);
	} else {
		npcx_shm_set_bit_u8(&config->shm->SMCE_CTL, NPCX_SMCE_CTL_HSEM4_IE,
				 enable);
	}
}

/* 
 * Configure the host semaphore for the selected SHM window.
 *
 * Parameters:
 * - dev: SHM window device instance.
 * - enable_irq: Enable or disable IRQ generation.
 * - enable_smi: Enable or disable SMI generation.
 */
static void npcx_shm_host_semaphore_config(const struct device *dev,
				       bool enable_irq,
				       bool enable_smi)
{
	const struct npcx_shm_config *config = dev->config;
	struct npcx_shm_data *data = dev->data;
	uint8_t sem_cts;
	uint8_t ie_bit;
	uint8_t se_bit;
	uint8_t reg_idx;
	uint16_t device_mask;

	switch (data->window_id) {
	case NPCX_SHM_WINDOW_1:
		reg_idx = NPCX_SHM_CFG_IDX_SEM_CTS;
		device_mask = NPCX_CRSMAE_SHM_REGS_MASK;
		ie_bit = 2U;
		se_bit = 4U;
		break;
	case NPCX_SHM_WINDOW_2:
		reg_idx = NPCX_SHM_CFG_IDX_SEM_CTS;
		device_mask = NPCX_CRSMAE_SHM_REGS_MASK;
		ie_bit = 3U;
		se_bit = 5U;
		break;
	case NPCX_SHM_WINDOW_3:
		reg_idx = NPCX_SHM_CFG_IDX_SEME_CTS;
		device_mask = NPCX_CRSMAE_ESHM_REGS_MASK;
		ie_bit = 2U;
		se_bit = 4U;
		break;
	case NPCX_SHM_WINDOW_4:
		reg_idx = NPCX_SHM_CFG_IDX_SEME_CTS;
		device_mask = NPCX_CRSMAE_ESHM_REGS_MASK;
		ie_bit = 3U;
		se_bit = 5U;
		break;
	default:
		return;
	}

	sem_cts = npcx_c2h_read_reg(config, data, device_mask, reg_idx);

	npcx_shm_set_bit_u8(&sem_cts, ie_bit, enable_irq);
	npcx_shm_set_bit_u8(&sem_cts, se_bit, enable_smi);

	npcx_c2h_write_reg(config, data, device_mask, reg_idx, sem_cts);
}

/*
 * Configure the selected SHM window base address and size.
 *
 * Parameters:
 * - dev: SHM window device instance.
 * - window_base: Window base address in RAM.
 * - window_size: Requested window size in bytes.
 */
static void npcx_shm_set_window(const struct device *dev,
		       uint32_t window_base,
		       uint16_t window_size)
{
	const struct npcx_shm_config *config = dev->config;
	const struct npcx_shm_data *data = dev->data;
	struct shm_reg *shm = config->shm;
	uint8_t size = npcx_shm_covert_wnd_size(window_size);

	switch (data->window_id) {
	case NPCX_SHM_WINDOW_1:
		shm->WIN_BASE1 = window_base;
		SET_FIELD(shm->WIN_SIZE, NPCX_WIN_SIZE_RWIN1_SIZE_FIELD, size);
		break;
	case NPCX_SHM_WINDOW_2:
		shm->WIN_BASE2 = window_base;
		SET_FIELD(shm->WIN_SIZE, NPCX_WIN_SIZE_RWIN2_SIZE_FIELD, size);
		break;
	case NPCX_SHM_WINDOW_3:
		shm->WIN_BASE3 = window_base;
		SET_FIELD(shm->WINE_SIZE, NPCX_WIN_SIZE_RWIN3_SIZE_FIELD, size);
		break;
	default:
		shm->WIN_BASE4 = window_base;
		SET_FIELD(shm->WINE_SIZE, NPCX_WIN_SIZE_RWIN4_SIZE_FIELD, size);
		break;
	}
}

/*
 * Configure read/write protection bits for the selected SHM window.
 *
 * Parameters:
 * - dev: SHM window device instance.
 * - window_prot: Packed protection value; low byte is write protection and
 *   high byte is read protection.
 */
static void npcx_shm_window_protect(const struct device *dev,
				uint16_t window_prot)
{
	const struct npcx_shm_config *config = dev->config;
	const struct npcx_shm_data *data = dev->data;

	switch (data->window_id) {
	case NPCX_SHM_WINDOW_1:
		config->shm->WIN1_WR_PROT = (uint8_t)(window_prot & 0xFFU);
		config->shm->WIN1_RD_PROT = (uint8_t)(window_prot >> 8);
		break;
	case NPCX_SHM_WINDOW_2:
		config->shm->WIN2_WR_PROT = (uint8_t)(window_prot & 0xFFU);
		config->shm->WIN2_RD_PROT = (uint8_t)(window_prot >> 8);
		break;
	case NPCX_SHM_WINDOW_3:
		config->shm->WIN3_WR_PROT = (uint8_t)(window_prot & 0xFFU);
		config->shm->WIN3_RD_PROT = (uint8_t)(window_prot >> 8);
		break;
	default:
		config->shm->WIN4_WR_PROT = (uint8_t)(window_prot & 0xFFU);
		config->shm->WIN4_RD_PROT = (uint8_t)(window_prot >> 8);
		break;
	}
}

/* 
 * Set the "Core offset" from the windows base.
 * A core WRITE to this offset can generate an IRQ/SMI to the host
 * A host READ from this offset can generate an Interrupt to the core
 */
static void npcx_shm_set_core_offset(const struct device *dev,
				 uint16_t offset)
{
	const struct npcx_shm_config *config = dev->config;
	const struct npcx_shm_data *data = dev->data;

	switch (data->window_id) {
	case NPCX_SHM_WINDOW_1:
		config->shm->COFS1 = offset;
		break;
	case NPCX_SHM_WINDOW_2:
		config->shm->COFS2 = offset;
		break;
	case NPCX_SHM_WINDOW_3:
		config->shm->COFS3 = offset;
		break;
	default:
		config->shm->COFS4 = offset;
		break;
	}
}
/* 
 * Enable core offset interrupts for the selected SHM window.
 */
static void npcx_shm_enable_core_offset_interrupts(const struct device *dev,
				       uint8_t mask)
{
	const struct npcx_shm_config *config = dev->config;
	const struct npcx_shm_data *data = dev->data;
	uint8_t bit;

	/* WIN1/2 use HOFS_CTL, WIN3/4 use HOFSE_CTL. */
	if ((mask & NPCX_SHM_INT_ON_HOST_RD_CORE_OFFSET) != 0U) {
		if (data->window_id <= NPCX_SHM_WINDOW_2) {
			bit = (uint8_t)(data->window_id - 1U) * 2U;
			npcx_shm_set_bit_u8(&config->shm->HOFS_CTL, bit, true);
		} else {
			bit = (uint8_t)(data->window_id - 3U) * 2U;
			npcx_shm_set_bit_u8(&config->shm->HOFSE_CTL, bit, true);
		}
	}

	if ((mask & NPCX_SHM_INT_ON_HOST_WR_HOST_OFFSET) != 0U) {
		if (data->window_id <= NPCX_SHM_WINDOW_2) {
			bit = 1U + ((uint8_t)(data->window_id - 1U) * 2U);
			npcx_shm_set_bit_u8(&config->shm->HOFS_CTL, bit, true);
		} else {
			bit = 1U + ((uint8_t)(data->window_id - 3U) * 2U);
			npcx_shm_set_bit_u8(&config->shm->HOFSE_CTL, bit, true);
		}
	}
}

/* 
 * Uses the Core2Host (c2h) interface to access the Host registers in order to set the "Host offset"
 * A host WRITE to this offset can generate an Interrupt to the core
 * A core READ from this offset can generate an IRQ/SMI to the host
 */
static void npcx_shm_set_host_offset(const struct device *dev,
				 uint16_t offset)
{
	const struct npcx_shm_config *config = dev->config;
	struct npcx_shm_data *data = dev->data;
	uint8_t idx_l;
	uint8_t idx_h;
	uint16_t device_mask;

	npcx_shm_hofs_cfg_index(data->window_id, &idx_l, &idx_h);

	if (data->window_id <= NPCX_SHM_WINDOW_2) {
		device_mask = NPCX_CRSMAE_SHM_REGS_MASK;
	} else {
		device_mask = NPCX_CRSMAE_ESHM_REGS_MASK;
	}

	npcx_c2h_write_reg(config, data, device_mask, idx_l, (uint8_t)(offset & 0xFFU));
	npcx_c2h_write_reg(config, data, device_mask, idx_h, (uint8_t)(offset >> 8));
}

/* 
 * Enable host offset interrupts for the selected SHM window. 
 */
static void npcx_shm_enable_host_offset_interrupts(const struct device *dev,
					 uint8_t mask)
{
	const struct npcx_shm_config *config = dev->config;
	struct npcx_shm_data *data = dev->data;
	uint8_t cofs_ctl;
	uint16_t device_mask;
	uint8_t shift = (data->window_id == NPCX_SHM_WINDOW_2 ||
			 data->window_id == NPCX_SHM_WINDOW_4) ? 2U : 0U;
	uint8_t win_mask = NPCX_SHM_IRQ_CORE_RD_FROM_HOST_OFFSET |
			   NPCX_SHM_IRQ_CORE_WR_TO_CORE_OFFSET |
			   NPCX_SHM_SMI_CORE_RD_FROM_HOST_OFFSET |
			   NPCX_SHM_SMI_CORE_WR_TO_CORE_OFFSET;
	uint8_t reg_idx;

	if (data->window_id <= NPCX_SHM_WINDOW_2) {
		device_mask = NPCX_CRSMAE_SHM_REGS_MASK;
		reg_idx = NPCX_SHM_CFG_IDX_HOST_COFS_CTL;
	} else {
		device_mask = NPCX_CRSMAE_ESHM_REGS_MASK;
		reg_idx = NPCX_SHM_CFG_IDX_HOST_COFSE_CTL;
	}

	cofs_ctl = npcx_c2h_read_reg(config, data, device_mask, reg_idx);
	cofs_ctl &= (uint8_t)~(win_mask << shift);
	cofs_ctl |= (uint8_t)((mask & win_mask) << shift);

	npcx_c2h_write_reg(config, data, device_mask, reg_idx, cofs_ctl);
}

/*
 * Read the current host semaphore value for the selected SHM window.
 *
 * Returns:
 * - The semaphore value.
 */
static uint8_t npcx_shm_api_semaphore_read(const struct device *dev)
{
	const struct npcx_shm_config *config = dev->config;
	const struct npcx_shm_data *data = dev->data;

	switch (data->window_id) {
	case NPCX_SHM_WINDOW_1:
		return config->shm->SHAW1_SEM;
	case NPCX_SHM_WINDOW_2:
		return config->shm->SHAW2_SEM;
	case NPCX_SHM_WINDOW_3:
		return config->shm->SHAW3_SEM;
	default:
		return config->shm->SHAW4_SEM;
	}
}

/*
 * Write value to the selected SHM window semaphore register.
 *
 * Parameters:
 * - dev: SHM window device instance.
 * - value: The value to be written to the Semaphore register.
 */
static void npcx_shm_api_semaphore_write(const struct device *dev,
				    uint8_t value)
{
	const struct npcx_shm_config *config = dev->config;
	const struct npcx_shm_data *data = dev->data;

	switch (data->window_id) {
	case NPCX_SHM_WINDOW_1:
		config->shm->SHAW1_SEM = value;
		break;
	case NPCX_SHM_WINDOW_2:
		config->shm->SHAW2_SEM = value;
		break;
	case NPCX_SHM_WINDOW_3:
		config->shm->SHAW3_SEM = value;
		break;
	default:
		config->shm->SHAW4_SEM = value;
		break;
	}
}

/*
 * SHM window configuration API.
 *
 * Applies callback registration, semaphore behavior, window base/size,
 * protection, core/host offsets, and interrupt masks from one config object.
 */
static void npcx_shm_api_window_config(const struct device *dev,
				       const struct npcx_shm_window_config *cfg)
{
	if (cfg->callback != NULL) {
		const struct npcx_shm_data *data = dev->data;
		npcx_shm_callback[data->window_id-1] = cfg->callback;
	}

	/* Configure semaphore core interrupt */
	npcx_shm_enable_semaphore_interrupt(dev,
		(cfg->core_interrupts_mask & NPCX_SHM_INT_ON_HOST_WR_SEMAPHORE) != 0U);

	/* Configure semaphore host IRQ/SMI */
	npcx_shm_host_semaphore_config(dev,
		(cfg->host_interrupts_mask & NPCX_SHM_IRQ_CORE_WR_TO_SEMAPHOR) != 0U,
		(cfg->host_interrupts_mask & NPCX_SHM_SMI_CORE_WR_TO_SEMAPHOR) != 0U);

	/* Configure window */
	npcx_shm_set_window(dev, cfg->window_base, cfg->window_size);

	/* Configure window protection */
	npcx_shm_window_protect(dev, cfg->window_protection_mask);

	/* Set core offset */
	npcx_shm_set_core_offset(dev, cfg->core_offset);

	/* Configure core offset interrupts from the mask. */
	npcx_shm_enable_core_offset_interrupts(dev, cfg->core_interrupts_mask);

	/* Set host offset */
	npcx_shm_set_host_offset(dev, cfg->host_offset);

	/* Enable host offset interrupts */
	npcx_shm_enable_host_offset_interrupts(dev, cfg->host_interrupts_mask);
}

/*
 * Unified SHM interrupt control API.
 *
 * Enables or disables one interrupt source selected by type.
 */
static void npcx_shm_api_interrupt_control(const struct device *dev,
					   enum npcx_shm_interrupt_type type,
					   bool enable)
{
	switch (type) {
	case NPCX_SHM_INT_SEMAPHORE:
		npcx_shm_enable_semaphore_interrupt(dev, enable);
		break;
	case NPCX_SHM_INT_CORE_OFFSET_ON_HOST_RD:
		npcx_shm_enable_core_offset_interrupts(dev,
			enable ? NPCX_SHM_INT_ON_HOST_RD_CORE_OFFSET : 0U);
		break;
	case NPCX_SHM_INT_CORE_OFFSET_ON_HOST_WR:
		npcx_shm_enable_core_offset_interrupts(dev,
			enable ? NPCX_SHM_INT_ON_HOST_WR_HOST_OFFSET : 0U);
		break;
	case NPCX_SHM_INT_HOST_OFFSET_IRQ_RD:
		if (enable) {
			npcx_shm_enable_host_offset_interrupts(dev,
				NPCX_SHM_IRQ_CORE_RD_FROM_HOST_OFFSET);
		} else {
			npcx_shm_enable_host_offset_interrupts(dev, 0U);
		}
		break;
	case NPCX_SHM_INT_HOST_OFFSET_IRQ_WR:
		if (enable) {
			npcx_shm_enable_host_offset_interrupts(dev,
				NPCX_SHM_IRQ_CORE_WR_TO_CORE_OFFSET);
		} else {
			npcx_shm_enable_host_offset_interrupts(dev, 0U);
		}
		break;
	case NPCX_SHM_INT_HOST_OFFSET_SMI_RD:
		if (enable) {
			npcx_shm_enable_host_offset_interrupts(dev,
				NPCX_SHM_SMI_CORE_RD_FROM_HOST_OFFSET);
		} else {
			npcx_shm_enable_host_offset_interrupts(dev, 0U);
		}
		break;
	case NPCX_SHM_INT_HOST_OFFSET_SMI_WR:
		if (enable) {
			npcx_shm_enable_host_offset_interrupts(dev,
				NPCX_SHM_SMI_CORE_WR_TO_CORE_OFFSET);
		} else {
			npcx_shm_enable_host_offset_interrupts(dev, 0U);
		}
		break;
	default:
		break;
	}
}

static DEVICE_API(npcx_shm, npcx_shm_api) = {
	.window_config = npcx_shm_api_window_config,
	.interrupt_control = npcx_shm_api_interrupt_control,
	.semaphore_read = npcx_shm_api_semaphore_read,
	.semaphore_write = npcx_shm_api_semaphore_write,
};

#define NPCX_SHM_PARENT(inst) DT_PARENT(DT_DRV_INST(inst))

#define NPCX_SHM_INIT(inst) \
	static const struct npcx_shm_config npcx_shm_cfg_##inst = { \
		.shm = (struct shm_reg *)DT_REG_ADDR_BY_NAME(NPCX_SHM_PARENT(inst), shm), \
		.c2h = (struct c2h_reg *)DT_REG_ADDR_BY_NAME(NPCX_SHM_PARENT(inst), c2h), \
		.clk_cfg = { \
			{ \
				.bus = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 0, bus), \
				.ctrl = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 0, ctl), \
				.bit = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 0, bit), \
			}, \
			{ \
				.bus = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 1, bus), \
				.ctrl = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 1, ctl), \
				.bit = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 1, bit), \
			}, \
			{ \
				.bus = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 2, bus), \
				.ctrl = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 2, ctl), \
				.bit = DT_CLOCKS_CELL_BY_IDX(NPCX_SHM_PARENT(inst), 2, bit), \
			}, \
		}, \
	}; \
	static struct npcx_shm_data npcx_shm_data_##inst = { \
		.window_id = DT_INST_PROP(inst, id), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, \
			      &npcx_shm_data_##inst, &npcx_shm_cfg_##inst, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
			      &npcx_shm_api)

DT_INST_FOREACH_STATUS_OKAY(NPCX_SHM_INIT)

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0
/* Register all enabled window devices exactly once at boot. */
#define NPCX_SHM_WINDOW_INIT(inst) \
	do { \
		const struct device *window_dev = DEVICE_DT_GET(DT_DRV_INST(inst)); \
		struct npcx_shm_data *window_data = window_dev->data; \
		if (!npcx_shm_valid_window_id(window_data->window_id)) { \
			return -EINVAL; \
		} \
		npcx_shm_callback[window_data->window_id-1] = NULL; \
	} while (0);

/* Initialize the SHM module */
static int npcx_shm_module_init(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	const struct npcx_shm_config *config;
	const struct device *clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	int ret;
	int i;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	if (!device_is_ready(clk_dev)) {
		return -ENODEV;
	}

	config = dev->config;
	for (i = 0; i < NPCX_SHM_CLOCK_COUNT; i++) {
		ret = clock_control_on(clk_dev, (clock_control_subsys_t)&config->clk_cfg[i]);
		if (ret < 0) {
			return ret;
		}
	}

	DT_INST_FOREACH_STATUS_OKAY(NPCX_SHM_WINDOW_INIT)

	/* clear all interrupts */
	config->shm->SMC_STS = 0xFFU;
	config->shm->SMCE_STS = 0xFFU;
	config->shm->HOFS_STS = 0xFFU;
	config->shm->HOFSE_STS = 0xFFU;

	/* install the Interrupt hadler */
	IRQ_CONNECT(DT_IRQN(NPCX_SHM_PARENT(0)), DT_IRQ(NPCX_SHM_PARENT(0), priority),
		    npcx_shm_isr, DEVICE_DT_GET(DT_DRV_INST(0)), 0);
	irq_enable(DT_IRQN(NPCX_SHM_PARENT(0)));

	return 0;
}

SYS_INIT(npcx_shm_module_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif
