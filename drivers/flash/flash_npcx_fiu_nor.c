/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcx_fiu_nor

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/npcx_flash_api_ex.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/flash_controller/npcx_fiu_qspi.h>
#include <soc.h>
#ifdef CONFIG_USERSPACE
#include <zephyr/syscall.h>
#include <zephyr/internal/syscall_handler.h>
#endif

#include "flash_npcx_fiu_qspi.h"
#include "spi_nor.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_npcx_fiu_nor, CONFIG_FLASH_LOG_LEVEL);

#define BLOCK_64K_SIZE KB(64)
#define BLOCK_4K_SIZE  KB(4)

#define NUM_OF_PARAMETER_HEADER 	1
#define NUM_OF_BFP_ENTRIES 			16

/* Device config */
struct flash_npcx_nor_config {
	/* QSPI bus device for mutex control and bus configuration */
	const struct device *qspi_bus;
	/* Mapped address for flash read via direct access */
	uintptr_t mapped_addr;
	/* Maximum chip erase time-out in ms */
	uint32_t max_timeout;
};

/* Device data */
struct flash_npcx_nor_data {
	/* Specific control operation for Quad-SPI Nor Flash */
	uint32_t operation;
	/* Size of nor device in bytes, from size property */
	uint32_t flash_size;
	/* SPI Nor device configuration on QSPI bus */
	struct npcx_qspi_cfg qspi_cfg;

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
};

static const struct flash_parameters flash_npcx_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

#define DT_INST_QUAD_EN_PROP_OR(inst)						\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, quad_enable_requirements),	\
		    (_CONCAT(JESD216_DW15_QER_VAL_,				\
		     DT_INST_STRING_TOKEN(inst, quad_enable_requirements))),	\
		    ((JESD216_DW15_QER_VAL_NONE)))

static inline bool is_within_region(off_t addr, size_t size, off_t region_start,
				    size_t region_size)
{
	return (addr >= region_start &&
		(addr < (region_start + region_size)) &&
		((addr + size) <= (region_start + region_size)));
}

static int flash_npcx_uma_transceive(const struct device *dev, struct npcx_uma_cfg *cfg,
				     uint32_t flags)
{
	const struct flash_npcx_nor_config *config = dev->config;
	struct flash_npcx_nor_data *data = dev->data;
	int ret;

	/* Lock SPI bus and configure it if needed */
	qspi_npcx_fiu_mutex_lock_configure(config->qspi_bus, &data->qspi_cfg,
					   data->operation);

	/* Execute UMA transaction */
	ret = qspi_npcx_fiu_uma_transceive(config->qspi_bus, cfg, flags);

	/* Unlock SPI bus */
	qspi_npcx_fiu_mutex_unlock(config->qspi_bus);

	return ret;
}

/* NPCX UMA functions for SPI NOR flash */
static int flash_npcx_uma_cmd_only(const struct device *dev, uint8_t opcode)
{
	struct npcx_uma_cfg cfg = { .opcode = opcode};

	return flash_npcx_uma_transceive(dev, &cfg, 0); /* opcode only */
}

static int flash_npcx_uma_cmd_by_addr(const struct device *dev, uint8_t opcode,
					     uint32_t addr)
{
	struct npcx_uma_cfg cfg = { .opcode = opcode};

	cfg.addr.u32 = sys_cpu_to_be32(addr);
	return flash_npcx_uma_transceive(dev, &cfg, NPCX_UMA_ACCESS_ADDR);
}

static int flash_npcx_uma_read(const struct device *dev, uint8_t opcode,
			       uint8_t *dst, const size_t size)
{
	struct npcx_uma_cfg cfg = { .opcode = opcode,
				    .rx_buf = dst,
				    .rx_count = size};

	return flash_npcx_uma_transceive(dev, &cfg, NPCX_UMA_ACCESS_READ);
}

static int flash_npcx_uma_write(const struct device *dev, uint8_t opcode,
				       uint8_t *src, const size_t size)
{
	struct npcx_uma_cfg cfg = { .opcode = opcode,
				    .tx_buf = src,
				    .tx_count = size};

	return flash_npcx_uma_transceive(dev, &cfg, NPCX_UMA_ACCESS_WRITE);
}

static int flash_npcx_uma_write_by_addr(const struct device *dev, uint8_t opcode,
					       uint8_t *src, const size_t size, uint32_t addr)
{
	struct npcx_uma_cfg cfg = { .opcode = opcode,
					.tx_buf = src,
					.tx_count = size};

	cfg.addr.u32 = sys_cpu_to_be32(addr);
	return flash_npcx_uma_transceive(dev, &cfg, NPCX_UMA_ACCESS_WRITE |
					 NPCX_UMA_ACCESS_ADDR);
}

/* Local SPI NOR flash functions */
static int flash_npcx_nor_wait_until_ready(const struct device *dev)
{
	int ret;
	uint8_t reg;
	const struct flash_npcx_nor_config *config = dev->config;
	int64_t st = k_uptime_get();

	do {
		ret = flash_npcx_uma_read(dev, SPI_NOR_CMD_RDSR, &reg, sizeof(reg));
		if (ret != 0) {
			return ret;
		} else if ((reg & SPI_NOR_WIP_BIT) == 0) {
			return 0;
		}

	} while ((k_uptime_get() - st) < config->max_timeout);

	return -EBUSY;
}

static int flash_npcx_nor_read_status_regs(const struct device *dev, uint8_t *sts_reg, uint8_t num_regs)
{
	int ret;

	if (sts_reg == NULL || num_regs == 0) {
		return -EINVAL;
	}

	if (num_regs >= 1) {
		ret = flash_npcx_uma_read(dev, SPI_NOR_CMD_RDSR, sts_reg, 1);
		if (ret != 0) {
			return ret;
		}
	}

	if (num_regs >= 2) {
		ret = flash_npcx_uma_read(dev, SPI_NOR_CMD_RDSR2, sts_reg + 1, 1);
		if (ret != 0) {
			return ret;
		}
	}

	if (num_regs >= 3) {
		ret = flash_npcx_uma_read(dev, SPI_NOR_CMD_RDSR3, sts_reg + 2, 1);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

static int flash_npcx_nor_write_status_regs(const struct device *dev, uint8_t *sts_reg, uint8_t num_regs)
{
	int ret;

	if (sts_reg == NULL || num_regs == 0) {
		return -EINVAL;
	}

	ret = flash_npcx_uma_cmd_only(dev, SPI_NOR_CMD_WREN);
	if (ret != 0) {
		return ret;
	}

	ret = flash_npcx_uma_write(dev, SPI_NOR_CMD_WRSR, sts_reg, num_regs);
	if (ret != 0) {
		return ret;
	}

	return flash_npcx_nor_wait_until_ready(dev);
}

static int flash_npcx_nor_write_status_by_cmd(const struct device *dev, uint8_t cmd_code, uint8_t *sts_reg, uint8_t num_regs)
{
	int ret;

	if (sts_reg == NULL || num_regs == 0) {
		return -EINVAL;
	}

	ret = flash_npcx_uma_cmd_only(dev, SPI_NOR_CMD_WREN);
	if (ret != 0) {
		return ret;
	}

	ret = flash_npcx_uma_write(dev, cmd_code, sts_reg, num_regs);
	if (ret != 0) {
		return ret;
	}

	return flash_npcx_nor_wait_until_ready(dev);
}

/* Flash API functions */
#if defined(CONFIG_FLASH_JESD216_API)
static int flash_npcx_nor_read_jedec_id(const struct device *dev, uint8_t *id)
{
	if (id == NULL) {
		return -EINVAL;
	}

	return flash_npcx_uma_read(dev, SPI_NOR_CMD_RDID, id, SPI_NOR_MAX_ID_LEN);
}

static int flash_npcx_nor_read_sfdp(const struct device *dev, off_t addr,
				    void *data, size_t size)
{
	uint8_t sfdp_addr[4];
	struct npcx_uma_cfg cfg = { .opcode = JESD216_CMD_READ_SFDP,
					.tx_buf = sfdp_addr,
					.tx_count = 4,
					.rx_buf = data,
					.rx_count = size};

	if (data == NULL) {
		return -EINVAL;
	}

	/* CMD_READ_SFDP needs a 24-bit address followed by a dummy byte */
	sfdp_addr[0] = (addr >> 16) & 0xff;
	sfdp_addr[1] = (addr >> 8) & 0xff;
	sfdp_addr[2] = addr & 0xff;
	return flash_npcx_uma_transceive(dev, &cfg, NPCX_UMA_ACCESS_WRITE |
					 NPCX_UMA_ACCESS_READ);
}
#endif /* CONFIG_FLASH_JESD216_API */

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_npcx_nor_pages_layout(const struct device *dev,
					const struct flash_pages_layout **layout,
					size_t *layout_size)
{
	struct flash_npcx_nor_data *dev_data = dev->data;

	*layout = &dev_data->layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int flash_npcx_nor_read(const struct device *dev, off_t addr,
				 void *data, size_t size)
{
	const struct flash_npcx_nor_config *config = dev->config;
	struct flash_npcx_nor_data *dev_data = dev->data;

	/* Out of the region of nor flash device? */
	if (!is_within_region(addr, size, 0, dev_data->flash_size)) {
		return -EINVAL;
	}

	/* Lock/Unlock SPI bus also for DRA mode */
	qspi_npcx_fiu_mutex_lock_configure(config->qspi_bus, &dev_data->qspi_cfg,
					   dev_data->operation);

	/* Trigger Direct Read Access (DRA) via reading memory mapped-address */
	memcpy(data, (void *)(config->mapped_addr + addr), size);

	qspi_npcx_fiu_mutex_unlock(config->qspi_bus);

	return 0;
}

static int flash_npcx_nor_erase(const struct device *dev, off_t addr, size_t size)
{
	struct flash_npcx_nor_data *dev_data = dev->data;
	int ret = 0;

	/* Out of the region of nor flash device? */
	if (!is_within_region(addr, size, 0, dev_data->flash_size)) {
		LOG_ERR("Addr %ld, size %d are out of range", addr, size);
		return -EINVAL;
	}

	/* address must be sector-aligned */
	if (!SPI_NOR_IS_SECTOR_ALIGNED(addr)) {
		LOG_ERR("Addr %ld is not sector-aligned", addr);
		return -EINVAL;
	}

	/* size must be a multiple of sectors */
	if ((size % BLOCK_4K_SIZE) != 0) {
		LOG_ERR("Size %d is not a multiple of sectors", size);
		return -EINVAL;
	}

	/* Select erase opcode by size */
	if (size == dev_data->flash_size) {
		flash_npcx_uma_cmd_only(dev, SPI_NOR_CMD_WREN);
		/* Send chip erase command */
		flash_npcx_uma_cmd_only(dev, SPI_NOR_CMD_CE);
		return flash_npcx_nor_wait_until_ready(dev);
	}

	while (size > 0) {
		flash_npcx_uma_cmd_only(dev, SPI_NOR_CMD_WREN);
		/* Send page/block erase command with addr */
		if ((size >= BLOCK_64K_SIZE) && SPI_NOR_IS_64K_ALIGNED(addr)) {
			flash_npcx_uma_cmd_by_addr(dev, SPI_NOR_CMD_BE, addr);
			addr += BLOCK_64K_SIZE;
			size -= BLOCK_64K_SIZE;
		} else {
			flash_npcx_uma_cmd_by_addr(dev, SPI_NOR_CMD_SE, addr);
			addr += BLOCK_4K_SIZE;
			size -= BLOCK_4K_SIZE;
		}
		ret = flash_npcx_nor_wait_until_ready(dev);
		if (ret != 0) {
			break;
		}
	}

	return ret;
}

static int flash_npcx_nor_write(const struct device *dev, off_t addr,
				  const void *data, size_t size)
{
	struct flash_npcx_nor_data *dev_data = dev->data;
	uint8_t *tx_buf = (uint8_t *)data;
	int ret = 0;
	size_t sz_write;

	/* Out of the region of nor flash device? */
	if (!is_within_region(addr, size, 0, dev_data->flash_size)) {
		return -EINVAL;
	}

	/* Don't write more than a page. */
	if (size > SPI_NOR_PAGE_SIZE) {
		sz_write = SPI_NOR_PAGE_SIZE;
	} else {
		sz_write = size;
	}

	/*
	 * Correct the size of first write to not go through page boundary and
	 * make the address of next write to align to page boundary.
	 */
	if (((addr + sz_write - 1U) / SPI_NOR_PAGE_SIZE) != (addr / SPI_NOR_PAGE_SIZE)) {
		sz_write -= (addr + sz_write) & (SPI_NOR_PAGE_SIZE - 1);
	}

	while (size > 0) {
		/* Start to write */
		flash_npcx_uma_cmd_only(dev, SPI_NOR_CMD_WREN);
		ret = flash_npcx_uma_write_by_addr(dev, SPI_NOR_CMD_PP, tx_buf,
						   sz_write, addr);
		if (ret != 0) {
			break;
		}

		/* Wait for writing completed */
		ret = flash_npcx_nor_wait_until_ready(dev);
		if (ret != 0) {
			break;
		}

		size -= sz_write;
		tx_buf += sz_write;
		addr += sz_write;

		if (size > SPI_NOR_PAGE_SIZE) {
			sz_write = SPI_NOR_PAGE_SIZE;
		} else {
			sz_write = size;
		}
	}

	return ret;
}

static const struct flash_parameters *
flash_npcx_nor_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_npcx_parameters;
};

#ifdef CONFIG_FLASH_EX_OP_ENABLED
static int flash_npcx_nor_ex_exec_uma(const struct device *dev,
				      const struct npcx_ex_ops_uma_in *op_in,
				      const struct npcx_ex_ops_uma_out *op_out)
{
	int flag = 0;
	struct npcx_uma_cfg cfg;

	if (op_in == NULL) {
		return -EINVAL;
	}

	/* Organize a UMA transaction */
	cfg.opcode = op_in->opcode;
	if (op_in->tx_count != 0) {
		cfg.tx_buf = op_in->tx_buf;
		cfg.tx_count = op_in->tx_count;
		flag |= NPCX_UMA_ACCESS_WRITE;
	}

	if (op_in->addr_count != 0) {
		cfg.addr.u32 = sys_cpu_to_be32(op_in->addr);
		flag |= NPCX_UMA_ACCESS_ADDR;
	}

	if (op_out != NULL && op_in->rx_count != 0) {
		cfg.rx_buf = op_out->rx_buf;
		cfg.rx_count = op_in->rx_count;
		flag |= NPCX_UMA_ACCESS_READ;
	}

	return flash_npcx_uma_transceive(dev, &cfg, flag);
}

static int flash_npcx_nor_ex_set_spi_spec(const struct device *dev,
					  const struct npcx_ex_ops_qspi_oper_in *op_in)
{
	struct flash_npcx_nor_data *data = dev->data;

	/* Cannot disable write protection of internal flash */
	if ((data->operation & NPCX_EX_OP_INT_FLASH_WP) != 0) {
		if ((op_in->mask & NPCX_EX_OP_INT_FLASH_WP) != 0 && !op_in->enable) {
			return -EINVAL;
		}
	}

	if (op_in->enable) {
		data->operation |= op_in->mask;
	} else {
		data->operation &= ~op_in->mask;
	}

	return 0;
}

static int flash_npcx_nor_ex_get_spi_spec(const struct device *dev,
					  struct npcx_ex_ops_qspi_oper_out *op_out)
{
	struct flash_npcx_nor_data *data = dev->data;

	op_out->oper = data->operation;
	return 0;
}

static int flash_npcx_nor_ex_op(const struct device *dev, uint16_t code,
				const uintptr_t in, void *out)
{
#ifdef CONFIG_USERSPACE
	bool syscall_trap = z_syscall_trap();
#endif
	int ret;

	switch (code) {
	case FLASH_NPCX_EX_OP_EXEC_UMA:
	{
		struct npcx_ex_ops_uma_in *op_in = (struct npcx_ex_ops_uma_in *)in;
		struct npcx_ex_ops_uma_out *op_out = (struct npcx_ex_ops_uma_out *)out;
#ifdef CONFIG_USERSPACE
		struct npcx_ex_ops_uma_in in_copy;
		struct npcx_ex_ops_uma_out out_copy;

		if (syscall_trap) {
			K_OOPS(k_usermode_from_copy(&in_copy, op_in, sizeof(in_copy)));
			op_in = &in_copy;
			op_out = &out_copy;
		}
#endif

		ret = flash_npcx_nor_ex_exec_uma(dev, op_in, op_out);
#ifdef CONFIG_USERSPACE
		if (ret == 0 && syscall_trap) {
			K_OOPS(k_usermode_to_copy(out, op_out, sizeof(out_copy)));
		}
#endif
		break;
	}
	case FLASH_NPCX_EX_OP_SET_QSPI_OPER:
	{
		struct npcx_ex_ops_qspi_oper_in *op_in = (struct npcx_ex_ops_qspi_oper_in *)in;
#ifdef CONFIG_USERSPACE
		struct npcx_ex_ops_qspi_oper_in in_copy;

		if (syscall_trap) {
			K_OOPS(k_usermode_from_copy(&in_copy, op_in, sizeof(in_copy)));
			op_in = &in_copy;
		}
#endif
		ret = flash_npcx_nor_ex_set_spi_spec(dev, op_in);
		break;
	}
	case FLASH_NPCX_EX_OP_GET_QSPI_OPER:
	{
		struct npcx_ex_ops_qspi_oper_out *op_out =
		(struct npcx_ex_ops_qspi_oper_out *)out;
#ifdef CONFIG_USERSPACE
		struct npcx_ex_ops_qspi_oper_out out_copy;

		if (syscall_trap) {
			op_out = &out_copy;
		}
#endif
		ret = flash_npcx_nor_ex_get_spi_spec(dev, op_out);
#ifdef CONFIG_USERSPACE
		if (ret == 0 && syscall_trap) {
			K_OOPS(k_usermode_to_copy(out, op_out, sizeof(out_copy)));
		}
#endif
		break;
	}
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}
#endif

static const struct flash_driver_api flash_npcx_nor_driver_api = {
	.read = flash_npcx_nor_read,
	.write = flash_npcx_nor_write,
	.erase = flash_npcx_nor_erase,
	.get_parameters = flash_npcx_nor_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_npcx_nor_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_npcx_nor_read_sfdp,
	.read_jedec_id = flash_npcx_nor_read_jedec_id,
#endif
#ifdef CONFIG_FLASH_EX_OP_ENABLED
	.ex_op = flash_npcx_nor_ex_op,
#endif
};

#ifdef CONFIG_FLASH_NPCX_FIU_NOR_SFDP_RUNTIME
static int flash_npcx_nor_sfdp_config(const struct device *dev)
{
	struct flash_npcx_nor_data *data = dev->data;
	uint8_t sfdp_buf[JESD216_SFDP_SIZE(NUM_OF_PARAMETER_HEADER)] __aligned(4) = {0};
	uint32_t bfp_buf[NUM_OF_BFP_ENTRIES] = {0};
	struct jesd216_sfdp_header *sfdp = (struct jesd216_sfdp_header *) sfdp_buf;
	const struct jesd216_param_header *bfpt_header = &sfdp->phdr[0];
	struct jesd216_bfp *bfp = (struct jesd216_bfp *)bfp_buf;
	struct jesd216_bfp_dw15 dw15;
	struct jesd216_bfp_dw16 dw16;
	int ret;

	/* BFPT is the first parameter table in the SFDP hierarchy. */
	ret = flash_npcx_nor_read_sfdp(dev, 0, sfdp_buf, sizeof(sfdp_buf));
	if (ret != 0) {
		LOG_WRN("SFDP header read failed: %d", ret);
		return ret;
	}

	if (jesd216_sfdp_magic(sfdp) != JESD216_SFDP_MAGIC) {
		LOG_WRN("Invalid SFDP magic: 0x%08x", jesd216_sfdp_magic(sfdp));
		return -EINVAL;
	}

	LOG_DBG("SFDP v%u.%u: %u parameter headers", sfdp->rev_major,
		sfdp->rev_minor, sfdp->nph + 1U);

	if (jesd216_param_id(bfpt_header) != JESD216_SFDP_PARAM_ID_BFP) {
		LOG_WRN("SFDP PH0 is not BFPT");
		return -ENOTSUP;
	}

	LOG_DBG("SFDP PH0: ID 0x%04x, rev %u.%u: %u DW @ 0x%06x",
		jesd216_param_id(bfpt_header), bfpt_header->rev_major, bfpt_header->rev_minor,
		bfpt_header->len_dw, jesd216_param_addr(bfpt_header));

	ret = flash_npcx_nor_read_sfdp(dev, jesd216_param_addr(bfpt_header), bfp_buf,
					 MIN(bfpt_header->len_dw, NUM_OF_BFP_ENTRIES) * sizeof(uint32_t));
	if (ret != 0) {
		LOG_WRN("BFPT read failed: %d", ret);
		return ret;
	}

	data->flash_size = (uint32_t) (jesd216_bfp_density(bfp) / 8U);
	LOG_DBG("SFDP DW2 density: %u bytes", data->flash_size);

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	uint32_t pages_size = jesd216_bfp_page_size(bfpt_header, bfp);
	data->layout.pages_size = pages_size;
	data->layout.pages_count = data->flash_size / pages_size;
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

	switch (data->qspi_cfg.rd_mode) {
		case NPCX_RD_MODE_NORMAL:
		case NPCX_RD_MODE_FAST:
			break;
		case NPCX_RD_MODE_FAST_DUAL_OUTPUT:
			ret = jesd216_bfp_read_support(bfpt_header, bfp, JESD216_MODE_112, NULL);
			if (ret < 0) {
				LOG_WRN("Fast dual output mode not supported by SFDP");
				data->qspi_cfg.rd_mode = NPCX_RD_MODE_FAST;
			}
			break;
		case NPCX_RD_MODE_FAST_DUAL_IO:
			ret = jesd216_bfp_read_support(bfpt_header, bfp, JESD216_MODE_122, NULL);
			if (ret < 0) {
				LOG_WRN("Fast dual IO mode not supported by SFDP");
				data->qspi_cfg.rd_mode = NPCX_RD_MODE_FAST;
			}
			break;
		case NPCX_RD_MODE_FAST_QUAD_OUTPUT:
			ret = jesd216_bfp_read_support(bfpt_header, bfp, JESD216_MODE_114, NULL);
			if (ret < 0) {
				LOG_WRN("Fast quad output mode not supported by SFDP");
				data->qspi_cfg.rd_mode = NPCX_RD_MODE_FAST;
			}
			break;
		case NPCX_RD_MODE_FAST_QUAD_IO:
			ret = jesd216_bfp_read_support(bfpt_header, bfp, JESD216_MODE_144, NULL);
			if (ret < 0) {
				LOG_WRN("Fast quad IO mode not supported by SFDP");
				data->qspi_cfg.rd_mode = NPCX_RD_MODE_FAST;
			}
			break;
		default:
			LOG_WRN("Unsupported read mode: %d", data->qspi_cfg.rd_mode);
			data->qspi_cfg.rd_mode = NPCX_RD_MODE_FAST;
			break;
	}

	ret = jesd216_bfp_decode_dw15(bfpt_header, bfp, &dw15);
	if (ret != 0) {
		LOG_WRN("BFPT DW15 decode failed: %d", ret);
	}
	else {
		data->qspi_cfg.qer_type = dw15.qer;
	}

	ret = jesd216_bfp_decode_dw16(bfpt_header, bfp, &dw16);
	if (ret != 0) {
		LOG_WRN("BFPT DW16 decode failed: %d", ret);
		/* Keep the 4-byte address entry method described by the DTS */
		dw16.enter_4ba = data->qspi_cfg.enter_4ba;
	}

	/* The default case keeps the configuration coming from the DTS */
	switch (jesd216_bfp_addrbytes(bfp)) {
		case JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_3B:
			data->qspi_cfg.enter_4ba = 0;
			data->qspi_cfg.four_byte_addr = false;
			break;
		case JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_3B4B:
			data->qspi_cfg.enter_4ba = dw16.enter_4ba;
			data->qspi_cfg.four_byte_addr = false;
			break;
		case JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_4B:
			data->qspi_cfg.enter_4ba = 0;
			data->qspi_cfg.four_byte_addr = true;
			break;
		default:
			LOG_WRN("Unsupported address bytes: %d", jesd216_bfp_addrbytes(bfp));
	}

	return 0;
}
#endif /* CONFIG_FLASH_NPCX_FIU_NOR_SFDP_RUNTIME */

/* Set the Quad Enable bit in the status register according to the QER type */
static int flash_npcx_nor_quad_enable(const struct device *dev)
{
	struct flash_npcx_nor_data *data = dev->data;
	uint8_t sts_reg[2] = {0};
	uint8_t qe_idx, qe_bit;
	int ret;

	switch (data->qspi_cfg.qer_type) {
	case JESD216_DW15_QER_S1B6:
		qe_idx = 1;
		qe_bit = 6;
		break;
	case JESD216_DW15_QER_S2B7:
		qe_idx = 2;
		qe_bit = 7;
		break;
	case JESD216_DW15_QER_S2B1v1:
	case JESD216_DW15_QER_S2B1v4:
	case JESD216_DW15_QER_S2B1v5:
	case JESD216_DW15_QER_S2B1v6:
		qe_idx = 2;
		qe_bit = 1;
		break;
	default:
		return -ENOTSUP;
	}

	if (data->qspi_cfg.qer_type == JESD216_DW15_QER_S2B7) {
		/* This QER exposes status register 2 through opcodes 0x3F/0x3E */
		ret = flash_npcx_uma_read(dev, 0x3F, &sts_reg[1], 1);
	} else {
		ret = flash_npcx_nor_read_status_regs(dev, sts_reg, 2);
	}

	if (ret != 0) {
		LOG_ERR("Enable quad access: read reg failed %d!", ret);
		return ret;
	}

	/* quad mode is already enabled, no need to set the QE bit again */
	if ((sts_reg[qe_idx - 1] & BIT(qe_bit)) != 0) {
		return 0;
	}

	sts_reg[qe_idx - 1] |= BIT(qe_bit);

	switch (data->qspi_cfg.qer_type) {
	case JESD216_DW15_QER_S1B6:
		ret = flash_npcx_nor_write_status_regs(dev, sts_reg, 1);
		break;
	case JESD216_DW15_QER_S2B7:
		ret = flash_npcx_nor_write_status_by_cmd(dev, 0x3E, &sts_reg[1], 1);
		break;
	case JESD216_DW15_QER_S2B1v6:
		ret = flash_npcx_nor_write_status_by_cmd(dev, SPI_NOR_CMD_WRSR2, &sts_reg[1], 1);
		break;
	default:
		ret = flash_npcx_nor_write_status_regs(dev, sts_reg, 2);
		break;
	}

	if (ret != 0) {
		LOG_ERR("Enable quad access: write reg failed %d!", ret);
	}

	return ret;
}

/* Enter 4-byte address mode using the methods advertised by JESD216 DW16 */
static int flash_npcx_nor_enter_4ba(const struct device *dev)
{
	struct flash_npcx_nor_data *data = dev->data;
	uint8_t enter_4ba = data->qspi_cfg.enter_4ba;
	int ret;

	/* All bits set means the supported methods are unknown */
	if (enter_4ba == 0xff) {
		return -ENOTSUP;
	}

	if (enter_4ba & BIT(6)) {
		/* flash always operates in 4-Byte address mode */
		return 0;
	}

	/* Only the 4BA instruction, with or without a preceding WREN, is supported */
	if ((enter_4ba & (BIT(0) | BIT(1))) == 0) {
		return -ENOTSUP;
	}

	if (enter_4ba & BIT(1)) {
		ret = flash_npcx_uma_cmd_only(dev, SPI_NOR_CMD_WREN);
		if (ret != 0) {
			LOG_ERR("Enable 4byte addr: WREN failed %d!", ret);
			return ret;
		}
	}

	ret = flash_npcx_uma_cmd_only(dev, SPI_NOR_CMD_4BA);
	if (ret != 0) {
		LOG_ERR("Enable 4byte addr: 4BA failed %d!", ret);
	}

	return ret;
}

static int flash_npcx_nor_init(const struct device *dev)
{
	struct flash_npcx_nor_data *data = dev->data;
	int ret;

	if (!IS_ENABLED(CONFIG_FLASH_NPCX_FIU_NOR_INIT)) {
		/* Without the sequence below only a permanently 4-byte device qualifies */
		data->qspi_cfg.four_byte_addr = (data->qspi_cfg.enter_4ba & BIT(6)) != 0;
		return 0;
	}

#ifdef CONFIG_FLASH_NPCX_FIU_NOR_SFDP_RUNTIME
	ret = flash_npcx_nor_sfdp_config(dev);
	if (ret != 0) {
		LOG_WRN("SFDP runtime configuration failed: %d", ret);
		LOG_WRN("Proceeding with default configuration from the DTS");
	}
#endif /* CONFIG_FLASH_NPCX_FIU_NOR_SFDP_RUNTIME */

	if ((data->qspi_cfg.rd_mode == NPCX_RD_MODE_FAST_QUAD_IO) || 
	    (data->qspi_cfg.rd_mode == NPCX_RD_MODE_FAST_QUAD_OUTPUT)) {
		/* Enable quad access of spi NOR flash */
		if (data->qspi_cfg.qer_type != JESD216_DW15_QER_NONE) {
			ret = flash_npcx_nor_quad_enable(dev);
			if (ret != 0) {
				LOG_ERR("Quad enable failed %d, fall back to fast read!", ret);
				data->qspi_cfg.rd_mode = NPCX_RD_MODE_FAST;
			}
		}
	}

	/* Enable 4-byte address of spi NOR flash */
	if (data->qspi_cfg.enter_4ba != 0) {
		ret = flash_npcx_nor_enter_4ba(dev);
		if (ret != 0) {
			LOG_ERR("Enter 4-byte address mode failed %d!", ret);
			data->qspi_cfg.four_byte_addr = false;
		}
		else {
			data->qspi_cfg.four_byte_addr = true;
		}
	}

	return 0;
}

#define NPCX_FLASH_NOR_INIT(n)							\
BUILD_ASSERT(IS_ENABLED(CONFIG_SOC_SERIES_NPCX4) ||				\
	     (DT_INST_STRING_TOKEN(n, rd_mode) != NPCX_RD_MODE_FAST_DUAL_OUTPUT	\
	      && DT_INST_STRING_TOKEN(n, rd_mode) != NPCX_RD_MODE_FAST_QUAD_OUTPUT),\
	     "Fast Dual/Quad Output read is only supported by the NPCX4 series");\
PINCTRL_DT_INST_DEFINE(n);							\
static const struct flash_npcx_nor_config flash_npcx_nor_config_##n = {		\
	.qspi_bus = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(n))),			\
	.mapped_addr = DT_INST_PROP(n, mapped_addr),				\
	.max_timeout = DT_INST_PROP(n, max_timeout),				\
};										\
static struct flash_npcx_nor_data flash_npcx_nor_data_##n = {			\
	.flash_size = DT_INST_PROP(n, size) / 8,				\
	.qspi_cfg = {								\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.flags = DT_INST_PROP(n, qspi_flags),				\
		.enter_4ba = DT_INST_PROP_OR(n, enter_4byte_addr, 0),		\
		.qer_type = DT_INST_QUAD_EN_PROP_OR(n),				\
		.rd_mode = DT_INST_STRING_TOKEN(n, rd_mode),			\
		.four_byte_addr = false, 							\
	},									\
	IF_ENABLED(CONFIG_FLASH_PAGE_LAYOUT, (					\
		.layout = {							\
			.pages_count = DT_INST_PROP(n, size) /			\
					      (8 * SPI_NOR_PAGE_SIZE),			\
			.pages_size  = SPI_NOR_PAGE_SIZE,				\
		},))								\
};										\
DEVICE_DT_INST_DEFINE(n, flash_npcx_nor_init, NULL,				\
		      &flash_npcx_nor_data_##n, &flash_npcx_nor_config_##n,	\
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,			\
		      &flash_npcx_nor_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NPCX_FLASH_NOR_INIT)
