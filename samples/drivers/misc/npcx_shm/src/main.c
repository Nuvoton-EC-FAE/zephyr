/*
 * Copyright (c) 2026 Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/misc/npcx_shm/npcx_shm.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define ESPI_FREQ_20MHZ 20u
#define SHM_WIN1 0
#define SHM_WIN2 1

/* Global flags to indicate host write to host_offset */
static bool shm_host_offset_written[2] = {false, false};

/* Global flags to indicate host read from core_offset */
static bool shm_core_offset_read[2] = {false, false};

/* Global flags to indicate host write to semaphore */
static bool shm_semaphore_written[2] = {false, false};

/* Semaphore to wake up main loop when callback events occur */
static struct k_sem shm_event_sem;

static void shm_win_gen_irq_callback(uint8_t sts, uint32_t win)
{

	if (sts & NPCX_SHM_INT_ON_HOST_RD_CORE_OFFSET) {
		/* Set flag to indicate host has read from core_offset */
		shm_core_offset_read[win] = true;
	}

	if (sts & NPCX_SHM_INT_ON_HOST_WR_HOST_OFFSET) {
		/* Set flag to indicate host has written to host_offset */
		shm_host_offset_written[win] = true;
	}

	if (sts & NPCX_SHM_INT_ON_HOST_WR_SEMAPHORE) {
		/* Set flag to indicate host has written to semaphore */
		shm_semaphore_written[win] = true;
	}

	/* Wake up main loop to process the event */
	k_sem_give(&shm_event_sem);
}

static void shm_win1_irq_callback(uint8_t sts)
{
	shm_win_gen_irq_callback(sts, SHM_WIN1);
}

static void shm_win2_irq_callback(uint8_t sts)
{
	shm_win_gen_irq_callback(sts, SHM_WIN2);
}

void espi_init(void)
{
	const struct device *espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi0));
	int ret;

	struct espi_cfg cfg = {
		.io_caps = ESPI_IO_MODE_SINGLE_LINE,
		.channel_caps = ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_PERIPHERAL,
		.max_freq = ESPI_FREQ_20MHZ,
	};

	ret = espi_config(espi_dev, &cfg);
	if (ret) {
		printk("Failed to configure eSPI target\n");
	} else {
		printk("eSPI target configured successfully!\n");
	}
}

int main(void)
{
	const struct device *dev_win1 = DEVICE_DT_GET(DT_NODELABEL(shm_win1));
	const struct device *dev_win2 = DEVICE_DT_GET(DT_NODELABEL(shm_win2));

	/*
	 * Initialize eSPI.
	 * If eSPI is not enable the eSPI module is powered down and the
	 * comunication between host and the SHM memory will not work.
	 */
	espi_init();

	/* Check if devices are ready */
	if (!device_is_ready(dev_win1)) {
		printk("shm_win1 is not ready\n");
		return 0;
	}

	if (!device_is_ready(dev_win2)) {
		printk("shm_win2 is not ready\n");
		return 0;
	}

	printk("NPCX SHM sample start\n");

	/* Initialize semaphore for event-driven main loop */
	k_sem_init(&shm_event_sem, 0, 1);

	/* prepare window 1 configuration */
	struct npcx_shm_window_config win1_cfg = {
		.callback = shm_win1_irq_callback,
		.window_base = 0x200D0000U,
		.window_size = 256U,
		.window_protection_mask = 0x0000U,
		.core_offset = 0x0000U,
		.host_offset = 0x0001U,
		.core_interrupts_mask = NPCX_SHM_INT_ON_HOST_RD_CORE_OFFSET | /* Generate interrupt when host read core offset */
								NPCX_SHM_INT_ON_HOST_WR_HOST_OFFSET | /* Generate interrupt when host write host offset */
								NPCX_SHM_INT_ON_HOST_WR_SEMAPHORE,    /* Generate interrupt when host write to semaphor */

		.host_interrupts_mask = NPCX_SHM_SMI_CORE_RD_FROM_HOST_OFFSET | /* Generate SMI when core read from host offset */
								NPCX_SHM_SMI_CORE_WR_TO_SEMAPHOR,       /* Generate SMI when core write to semaphore */
	};

	/* prepare window 2 configuration */
	struct npcx_shm_window_config win2_cfg = {
		.callback = shm_win2_irq_callback,
		.window_base = 0x200D0200U,
		.window_size = 256U,
		.window_protection_mask = 0x0000U,
		.core_offset = 0x0001U,
		.host_offset = 0x0002U,
		.core_interrupts_mask = NPCX_SHM_INT_ON_HOST_RD_CORE_OFFSET | /* Generate interrupt when host read core offset  */
								NPCX_SHM_INT_ON_HOST_WR_HOST_OFFSET | /* Generate interrupt when host write host offset */
								NPCX_SHM_INT_ON_HOST_WR_SEMAPHORE,    /* Generate interrupt when host write to semaphor */

		.host_interrupts_mask = NPCX_SHM_IRQ_CORE_RD_FROM_HOST_OFFSET | /* Generate host IRQ when core read from host offset */
								NPCX_SHM_IRQ_CORE_WR_TO_SEMAPHOR,       /* Generate host IRQ when core write to semaphore    */
	};

	printk("Configure SHM win1\n");
	npcx_shm_window_config(dev_win1, &win1_cfg);

	printk("Configure SHM win2\n");
	npcx_shm_window_config(dev_win2, &win2_cfg);

	while (1) {
		/* Wait for callback to signal an event */
		k_sem_take(&shm_event_sem, K_FOREVER);

		/* Check if host has read from win1 core_offset */
		if (shm_core_offset_read[SHM_WIN1]) {
			printk("SHM Win1 core_offset data read by host\n");
			/* TODO: Process the core_offset read event in your application. */
			shm_core_offset_read[SHM_WIN1] = false;
		}

		/* Check if host has read from win2 core_offset */
		if (shm_core_offset_read[SHM_WIN2]) {
			printk("SHM Win2 core_offset data read by host\n");
			/* TODO: Process the core_offset read event in your application. */
			shm_core_offset_read[SHM_WIN2] = false;
		}

		/* Check if host has written to win1 host_offset */
		if (shm_host_offset_written[SHM_WIN1]) {
			uint32_t win1_host_addr = win1_cfg.window_base + win1_cfg.host_offset;
			uint8_t data = *(volatile uint8_t *)win1_host_addr;

			printk("SHM Win1 host_offset data read: 0x%02x\n", data);
			/* TODO: Process the data from host in your application. */
			shm_host_offset_written[SHM_WIN1] = false;
		}

		/* Check if host has written to win2 host_offset */
		if (shm_host_offset_written[SHM_WIN2]) {
			uint32_t win2_host_addr = win2_cfg.window_base + win2_cfg.host_offset;
			uint8_t data = *(volatile uint8_t *)win2_host_addr;

			printk("SHM Win2 host_offset data read: 0x%02x\n", data);
			/* TODO: Process the data from host in your application. */
			shm_host_offset_written[SHM_WIN2] = false;
		}

		/* Check if host has written to win1 semaphore */
		if (shm_semaphore_written[SHM_WIN1]) {
			uint8_t sem_val = npcx_shm_semaphore_read(dev_win1);

			printk("SHM Win1 semaphore value: 0x%02x\n", sem_val);
			/* TODO: Use sem_val to trigger window-specific handling in your application. */
			shm_semaphore_written[SHM_WIN1] = false;
		}

		/* Check if host has written to win2 semaphore */
		if (shm_semaphore_written[SHM_WIN2]) {
			uint8_t sem_val = npcx_shm_semaphore_read(dev_win2);

			printk("SHM Win2 semaphore value: 0x%02x\n", sem_val);
			/* TODO: Use sem_val to trigger window-specific handling in your application. */
			shm_semaphore_written[SHM_WIN2] = false;
		}
	}

	return 0;
}
