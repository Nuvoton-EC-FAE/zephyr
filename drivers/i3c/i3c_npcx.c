/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcx_i3c

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/target_device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(npcx_i3c, CONFIG_I3C_LOG_LEVEL);

#define NPCX_I3C_CHK_TIMEOUT 10000    /* Unit: us, timeout for checking register status */
#define I3C_SCL_PP_FREQ_MAX  12500000 /* Unit: Mhz*/
#define I3C_SCL_OD_FREQ_MAX  4170000  /* Unit: Mhz*/

#define I3C_BUS_TLOW_PP_MIN_NS  24  /* T_LOW period in push-pull mode */
#define I3C_BUS_THigh_PP_MIN_NS 24  /* T_High period in push-pull mode */
#define I3C_BUS_TLOW_OD_MIN_NS  200 /* T_LOW period in open-drain mode */

#define PPBAUD_DIV_MAX (BIT(GET_FIELD_SZ(NPCX_I3C_MCONFIG_PPBAUD)) - 1) /* PPBAUD divider max */

#define DAA_TGT_INFO_SZ 0x8 /* 8 bytes = PID(6) + BCR(1) + DCR(1) */
#define BAMATCH_DIV     0x4 /* BAMATCH = APB4_CLK divided by four */

/* Supported I3C MCLKD frequency */
enum npcx_i3c_speed {
	NPCX_I3C_BUS_SPEED_45MHZ,
};

/* I3C timing configuration for each i3c speed */
struct npcx_i3c_freq_cfg {
	uint8_t ppbaud; /* Push-Pull high period */
	uint8_t pplow;  /* Push-Pull low period */
	uint8_t odhpp;  /* Open-Drain high period */
	uint8_t odbaud; /* Open-Drain low period */
};

/* Recommended I3C timing values are based on MCLKD 45 MHz */
static const struct npcx_i3c_freq_cfg npcx_def_speed_cfg[] = {
	/* PP = 12.5 mhz, OD = 4.17 Mhz */
	[NPCX_I3C_BUS_SPEED_45MHZ] = {.ppbaud = 1, .pplow = 0, .odhpp = 1, .odbaud = 4},
};

struct npcx_i3c_config {
	/* Common I3C Driver Config */
	struct i3c_driver_config common;

	/* Pointer to controller registers. */
	struct i3c_reg *base;

	/* Pointer to controller registers. */
	struct mdma_reg *mdma;

	/* Pointer to the clock device. */
	const struct device *clock_dev;

	/* Reset controller */
	struct reset_dt_spec reset;

	/* Clock control subsys related struct. */
	struct npcx_clk_cfg clock_subsys;

	/* Reference clock to determine 1-μs bus available time */
	struct npcx_clk_cfg ref_clk_subsys;

	/* Pointer to pin control device. */
	const struct pinctrl_dev_config *pincfg;

	/* Interrupt configuration function. */
	void (*irq_config_func)(const struct device *dev);

	/* Disable open drain high push pull */
	bool disable_open_drain_high_pp;

	bool target_mode;
	bool secondary_mode;
	uint8_t  instance_id;
	uint8_t  mdma_channel;
	uint8_t  static_address;
	uint8_t  bcr;
	uint8_t  dcr;
	uint32_t  part_id;
	uint32_t  vendor_id;
};

struct npcx_i3c_data {
	struct i3c_driver_data common; /* Common i3c driver data */
	struct k_sem lock_sem;         /* Mutex of i3c controller */
	struct k_sem sync_sem;         /* Semaphore used for synchronization */
	struct k_sem ibi_lock_sem;     /* Semaphore used for ibi */
	struct k_sem target_lock_sem;         /* Mutex of i3c target */
	struct k_sem target_event_sem;         /* Mutex of i3c target ibi/hot join/bus controll event */

	struct {
		/* I3C open drain clock frequency in Hz. */
		uint32_t i3c_od_scl_hz;
	} clocks;

    struct i3c_target_config *target_config;

#ifdef CONFIG_I3C_USE_IBI
	struct {
		/* List of addresses used in the MIBIRULES register. */
		uint8_t addr[5];

		/* Number of valid addresses in MIBIRULES. */
		uint8_t num_addr;

		/* True if all addresses have MSB set. */
		bool msb;

		/*
		 * True if all target devices require mandatory byte
		 * for IBI.
		 */
		bool has_mandatory_byte;
	} ibi;
#endif
};

static inline uint32_t npcx_i3c_state_get(struct i3c_reg *inst)
{
	return GET_FIELD(inst->MSTATUS, NPCX_I3C_MSTATUS_STATE);
}

static inline void npcx_i3c_interrupt_disable(struct i3c_reg *inst, uint32_t mask)
{
	inst->MINTCLR = mask;
}

static inline void npcx_i3c_interrupt_enable(struct i3c_reg *inst, uint32_t mask)
{
	inst->MINTSET = mask;
}

static bool npcx_i3c_has_error(struct i3c_reg *inst)
{
	if (IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_ERRWARN)) {

		LOG_DBG("ERROR: MSTATUS 0x%08x MERRWARN 0x%08x",
			inst->MSTATUS, inst->MERRWARN);

		return true;
	}

	return false;
}

static inline void npcx_i3c_status_clear_all(struct i3c_reg *inst)
{
	uint32_t mask = (BIT(NPCX_I3C_MSTATUS_TGTSTART) | BIT(NPCX_I3C_MSTATUS_MCTRLDONE) |
			 BIT(NPCX_I3C_MSTATUS_COMPLETE) | BIT(NPCX_I3C_MSTATUS_IBIWON) |
			 BIT(NPCX_I3C_MSTATUS_NOWCNTLR));

	inst->MSTATUS = mask;
}

static inline void npcx_i3c_errwarn_clear_all(struct i3c_reg *inst)
{
	inst->MERRWARN = inst->MERRWARN;
}

static inline void npcx_i3c_fifo_flush(struct i3c_reg *inst)
{
	inst->MDATACTRL |= (BIT(NPCX_I3C_MDATACTRL_FLUSHTB) | BIT(NPCX_I3C_MDATACTRL_FLUSHFB));
}

/* Start DAA procedure and continue the DAA with a Repeated START */
static inline void npcx_i3c_request_daa(struct i3c_reg *inst)
{
	uint32_t val = 0;

	/* Set IBI response NACK while processing DAA */
	SET_FIELD(val, NPCX_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_NACK);

	/* Send DAA request */
	SET_FIELD(val, NPCX_I3C_MCTRL_REQUEST, MCTRL_REQUEST_PROCESSDAA);

	inst->MCTRL = val;
}

/* Tell controller to start auto IBI */
static inline void npcx_i3c_request_auto_ibi(struct i3c_reg *inst)
{
	uint32_t val = 0;

	SET_FIELD(val, NPCX_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_ACK);
	SET_FIELD(val, NPCX_I3C_MCTRL_REQUEST, MCTRL_REQUEST_AUTOIBI);

	inst->MCTRL = val;
}

/*
 * brief:  Controller emit start and send address
 *
 * param[in] inst     Pointer to I3C register.
 * param[in] addr     Dyamic address for xfer or 0x7E for CCC command.
 * param[in] op_type  Request type.
 * param[in] is_read  Read(true) or write(false) operation.
 * param[in] read_sz  Read size.
 *
 * return  0, success
 *         else, error
 */
static int npcx_i3c_request_emit_start(struct i3c_reg *inst, uint8_t addr,
				       enum npcx_i3c_mctrl_type op_type, bool is_read,
				       size_t read_sz)
{
	uint32_t mctrl = 0;
	int ret = 0;

	/* Set request and target address*/
	SET_FIELD(mctrl, NPCX_I3C_MCTRL_REQUEST, MCTRL_REQUEST_EMITSTARTADDR);

	/* Set operation type */
	SET_FIELD(mctrl, NPCX_I3C_MCTRL_TYPE, op_type);

	/* Set IBI response NACK in emit start */
	SET_FIELD(mctrl, NPCX_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_NACK);

	/* Set dynamic address */
	SET_FIELD(mctrl, NPCX_I3C_MCTRL_ADDR, addr);

	/* Set read(1) or write(0) */
	if (is_read == true) {
		mctrl |= BIT(NPCX_I3C_MCTRL_DIR);
		SET_FIELD(mctrl, NPCX_I3C_MCTRL_RDTERM, read_sz); /* Set read length */
	} else {
		mctrl &= ~BIT(NPCX_I3C_MCTRL_DIR);
	}

	/* Apply MCTRL register */
	inst->MCTRL = mctrl;

	/* Wait for controller request done */
	if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_MCTRLDONE), NPCX_I3C_CHK_TIMEOUT,
		     NULL) == false) {
		ret = -ETIMEDOUT;
		LOG_DBG("%s: check MCTRLDONE timed out", __func__);

		goto ccc_start_out;
	}

	/* Check NACK after MCTRLDONE is get */
	if (IS_BIT_SET(inst->MERRWARN, NPCX_I3C_MERRWARN_NACK)) {
		ret = -ENODEV;
		LOG_DBG("%s: ERR: nack", __func__);
	}

ccc_start_out:
	inst->MSTATUS = BIT(NPCX_I3C_MSTATUS_MCTRLDONE); /* W1C */

	return ret;
}

/*
 * brief:  Controller emit STOP.
 *
 * This emits STOP when controller is in NORMACT state.
 *
 * param[in] inst  Pointer to I3C register.
 *
 */
static inline void npcx_i3c_request_emit_stop(struct i3c_reg *inst)
{
	uint32_t i3c_state = npcx_i3c_state_get(inst);

	/* Make sure we are in a state where we can emit STOP */
	if ((i3c_state != MSTATUS_STATE_NORMACT) && (i3c_state != MSTATUS_STATE_DAA)) {
		return;
	}

	/* Emit stop */
	SET_FIELD(inst->MCTRL, NPCX_I3C_MCTRL_REQUEST, MCTRL_REQUEST_EMITSTOP);
}

static inline void npcx_i3c_ibi_respond_nack(struct i3c_reg *inst)
{
	uint32_t val = 0;

	SET_FIELD(val, NPCX_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_NACK);
	SET_FIELD(val, NPCX_I3C_MCTRL_REQUEST, MCTRL_REQUEST_IBIACKNACK);

	inst->MCTRL = val;

	/* Wait request done */
	if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_MCTRLDONE), NPCX_I3C_CHK_TIMEOUT,
		     NULL) == false) {
		LOG_DBG("ibi_respond_nack timeout");
	}

	inst->MSTATUS = BIT(NPCX_I3C_MSTATUS_MCTRLDONE); /* W1C */
}

static inline void npcx_i3c_ibi_respond_ack(struct i3c_reg *inst)
{
	uint32_t val = 0;

	SET_FIELD(val, NPCX_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_ACK);
	SET_FIELD(val, NPCX_I3C_MCTRL_REQUEST, MCTRL_REQUEST_IBIACKNACK);

	inst->MCTRL = val;

	/* Wait request done */
	if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_MCTRLDONE), NPCX_I3C_CHK_TIMEOUT,
		     NULL) == false) {
		LOG_DBG("ibi_respond_ack timeout");
	}

	inst->MSTATUS = BIT(NPCX_I3C_MSTATUS_MCTRLDONE); /* W1C */
}

/*
 * brief:  Find a registered I3C target device.
 *
 * This returns the I3C device descriptor of the I3C device
 * matching the incoming id.
 *
 * param[in] dev  Pointer to controller device driver instance.
 * param[in] id   Pointer to I3C device ID.
 *
 * return  see i3c_device_find.
 */
static inline struct i3c_device_desc *npcx_i3c_device_find(const struct device *dev,
							   const struct i3c_device_id *id)
{
	const struct npcx_i3c_config *config = dev->config;

	return i3c_dev_list_find(&config->common.dev_list, id);
}

/*
 * brief:  Perform bus recovery.
 *
 * param[in] dev  Pointer to controller device driver instance.
 */
static int npcx_i3c_recover_bus(const struct device *dev)
{
	const struct npcx_i3c_config *config = dev->config;
	struct i3c_reg *inst = config->base;
	int ret = 0;

	/*
	 * If the controller is in NORMACT state, tells it to emit STOP
	 * so it can return to IDLE, or is ready to clear any pending
	 * target initiated IBIs.
	 */
	if (npcx_i3c_state_get(inst) == MSTATUS_STATE_NORMACT) {
		npcx_i3c_request_emit_stop(inst);
	};

	/* Exhaust all target initiated IBI */
	while (IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_TGTSTART)) {
		/* Tell the controller to perform auto IBI. */
		npcx_i3c_request_auto_ibi(inst);

		if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_COMPLETE),
			     NPCX_I3C_CHK_TIMEOUT, NULL) == false) {
			break;
		}

		/* Once auto IBI is done, discard bytes in FIFO. */
		while (IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_RXPEND)) {
			/* Flush FIFO as long as RXPEND is set. */
			npcx_i3c_fifo_flush(inst);
		}

		/*
		 * There might be other IBIs waiting.
		 * So pause a bit to let other targets initiates
		 * their IBIs.
		 */
		k_busy_wait(100);
	}

	/* Check IDLE state */
	if (WAIT_FOR((npcx_i3c_state_get(inst) == MSTATUS_STATE_IDLE), NPCX_I3C_CHK_TIMEOUT,
		     NULL) == false) {
		ret = -EBUSY;
	}

	return ret;
}

static inline void npcx_i3c_xfer_reset(struct i3c_reg *inst)
{
	npcx_i3c_status_clear_all(inst);
	npcx_i3c_errwarn_clear_all(inst);
	npcx_i3c_fifo_flush(inst);
}

/*
 * brief:  Perform one write transaction.
 *
 * This writes all data in buf to TX FIFO or time out
 * waiting for FIFO spaces.
 *
 * param[in] inst       Pointer to controller registers.
 * param[in] buf        Buffer containing data to be sent.
 * param[in] buf_sz     Number of bytes in buf to send.
 * param[in] no_ending  True, not including ending byte in message.
 *                       False, including ending byte in message
 *
 * return  Number of bytes written, or negative if error.
 *
 */
static int npcx_i3c_xfer_write_fifo(struct i3c_reg *inst, uint8_t *buf, uint8_t buf_sz,
				    bool no_ending)
{
	int offset = 0;
	int remaining = buf_sz;
	int ret = 0;

	while (remaining > 0) {
		/* Check tx fifo not full */
		if (WAIT_FOR(!IS_BIT_SET(inst->MDATACTRL, NPCX_I3C_MDATACTRL_TXFULL),
			     NPCX_I3C_CHK_TIMEOUT, NULL) == false) {
			ret = -ETIMEDOUT;
			LOG_DBG("%s: check tx fifo not full timed out", __func__);

			goto one_xfer_write_out;
		}

		if ((remaining > 1) || no_ending) {
			inst->MWDATAB = (uint32_t)buf[offset];
		} else {
			inst->MWDATABE = (uint32_t)buf[offset];
		}

		offset += 1;
		remaining -= 1;
	}

	ret = offset;

one_xfer_write_out:
	return ret;
}

/*
 * brief:  Perform read transaction.
 *
 * This reads from RX FIFO until COMPLETE bit is set in MSTATUS
 * or time out.
 *
 * param[in] inst    Pointer to controller registers.
 * param[in] buf     Buffer to store data.
 * param[in] buf_sz  Number of bytes to read.
 *
 * return  Number of bytes read, or negative if error.
 *
 */
static int npcx_i3c_xfer_read_fifo(struct i3c_reg *inst, uint8_t *buf, uint8_t rd_sz)
{
	bool is_done = false;
	int ret = 0;
	int offset = 0;

	while (is_done == false) {
		/* Check message is terminated */
		if (IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_COMPLETE)) {
			is_done = true;
		}

		/* Check I3C bus error */
		if (npcx_i3c_has_error(inst)) {
			/* Check timeout*/
			if (IS_BIT_SET(inst->MERRWARN, NPCX_I3C_MERRWARN_TIMEOUT)) {
				ret = -ETIMEDOUT;
				LOG_DBG("%s: ERR: timeout", __func__);
			}

			inst->MERRWARN = inst->MERRWARN;
			ret = -EIO;

			goto xfer_read_out;
		}

		/* Check rx not empty */
		if (IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_RXPEND)) {

			/* Receive all the data in this round.
			 * Read in a tight loop to reduce chance of losing
			 * FIFO data when the i3c speed is high.
			 */
			while (offset < rd_sz) {
				if (GET_FIELD(inst->MDATACTRL, NPCX_I3C_MDATACTRL_RXCOUNT) == 0) {
					break;
				}

				buf[offset++] = (uint8_t)inst->MRDATAB;
			}
		}
	}

	ret = offset;

xfer_read_out:
	return ret;
}

/*
 * brief:  Perform one transfer transaction.
 *
 * param[in] inst        Pointer to controller registers.
 * param[in] addr        Target address.
 * param[in] op_type     Request type.
 * param[in] buf         Buffer for data to be sent or received.
 * param[in] buf_sz      Buffer size in bytes.
 * param[in] is_read     True if this is a read transaction, false if write.
 * param[in] emit_start  True if START is needed before read/write.
 * param[in] emit_stop   True if STOP is needed after read/write.
 * param[in] no_ending   True if not to signal end of write message.
 *
 * return  Number of bytes read/written, or negative if error.
 */
static int npcx_i3c_do_one_xfer(struct i3c_reg *inst, uint8_t addr,
				enum npcx_i3c_mctrl_type op_type, uint8_t *buf, size_t buf_sz,
				bool is_read, bool emit_start, bool emit_stop, bool no_ending)
{
	int ret = 0;

	npcx_i3c_status_clear_all(inst);
	npcx_i3c_errwarn_clear_all(inst);

	/* Emit START if needed */
	if (emit_start) {
		ret = npcx_i3c_request_emit_start(inst, addr, op_type, is_read, buf_sz);
		if (ret != 0) {
			emit_stop = true;

			goto out_one_xfer;
		}
	}

	/* No data to be transferred */
	if ((buf == NULL) || (buf_sz == 0)) {
		goto out_one_xfer;
	}

	/* Select read or write operation */
	if (is_read) {
		ret = npcx_i3c_xfer_read_fifo(inst, buf, buf_sz);
	} else {
		ret = npcx_i3c_xfer_write_fifo(inst, buf, buf_sz, no_ending);
	}

	/* Check message complete if is a read transaction or
	 * ending byte of a write transaction.
	 */
	if (is_read || !no_ending) {
		/* Wait message transfer complete */
		if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_COMPLETE),
			     NPCX_I3C_CHK_TIMEOUT, NULL) == false) {
			LOG_DBG("%s: timed out addr 0x%02x, buf_sz %u", __func__, addr, buf_sz);

			ret = -ETIMEDOUT;
			emit_stop = true;

			goto out_one_xfer;
		}

		inst->MSTATUS = BIT(NPCX_I3C_MSTATUS_COMPLETE); /* W1C */
	}

	/* Check I3C bus error */
	if (npcx_i3c_has_error(inst)) {
		ret = -EIO;
	}

out_one_xfer:
	/* Emit STOP if needed */
	if (emit_stop) {
		npcx_i3c_request_emit_stop(inst);
	}

	return ret;
}

void npcx_i3c_mutex_lock(const struct device *i3c_dev)
{
	struct npcx_i3c_data *const data = i3c_dev->data;

	k_sem_take(&data->lock_sem, K_FOREVER);
}

void npcx_i3c_mutex_unlock(const struct device *i3c_dev)
{
	struct npcx_i3c_data *const data = i3c_dev->data;

	k_sem_give(&data->lock_sem);
}

/*
 * brief:  Transfer messages in I3C mode.
 *
 * see i3c_transfer
 *
 * param[in] dev       Pointer to device driver instance.
 * param[in] target    Pointer to target device descriptor.
 * param[in] msgs      Pointer to I3C messages.
 * param[in] num_msgs  Number of messages to transfers.
 *
 * return  see i3c_transfer
 */
static int npcx_i3c_transfer(const struct device *dev, struct i3c_device_desc *target,
			     struct i3c_msg *msgs, uint8_t num_msgs)
{
	const struct npcx_i3c_config *config = dev->config;
	struct i3c_reg *inst = config->base;
	uint32_t intmask, xfered_len;
	int ret;
	bool send_broadcast = true;

	if (msgs == NULL) {
		ret = -EINVAL;
		goto out_xfer_i3c;
	}

	if (target->dynamic_addr == 0U) {
		ret = -EINVAL;
		goto out_xfer_i3c;
	}

	npcx_i3c_mutex_lock(dev);

	/* Disable interrupt */
	intmask = inst->MINTSET;
	npcx_i3c_interrupt_disable(inst, intmask);

	/* Check bus in idle state */
	if (WAIT_FOR((npcx_i3c_state_get(inst) == MSTATUS_STATE_IDLE), NPCX_I3C_CHK_TIMEOUT,
		     NULL) == false) {
		ret = -ETIMEDOUT;

		goto out_xfer_i3c_unlock;
	}

	npcx_i3c_xfer_reset(inst);

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {

		/*
		 * Check message is read or write operaion.
		 * For write operation, check the last data byte of a transmit message.
		 */
		bool is_read = (msgs[i].flags & I3C_MSG_RW_MASK) == I3C_MSG_READ;
		bool no_ending = false;

		/*
		 * Emit start if this is the first message or that
		 * the RESTART flag is set in message.
		 */
		bool emit_start =
			(i == 0) || ((msgs[i].flags & I3C_MSG_RESTART) == I3C_MSG_RESTART);

		bool emit_stop = (msgs[i].flags & I3C_MSG_STOP) == I3C_MSG_STOP;

		/*
		 * The controller requires special treatment of last byte of
		 * a write message. Since the API permits having a bunch of
		 * write messages without RESTART in between, this is just some
		 * logic to determine whether to treat the last byte of this
		 * message to be the last byte of a series of write mssages.
		 * If not, tell the write function not to treat it that way.
		 */
		if (!is_read && !emit_stop && ((i + 1) != num_msgs)) {
			bool next_is_write = (msgs[i + 1].flags & I3C_MSG_RW_MASK) == I3C_MSG_WRITE;
			bool next_is_restart =
				((msgs[i + 1].flags & I3C_MSG_RESTART) == I3C_MSG_RESTART);

			/* Check next msg is still write operation and not including Sr */
			if (next_is_write && !next_is_restart) {
				no_ending = true;
			}
		}

		/*
		 * Two ways to do read/write transfer .
		 * 1. [S] + [0x7E]    + [address] + [data] + [Sr or P]
		 * 2. [S] + [address] + [data]    + [Sr or P]
		 *
		 * Send broadcast header(0x7E) on first transfer or after a STOP,
		 * unless flag is set not to.
		 */
		if (!(msgs[i].flags & I3C_MSG_NBCH) && (send_broadcast)) {
			ret = npcx_i3c_request_emit_start(inst, I3C_BROADCAST_ADDR,
							  NPCX_I3C_MCTRL_TYPE_I3C, false, 0);
			if (ret < 0) {
				LOG_ERR("emit start of broadcast addr failed, error (%d)", ret);
				goto out_xfer_i3c_stop_unlock;
			}
			send_broadcast = false;
		}

		/* Do transfer with target device */
		xfered_len = npcx_i3c_do_one_xfer(inst, target->dynamic_addr,
						  NPCX_I3C_MCTRL_TYPE_I3C, msgs[i].buf, msgs[i].len,
						  is_read, emit_start, emit_stop, no_ending);
		if (xfered_len < 0) {
			LOG_DBG("%s: do xfer fail", __func__);
			ret = xfered_len;

			goto out_xfer_i3c_stop_unlock;
		}

		/* Write back the total number of bytes transferred */
		msgs[i].num_xfer = xfered_len;

		if (emit_stop) {
			/* After a STOP, send broadcast header before next msg */
			send_broadcast = true;
		}
	}

	ret = 0;

out_xfer_i3c_stop_unlock:
	npcx_i3c_request_emit_stop(inst);

out_xfer_i3c_unlock:
	npcx_i3c_errwarn_clear_all(inst);
	npcx_i3c_status_clear_all(inst);

	npcx_i3c_interrupt_enable(inst, intmask);

	npcx_i3c_mutex_unlock(dev);

out_xfer_i3c:
	return ret;
}

/*
 * brief:  Perform Dynamic Address Assignment.
 *
 * param[in] dev  Pointer to controller device driver instance.
 *
 * return  0 If successful.
 *         -EBUSY Bus is busy.
 *         -EIO General input / output error.
 *         -ENODEV If a provisioned ID does not match to any target devices
 *                 in the registered device list.
 *         -ENOSPC No more free addresses can be assigned to target.
 *         -ENOSYS Dynamic address assignment is not supported by
 *                 the controller driver.
 */
static int npcx_i3c_do_daa(const struct device *dev)
{
	const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	struct i3c_reg *inst = config->base;
	int ret = 0;
	uint8_t rx_buf[8];
	size_t rx_count;
	uint32_t intmask;
	bool emit_stop = true;

	npcx_i3c_mutex_lock(dev);

	memset(rx_buf, 0xff, sizeof(rx_buf));

	/* Check bus in idle state */
	if (WAIT_FOR((npcx_i3c_state_get(inst) == MSTATUS_STATE_IDLE), NPCX_I3C_CHK_TIMEOUT,
		     NULL) == false) {
		ret = -ETIMEDOUT;
		LOG_DBG("err: DAA_STA:%#x", npcx_i3c_state_get(inst));

		goto out_daa_unlock;
	}

	LOG_DBG("DAA: ENTDAA");

	/* Disable interrupt */
	intmask = inst->MINTSET;
	npcx_i3c_interrupt_disable(inst, intmask);

	npcx_i3c_xfer_reset(inst);

	/* Emit process DAA */
	npcx_i3c_request_daa(inst);

	/* Loop until no more responses from devices */
	do {
		/* Check request done and target data in rx fifo */
		if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_MCTRLDONE),
			     NPCX_I3C_CHK_TIMEOUT, NULL) == false) {
			ret = -ETIMEDOUT;
			LOG_ERR("MCTRLDONE timeout error");

			goto out_daa;
		}

		/* Check ERRWARN bit set */
		if (npcx_i3c_has_error(inst)) {
			ret = -EIO;
			LOG_ERR("DAA recv error");

			goto out_daa;
		}

		inst->MSTATUS = BIT(NPCX_I3C_MSTATUS_MCTRLDONE); /* W1C */

		/* Receive Provisioned ID, BCR and DCR (total 8 bytes) */
		rx_count = GET_FIELD(inst->MDATACTRL, NPCX_I3C_MDATACTRL_RXCOUNT);

		if (rx_count == DAA_TGT_INFO_SZ) {
			for (int i = 0; i < rx_count; i++) {
				rx_buf[i] = (uint8_t)inst->MRDATAB;
			}
		} else {
			/* Data count not as expected, exit DAA*/
			ret = -EBADMSG;
			npcx_i3c_fifo_flush(inst);
			LOG_DBG("Rx count not as expected %d, abort DAA", rx_count);

			goto out_daa;
		}

		/* Start assign dynamic address */
		if ((npcx_i3c_state_get(inst) == MSTATUS_STATE_DAA) &&
		    IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_BETWEEN)) {
			struct i3c_device_desc *target;
			uint16_t vendor_id;
			uint32_t part_no;
			uint64_t pid;
			uint8_t dyn_addr = 0;

			/* PID[47:33] = manufacturer ID */
			vendor_id = (((uint16_t)rx_buf[0] << 8U) | (uint16_t)rx_buf[1]) & 0xFFFEU;

			/* PID[31:0] = vendor fixed falue or random value */
			part_no = (uint32_t)rx_buf[2] << 24U | (uint32_t)rx_buf[3] << 16U |
				  (uint32_t)rx_buf[4] << 8U | (uint32_t)rx_buf[5];

			/* Combine into one Provisioned ID */
			pid = (uint64_t)vendor_id << 32U | (uint64_t)part_no;

			LOG_DBG("DAA: Rcvd PID 0x%04x%08x", vendor_id, part_no);

			/* Find a usable address during ENTDAA */
			ret = i3c_dev_list_daa_addr_helper(&data->common.attached_dev.addr_slots,
							   &config->common.dev_list, pid, false,
							   false, &target, &dyn_addr);
			if (ret != 0) {
				goto out_daa;
			}

			if (target == NULL) {
				LOG_INF("%s: PID 0x%04x%08x is not in registered device "
					"list, given dynamic address 0x%02x",
					dev->name, vendor_id, part_no, dyn_addr);
			} else {
				/* Update target descriptor */
				target->dynamic_addr = dyn_addr;
				target->bcr = rx_buf[6];
				target->dcr = rx_buf[7];
			}

			/* Mark the address as I3C device */
			i3c_addr_slots_mark_i3c(&data->common.attached_dev.addr_slots, dyn_addr);

			/*
			 * If the device has static address, after address assignment,
			 * the device will not respond to the static address anymore.
			 * So free the static one from address slots if different from
			 * newly assigned one.
			 */
			if ((target != NULL) && (target->static_addr != 0U) &&
			    (dyn_addr != target->static_addr)) {
				i3c_addr_slots_mark_free(&data->common.attached_dev.addr_slots,
							 dyn_addr);
			}

			/* Emit process DAA again to send the address to the device */
			inst->MWDATAB = dyn_addr;
			npcx_i3c_request_daa(inst);

			LOG_DBG("PID 0x%04x%08x assigned dynamic address 0x%02x", vendor_id,
				part_no, dyn_addr);

			/* Target did not accept the assigned DA, exit DAA */
			if (IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_NACKED)) {
				ret = -EFAULT;
				LOG_DBG("TGT NACK assigned DA %#x", dyn_addr);

				/* Free the reserved DA */
				i3c_addr_slots_mark_free(&data->common.attached_dev.addr_slots,
							 dyn_addr);

				/* 0 if address has not been assigned */
				if (target != NULL) {
					target->dynamic_addr = 0;
				}

				goto out_daa;
			}
		}

		/* Check all targets have been assigned DA and DAA complete */
	} while ((!IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_COMPLETE)) &&
		 npcx_i3c_state_get(inst) != MSTATUS_STATE_IDLE);

	/* DAA process complete, no need to emit extra stop */
	emit_stop = false;

out_daa:
	/* Exit DAA mode */
	if (emit_stop == true) {
		npcx_i3c_request_emit_stop(inst);
	}

	/* Clear all flags. */
	npcx_i3c_errwarn_clear_all(inst);
	npcx_i3c_status_clear_all(inst);

	/* Re-Enable I3C IRQ sources. */
	npcx_i3c_interrupt_enable(inst, intmask);

out_daa_unlock:
	npcx_i3c_mutex_unlock(dev);

	return ret;
}

/*
 * brief:  Send Common Command Code (CCC).
 *
 * param[in] dev      Pointer to controller device driver instance.
 * param[in] payload  Pointer to CCC payload.
 *
 * return:  The same as i3c_do_ccc()
 *          0 If successful.
 *          -EBUSY Bus is busy.
 *          -EIO General Input / output error.
 *          -EINVAL Invalid valid set in the payload structure.
 *          -ENOSYS Not implemented.
 */
static int npcx_i3c_do_ccc(const struct device *dev, struct i3c_ccc_payload *payload)
{
	const struct npcx_i3c_config *config = dev->config;
	int ret = 0;
	struct i3c_reg *inst = config->base;
	uint32_t intmask, xfered_len;

	if (payload == NULL) {
		return -EINVAL;
	}

	if (config->common.dev_list.num_i3c == 0) {
		/*
		 * No i3c devices in dev tree. Just return so
		 * we don't get errors doing cmds when there
		 * are no devices listening/responding.
		 */
		return 0;
	}

	npcx_i3c_mutex_lock(dev);

	/* Disable interrupt */
	intmask = inst->MINTSET;
	npcx_i3c_interrupt_disable(inst, intmask);

	/* Clear status and flush fifo */
	npcx_i3c_xfer_reset(inst);

	LOG_DBG("CCC[0x%02x]", payload->ccc.id);

	/* Write emit START and broadcast address (0x7E) */
	ret = npcx_i3c_request_emit_start(inst, I3C_BROADCAST_ADDR, NPCX_I3C_MCTRL_TYPE_I3C, false,
					  0);
	if (ret < 0) {
		LOG_ERR("CCC[0x%02x] %s START error (%d)", payload->ccc.id,
			i3c_ccc_is_payload_broadcast(payload) ? "broadcast" : "direct", ret);

		goto out_ccc_stop;
	}

	/* Write CCC command */
	npcx_i3c_status_clear_all(inst);
	npcx_i3c_errwarn_clear_all(inst);
	xfered_len = npcx_i3c_xfer_write_fifo(inst, &payload->ccc.id, 1, payload->ccc.data_len > 0);
	if (xfered_len < 0) {
		LOG_ERR("CCC[0x%02x] %s command error (%d)", payload->ccc.id,
			i3c_ccc_is_payload_broadcast(payload) ? "broadcast" : "direct", ret);
		ret = xfered_len;

		goto out_ccc_stop;
	}

	/* Write data (defining byte or data bytes) for CCC if needed */
	if (payload->ccc.data_len > 0) {
		npcx_i3c_status_clear_all(inst);
		npcx_i3c_errwarn_clear_all(inst);
		xfered_len = npcx_i3c_xfer_write_fifo(inst, payload->ccc.data,
						      payload->ccc.data_len, false);
		if (xfered_len < 0) {
			LOG_ERR("CCC[0x%02x] %s command payload error (%d)", payload->ccc.id,
				i3c_ccc_is_payload_broadcast(payload) ? "broadcast" : "direct",
				ret);
			ret = xfered_len;

			goto out_ccc_stop;
		}

		/* Write back the transferred bytes */
		payload->ccc.num_xfer = xfered_len;
	}

	/* Wait message transfer complete */
	if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_COMPLETE), NPCX_I3C_CHK_TIMEOUT,
		     NULL) == false) {
		goto out_ccc_stop;
	}

	inst->MSTATUS = BIT(NPCX_I3C_MSTATUS_COMPLETE); /* W1C */

	/* For direct CCC */
	if (!i3c_ccc_is_payload_broadcast(payload)) {
		/*
		 * If there are payload(s) for each target,
		 * RESTART and then send payload for each target.
		 */
		for (int idx = 0; idx < payload->targets.num_targets; idx++) {
			struct i3c_ccc_target_payload *tgt_payload =
				&payload->targets.payloads[idx];

			bool is_read = (tgt_payload->rnw == 1U);

			xfered_len = npcx_i3c_do_one_xfer(
				inst, tgt_payload->addr, NPCX_I3C_MCTRL_TYPE_I3C, tgt_payload->data,
				tgt_payload->data_len, is_read, true, false, false);
			if (xfered_len < 0) {
				LOG_ERR("CCC[0x%02x] target payload error (%d)", payload->ccc.id,
					ret);
				ret = xfered_len;

				goto out_ccc_stop;
			}

			/* Write back the total number of bytes transferred */
			tgt_payload->num_xfer = xfered_len;
		}
	}

out_ccc_stop:
	npcx_i3c_request_emit_stop(inst);

	npcx_i3c_interrupt_enable(inst, intmask);

	npcx_i3c_mutex_unlock(dev);

	return ret;
}

#ifdef CONFIG_I3C_USE_IBI
/*
 * brief  Callback to service target initiated IBIs in workqueue.
 *
 * param[in] work  Pointer to k_work item.
 */
static void npcx_i3c_ibi_work(struct k_work *work)
{
	uint8_t payload[CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE];
	size_t payload_sz = 0;

	struct i3c_ibi_work *i3c_ibi_work = CONTAINER_OF(work, struct i3c_ibi_work, work);
	const struct device *dev = i3c_ibi_work->controller;
	const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	struct i3c_dev_attached_list *dev_list = &data->common.attached_dev;
	struct i3c_reg *inst = config->base;
	struct i3c_device_desc *target = NULL;
	uint32_t ibitype, ibiaddr;
	int ret;

	k_sem_take(&data->ibi_lock_sem, K_FOREVER);

	if (npcx_i3c_state_get(inst) != MSTATUS_STATE_TGTREQ) {
		LOG_DBG("IBI work %p running not because of IBI", work);
		LOG_DBG("MSTATUS 0x%08x MERRWARN 0x%08x", inst->MSTATUS, inst->MERRWARN);

		npcx_i3c_request_emit_stop(inst);

		goto out_ibi_work;
	};

	/* Use auto IBI to service the IBI */
	npcx_i3c_request_auto_ibi(inst);

	/* Wait for target to win address arbitration (ibitype and ibiaddr) */
	if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_IBIWON), NPCX_I3C_CHK_TIMEOUT,
		     NULL) == false) {
		LOG_ERR("IBI work, IBIWON timeout");

		goto out_ibi_work;
	}

	ibitype = GET_FIELD(inst->MSTATUS, NPCX_I3C_MSTATUS_IBITYPE);
	ibiaddr = GET_FIELD(inst->MSTATUS, NPCX_I3C_MSTATUS_IBIADDR);

	/*
	 * Wait for COMPLETE bit to be set to indicate auto IBI
	 * has finished for hot-join and controller role request.
	 * For target interrupts, the IBI payload may be longer
	 * than the RX FIFO so we won't get the COMPLETE bit set
	 * at the first round of data read. So checking of
	 * COMPLETE bit is deferred to the reading.
	 */
	switch (ibitype) {
	case MSTATUS_IBITYPE_HJ:
		__fallthrough;

	case MSTATUS_IBITYPE_CR:
		if (WAIT_FOR(IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_COMPLETE),
			     NPCX_I3C_CHK_TIMEOUT, NULL) == false) {
			LOG_ERR("Timeout waiting for COMPLETE");

			npcx_i3c_request_emit_stop(inst);

			goto out_ibi_work;
		}
		break;

	default:
		break;
	};

	switch (ibitype) {
	case MSTATUS_IBITYPE_IBI:
		target = i3c_dev_list_i3c_addr_find(dev_list, (uint8_t)ibiaddr);
		if (target != NULL) {
			ret = npcx_i3c_xfer_read_fifo(inst, &payload[0], sizeof(payload));
			if (ret >= 0) {
				payload_sz = (size_t)ret;
			} else {
				LOG_ERR("Error reading IBI payload");

				npcx_i3c_request_emit_stop(inst);

				goto out_ibi_work;
			}
		} else {
			/* NACK IBI coming from unknown device */
			npcx_i3c_ibi_respond_nack(inst);
		}
		break;
	case MSTATUS_IBITYPE_HJ:
		npcx_i3c_ibi_respond_ack(inst);
		npcx_i3c_request_emit_stop(inst);
		break;
	case MSTATUS_IBITYPE_CR:
		LOG_DBG("Controller role handoff not supported");
		npcx_i3c_ibi_respond_nack(inst);
		break;
	default:
		break;
	}

	if (npcx_i3c_has_error(inst)) {
		/*
		 * If the controller detects any errors, simply
		 * emit a STOP to abort the IBI. The target will
		 * raise IBI again if so desired.
		 */
		npcx_i3c_request_emit_stop(inst);

		goto out_ibi_work;
	}

	switch (ibitype) {
	case MSTATUS_IBITYPE_IBI:
		if (target != NULL) {
			if (i3c_ibi_work_enqueue_target_irq(target, &payload[0], payload_sz) != 0) {
				LOG_ERR("Error enqueue IBI IRQ work");
			}
		}

		/* Finishing the IBI transaction */
		npcx_i3c_request_emit_stop(inst);
		break;
	case MSTATUS_IBITYPE_HJ:
		if (i3c_ibi_work_enqueue_hotjoin(dev) != 0) {
			LOG_ERR("Error enqueue IBI HJ work");
		}
		break;
	case MSTATUS_IBITYPE_CR:
		/* Not supported */
		break;
	default:
		break;
	}

out_ibi_work:
	npcx_i3c_xfer_reset(inst);

	k_sem_give(&data->ibi_lock_sem);

	/* Re-enable target initiated IBI interrupt. */
	inst->MINTSET = BIT(NPCX_I3C_MINTSET_TGTSTART);
}

/* Set local IBI information to IBIRULES register */
static void npcx_i3c_ibi_rules_setup(struct npcx_i3c_data *data, struct i3c_reg *inst)
{
	uint32_t ibi_rules;
	int idx;

	ibi_rules = 0;

	for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
		uint32_t addr_6bit;

		/* Extract the lower 6-bit of target address */
		addr_6bit = (uint32_t)data->ibi.addr[idx] & IBIRULES_ADDR_MSK;

		/* Shift into correct place */
		addr_6bit <<= idx * IBIRULES_ADDR_SHIFT;

		/* Put into the temporary IBI Rules register */
		ibi_rules |= addr_6bit;
	}

	if (!data->ibi.msb) {
		/* The MSB0 field is 1 if MSB is 0 */
		ibi_rules |= BIT(NPCX_I3C_IBIRULES_MSB0);
	}

	if (!data->ibi.has_mandatory_byte) {
		/* The NOBYTE field is 1 if there is no mandatory byte */
		ibi_rules |= BIT(NPCX_I3C_IBIRULES_NOBYTE);
	}

	/* Update the register */
	inst->IBIRULES = ibi_rules;

	LOG_DBG("MIBIRULES 0x%08x", ibi_rules);
}

int npcx_i3c_ibi_enable(const struct device *dev, struct i3c_device_desc *target)
{
	const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	struct i3c_reg *inst = config->base;
	struct i3c_ccc_events i3c_events;
	uint8_t idx;
	bool msb, has_mandatory_byte;
	int ret = 0;

	/* Check target IBI request capable */
	if (!i3c_device_is_ibi_capable(target)) {
		ret = -EINVAL;
		goto out;
	}

	if (data->ibi.num_addr >= ARRAY_SIZE(data->ibi.addr)) {
		/* No more free entries in the IBI Rules table */
		ret = -ENOMEM;
		goto out;
	}

	/* Check whether the selected target is already in the list */
	for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
		if (data->ibi.addr[idx] == target->dynamic_addr) {
			ret = -EINVAL;
			goto out;
		}
	}

	/* Disable controller interrupt while we configure IBI rules. */
	inst->MINTCLR = BIT(NPCX_I3C_MINTCLR_TGTSTART);

	LOG_DBG("IBI enabling for 0x%02x (BCR 0x%02x)", target->dynamic_addr, target->bcr);

	msb = (target->dynamic_addr & BIT(6)) == BIT(6); /* Check addess(7-bit) MSB enable */
	has_mandatory_byte = i3c_ibi_has_payload(target);

	/*
	 * If there are already addresses in the table, we must
	 * check if the incoming entry is compatible with
	 * the existing ones.
	 *
	 * All targets in the list should follow the same IBI rules.
	 */
	if (data->ibi.num_addr > 0) {
		/*
		 * 1. All devices in the table must all use mandatory
		 *    bytes, or do not.
		 *
		 * 2. Each address in entry only captures the lowest 6-bit.
		 *    The MSB (7th bit) is captured separated in another bit
		 *    in the register. So all addresses must have the same MSB.
		 */
		if ((has_mandatory_byte != data->ibi.has_mandatory_byte) ||
		    (msb != data->ibi.msb)) {
			ret = -EINVAL;
			goto out;
		}

		/* Find an empty address slot */
		for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
			if (data->ibi.addr[idx] == 0U) {
				break;
			}
		}
	} else {
		/*
		 * If the incoming address is the first in the table,
		 * it dictates future compatibilities.
		 */
		data->ibi.has_mandatory_byte = has_mandatory_byte;
		data->ibi.msb = msb;

		idx = 0;
	}

	data->ibi.addr[idx] = target->dynamic_addr;
	data->ibi.num_addr += 1U;

	npcx_i3c_ibi_rules_setup(data, inst);

	/* Enable target IBI event by ENEC command */
	i3c_events.events = I3C_CCC_EVT_INTR;
	ret = i3c_ccc_do_events_set(target, true, &i3c_events);
	if (ret != 0) {
		LOG_ERR("Error sending IBI ENEC for 0x%02x (%d)", target->dynamic_addr, ret);
	}

out:
	if (data->ibi.num_addr > 0U) {
		/*
		 * If there is more than 1 target in the list,
		 * enable controller to raise interrupt when a target
		 * initiates IBI.
		 */
		inst->MINTSET = BIT(NPCX_I3C_MINTSET_TGTSTART);
	}

	return ret;
}

int npcx_i3c_ibi_disable(const struct device *dev, struct i3c_device_desc *target)
{
	const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	struct i3c_reg *inst = config->base;
	struct i3c_ccc_events i3c_events;
	int ret = 0;
	int idx;

	if (!i3c_device_is_ibi_capable(target)) {
		ret = -EINVAL;
		goto out;
	}

	for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
		if (target->dynamic_addr == data->ibi.addr[idx]) {
			break;
		}
	}

	if (idx == ARRAY_SIZE(data->ibi.addr)) {
		/* Target is not in list of registered addresses. */
		ret = -ENODEV;
		goto out;
	}

	/* Disable controller interrupt while we configure IBI rules. */
	inst->MINTCLR = BIT(NPCX_I3C_MINTCLR_TGTSTART);

	/* Clear the ibi rule data */
	data->ibi.addr[idx] = 0U;
	data->ibi.num_addr -= 1U;

	/* Disable disable target IBI */
	i3c_events.events = I3C_CCC_EVT_INTR;
	ret = i3c_ccc_do_events_set(target, false, &i3c_events);
	if (ret != 0) {
		LOG_ERR("Error sending IBI DISEC for 0x%02x (%d)", target->dynamic_addr, ret);

		goto out;
	}

	npcx_i3c_ibi_rules_setup(data, inst);

out:
	if (data->ibi.num_addr > 0U) {
		/*
		 * Enable controller to raise interrupt when a target
		 * initiates IBI.
		 */
		inst->MINTSET = BIT(NPCX_I3C_MINTSET_TGTSTART);
	}

	return ret;
}

#endif /* CONFIG_I3C_USE_IBI */

static void npcx_i3c_enable_target_interrupt(const struct device *dev, bool enable) 
{
    const struct npcx_i3c_config *config = dev->config;
	struct i3c_reg *inst = config->base;

    /* disbale the target interrupt events */
    inst->INTCLR = BIT(NPCX_I3C_INTCLR_START) | BIT(NPCX_I3C_INTCLR_MATCHED) | BIT(NPCX_I3C_INTCLR_STOP) |
                   BIT(NPCX_I3C_INTCLR_RXPEND) | BIT(NPCX_I3C_INTCLR_TXNOTFULL) | BIT(NPCX_I3C_INTCLR_DACHG) |
                   BIT(NPCX_I3C_INTCLR_CCC) | BIT(NPCX_I3C_INTCLR_ERRWARN) | BIT(NPCX_I3C_INTCLR_HDRMATCH) |
                   BIT(NPCX_I3C_INTCLR_CHANDLED) | BIT(NPCX_I3C_INTCLR_EVENT) | BIT(NPCX_I3C_INTCLR_TGTRST);

    /* clear the target interrupt status */
    inst->STATUS = inst->STATUS;

    /* enable the target interrupt events */
    if(enable == true)
    {
        inst->INTSET = BIT(NPCX_I3C_INTSET_START) | BIT(NPCX_I3C_INTSET_MATCHED)| BIT(NPCX_I3C_INTSET_STOP) | BIT(NPCX_I3C_INTSET_DACHG) | BIT(NPCX_I3C_INTSET_RXPEND) |
                       BIT(NPCX_I3C_INTSET_CCC) | BIT(NPCX_I3C_INTSET_ERRWARN) | BIT(NPCX_I3C_INTSET_HDRMATCH) | BIT(NPCX_I3C_INTSET_CHANDLED);
    }
}

static int npcx_i3c_target_config(const struct device *dev)
{
    const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	struct i3c_reg *inst = config->base;

    k_sem_init(&data->target_lock_sem, 1, 1);
    k_sem_init(&data->target_event_sem, 1, 1);

	npcx_i3c_target_sel(config->instance_id, true);

    inst->PARTNO = config->part_id;
    SET_FIELD(inst->VENDORID, NPCX_I3C_VENDORID_VID, (uint16_t) (config->vendor_id));
    SET_FIELD(inst->IDEXT, NPCX_I3C_IDEXT_DCR, config->dcr);
    SET_FIELD(inst->IDEXT, NPCX_I3C_IDEXT_BCR, config->bcr);
    SET_FIELD(inst->CONFIG, NPCX_I3C_CONFIG_SADDR, config->static_address);
    SET_FIELD(inst->CONFIG, NPCX_I3C_CONFIG_HDRCMD, false);
    //SET_FIELD(inst->MAXLIMITS, NPCX_I3C_MAXLIMITS_MAXRD, 4096 - 1);
    //SET_FIELD(inst->MAXLIMITS, NPCX_I3C_MAXLIMITS_MAXWR, 4096 - 1);

    inst->CONFIG &= ~BIT(NPCX_I3C_CONFIG_IDRAND);
    //inst->CONFIG |= BIT(NPCX_I3C_CONFIG_MATCHSS);
    inst->CONFIG |= BIT(NPCX_I3C_CONFIG_TGTENA);

    /* disbale the target interrupt events */
    /* enable the target interrupt events */
    npcx_i3c_enable_target_interrupt(dev, true); 

	/* Configure interrupt */
	config->irq_config_func(dev);

	return 0;
}

static int npcx_i3c_target_ibi_raise(const struct device *dev, struct i3c_ibi *request)
{
	const struct npcx_i3c_config *config = dev->config;
	struct i3c_reg *inst = config->base;
	struct npcx_i3c_data *data = dev->data;
    int index;

    /* the request or the payload were not specific */
	if((request == NULL) || ((request->payload_len) && (request->payload == NULL)))
    {
		return -EINVAL;
	}

    /* the I3C was not in target mode or the bus is in HDR mode now */
    if((!IS_BIT_SET(inst->CONFIG, NPCX_I3C_CONFIG_TGTENA)) || 
       (IS_BIT_SET(inst->STATUS, NPCX_I3C_STATUS_STHDR)))
    {
        return -EINVAL;
    }

	switch (request->ibi_type) {
        case I3C_IBI_TARGET_INTR:
            if(IS_BIT_SET(inst->STATUS, NPCX_I3C_STATUS_IBIDIS))
            {
                return -ENOTSUP;
            }

            k_sem_take(&data->target_event_sem, K_FOREVER);

            if(request->payload_len) 
            {
                SET_FIELD(inst->CTRL, NPCX_I3C_CTRL_IBIDATA, request->payload[0]);
                
                if(request->payload_len > 1)
                {
                    if(request->payload_len <= 32)
                    {
                        SET_FIELD(inst->IBIEXT1, NPCX_I3C_IBIEXT1_CNT, request->payload_len - 1);
                        for(index = 1; index < (request->payload_len - 2); index++)
                        {
                            inst->WDATAB = request->payload[index];
                        }

                        inst->WDATABE = request->payload[index];
                    }
                    else
                    {
                        /* transfer data from MDMA */
                    }

                    inst->CTRL |= BIT(NPCX_I3C_CTRL_EXTDATA);
                }
            }

            SET_FIELD(inst->CTRL, NPCX_I3C_CTRL_EVENT, 1);
            break;

        case I3C_IBI_CONTROLLER_ROLE_REQUEST:
            if(IS_BIT_SET(inst->STATUS, NPCX_I3C_STATUS_MRDIS))
            {
                return -ENOTSUP;
            }

			/* the bus controller request was generate only a target with controller mode capabilities mode */
            if(GET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_CTRENA) != MCONFIG_CTRENA_CAPABLE)
            {
                return -ENOTSUP;
            }

            k_sem_take(&data->target_event_sem, K_FOREVER);
            SET_FIELD(inst->CTRL, NPCX_I3C_CTRL_EVENT, 2);
            break;

        case I3C_IBI_HOTJOIN:
            if(IS_BIT_SET(inst->STATUS, NPCX_I3C_STATUS_HJDIS))
            {
                return -ENOTSUP;
            }

            k_sem_take(&data->target_event_sem, K_FOREVER);
            inst->CONFIG &= ~BIT(NPCX_I3C_CONFIG_TGTENA);
			SET_FIELD(inst->CTRL, NPCX_I3C_CTRL_IBIDATA, 0x00);
			inst->CTRL &= ~BIT(NPCX_I3C_CTRL_EXTDATA);
            SET_FIELD(inst->CTRL, NPCX_I3C_CTRL_EVENT, 3);
            inst->CONFIG |= BIT(NPCX_I3C_CONFIG_TGTENA);
            break;

        default:
            return -EINVAL;
	}

    return 0;
}

static int npcx_i3c_target_tx_write(const struct device *dev, uint8_t *buf, uint16_t len)
{
	const struct npcx_i3c_config *config = dev->config;
	struct i3c_reg *inst = config->base;
    //struct npcx_i3c_data *data = dev->data;
    int index = 0;

    /* check is in target mode */
    if(!IS_BIT_SET(inst->CONFIG, NPCX_I3C_CONFIG_TGTENA)) {
        return -EACCES;
    }

    if(buf != NULL)
    {
        for(index = 0; index < len; index++)
        {
            if(!IS_BIT_SET(inst->DATACTRL, NPCX_I3C_DATACTRL_TXFULL)) {
                if(index == (len - 1)) {
                    inst->WDATABE = buf[index];
                }
                else {
                    inst->WDATAB = buf[index];
                }
            }
        }
    }

	/* return total bytes written */
	return index;
}

static int npcx_i3c_target_register(const struct device *dev, struct i3c_target_config *cfg)
{
    //const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	//struct i3c_reg *inst = config->base;

    //LOG_ERR("npcx_i3c_target_register: %x\n", cfg);

	data->target_config = cfg;

    /* enable the target interrupt events */
    //inst->CONFIG |= BIT(NPCX_I3C_CONFIG_TGTENA);
    //npcx_i3c_enable_target_interrupt(dev, true); 

	return 0;
}

static int npcx_i3c_target_unregister(const struct device *dev, struct i3c_target_config *cfg)
{
    const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	struct i3c_reg *inst = config->base;

    /* disbale the target interrupt events */
    npcx_i3c_enable_target_interrupt(dev, false);
    inst->CONFIG &= ~BIT(NPCX_I3C_CONFIG_TGTENA);

	data->target_config = NULL;

	return 0;
}

static void npcx_i3c_target_isr(const struct device *dev)
{
    struct npcx_i3c_data *data = dev->data;
    struct i3c_target_config *target_config = data->target_config;
	const struct npcx_i3c_config *config = dev->config;
	struct i3c_reg *inst = config->base;
	const struct i3c_target_callbacks *target_cb = data->target_config->callbacks;
    uint32_t val;

	while(inst->INTMASKED) {
		if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_DACHG)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_DACHG);
			inst->STATUS = BIT(NPCX_I3C_STATUS_DACHG);

		    if(IS_BIT_SET(inst->DYNADDR, NPCX_I3C_DYNADDR_DAVALID))
	    	{
		        if(target_config != NULL) {
	            	target_config->address = GET_FIELD(inst->DYNADDR, NPCX_I3C_DYNADDR_DADDR);
	        	}
	    	}

			inst->INTSET = BIT(NPCX_I3C_INTSET_DACHG);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_START)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_START);
			inst->STATUS = BIT(NPCX_I3C_STATUS_START);

		    /* check the IBI was has finished or not -- nacked form host */
	    	/* config the MDMA for Tx and Rx transfer */
			inst->INTSET = BIT(NPCX_I3C_INTSET_START);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_MATCHED)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_MATCHED);
			inst->STATUS = BIT(NPCX_I3C_STATUS_MATCHED);

		    /* The current bus request is an SDR mode read from this target device */
		    if(IS_BIT_SET(inst->STATUS, NPCX_I3C_STATUS_STREQRD))
		    {
	        	if((target_cb != NULL) && (target_cb->read_requested_cb != NULL)) {
	            	target_cb->read_requested_cb(data->target_config, (uint8_t *) &val);
	        	}

	        	/* enable the Tx interrupt */
	     		//inst->INTCLR = BIT(NPCX_I3C_INTCLR_TXNOTFULL);
	    		//inst->STATUS = BIT(NPCX_I3C_STATUS_TXNOTFULL);
	    		//inst->INTSET = BIT(NPCX_I3C_INTSET_TXNOTFULL);
	    	}
	    	/* The current bus request is an SDR mode write to this target device */
	    	else if(IS_BIT_SET(inst->STATUS, NPCX_I3C_STATUS_STREQWR))
	    	{
	        	if ((target_cb != NULL) && (target_cb->write_requested_cb != NULL)) {
	            	target_cb->write_requested_cb(data->target_config);
	        	}
	    	}

			inst->INTSET = BIT(NPCX_I3C_INTSET_MATCHED);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_RXPEND)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_RXPEND);
			inst->STATUS = BIT(NPCX_I3C_STATUS_RXPEND);

		    while(!IS_BIT_SET(inst->DATACTRL, NPCX_I3C_DATACTRL_RXEMPTY)) {
		        val = inst->RDATAB;
	        	if ((target_cb != NULL) && (target_cb->write_received_cb != NULL)) {
		            target_cb->write_received_cb(data->target_config, (uint8_t) val);
	        	}
	    	}

			inst->INTSET = BIT(NPCX_I3C_INTSET_RXPEND);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_TXNOTFULL)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_TXNOTFULL);
			inst->STATUS = BIT(NPCX_I3C_STATUS_TXNOTFULL);

		    while(!IS_BIT_SET(inst->DATACTRL, NPCX_I3C_DATACTRL_TXFULL)) {
		        if ((target_cb != NULL) && (target_cb->read_processed_cb != NULL)) {
	            	target_cb->read_processed_cb(data->target_config, (uint8_t *) &val);
	            	inst->WDATAB = val;
	        	}
			}

			//inst->INTSET = BIT(NPCX_I3C_INTSET_TXNOTFULL);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_CCC)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_CCC);
			inst->STATUS = BIT(NPCX_I3C_STATUS_CCC);

			inst->INTSET = BIT(NPCX_I3C_INTSET_CCC);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_ERRWARN)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_ERRWARN);
			inst->STATUS = BIT(NPCX_I3C_STATUS_ERRWARN);

		    inst->ERRWARN = inst->ERRWARN;

			inst->INTSET = BIT(NPCX_I3C_INTSET_ERRWARN);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_HDRMATCH)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_HDRMATCH);
			inst->STATUS = BIT(NPCX_I3C_STATUS_HDRMATCH);

			inst->INTSET = BIT(NPCX_I3C_INTSET_HDRMATCH);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_CHANDLED)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_CHANDLED);
			inst->STATUS = BIT(NPCX_I3C_STATUS_CHANDLED);

			inst->INTSET = BIT(NPCX_I3C_INTSET_CHANDLED);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_EVENT)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_EVENT);
			inst->STATUS = BIT(NPCX_I3C_STATUS_EVENT);

		    if(0x03 == GET_FIELD(inst->STATUS, NPCX_I3C_STATUS_EVDET))
		    {
	        	k_sem_give(&data->target_event_sem);
	    	}

			inst->INTSET = BIT(NPCX_I3C_INTSET_EVENT);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_STOP)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_STOP);
			inst->STATUS = BIT(NPCX_I3C_STATUS_STOP);

		    /* disable the Tx interrupt */
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_TXNOTFULL);
			inst->STATUS = BIT(NPCX_I3C_STATUS_TXNOTFULL);

	    	/* flush the Tx/Rx FIFO */
	    	inst->DATACTRL |= BIT(NPCX_I3C_DATACTRL_FLUSHTB) | BIT(NPCX_I3C_DATACTRL_FLUSHFB);

	    	/* Notify upper layer a STOP condition received */
	    	if ((target_cb != NULL) && (target_cb->stop_cb != NULL)) {
		        target_cb->stop_cb(data->target_config);
	    	}

			inst->INTSET = BIT(NPCX_I3C_INTSET_STOP);
		}
		else if (IS_BIT_SET(inst->INTMASKED, NPCX_I3C_INTMASKED_TGTRST)) {
			inst->INTCLR = BIT(NPCX_I3C_INTCLR_TGTRST);
			inst->STATUS = BIT(NPCX_I3C_STATUS_TGTRST);

			inst->INTSET = BIT(NPCX_I3C_INTSET_TGTRST);
		}
	}
}

static void npcx_i3c_isr(const struct device *dev)
{
	const struct npcx_i3c_config *config = dev->config;
	struct i3c_reg *inst = config->base;

    //if((IS_BIT_SET(inst->CONFIG, NPCX_I3C_CONFIG_TGTENA) && inst->INTMASKED) {
	if(inst->INTMASKED) {
        npcx_i3c_target_isr(dev);
    }

	//if(((GET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_CTRENA) == 0x01) ||
	//   (GET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_CTRENA) == 0x02))  && 
	//   inst->MINTMASKED) {
	if(inst->MINTMASKED) {
	#ifdef CONFIG_I3C_USE_IBI
		LOG_DBG("I3C IBI_ISR");

		/* Target start detected */
		if (IS_BIT_SET(inst->MSTATUS, NPCX_I3C_MSTATUS_TGTSTART)) {
			/* Disable further target initiated IBI interrupt */
			inst->MINTCLR = BIT(NPCX_I3C_MINTCLR_TGTSTART);

			/* Handle IBI in workqueue */
			i3c_ibi_work_enqueue_cb(dev, npcx_i3c_ibi_work);
		}
	#endif /* CONFIG_I3C_USE_IBI */
	}
}

static int npcx_i3c_get_scl_config(struct npcx_i3c_freq_cfg *cfg, uint32_t i3c_src_clk,
				   uint32_t pp_baudrate_hz, uint32_t od_baudrate_hz)
{
	uint32_t div, freq;
	uint32_t ppbaud, odbaud;
	uint32_t pplow_ns, odlow_ns;

	if (cfg == NULL) {
		LOG_ERR("Freq config NULL");
		return -EINVAL;
	}

	if ((pp_baudrate_hz > I3C_SCL_PP_FREQ_MAX) || (od_baudrate_hz > I3C_SCL_OD_FREQ_MAX)) {
		LOG_ERR("I3C PP_SCL should within 12.5 Mhz, input: %d", pp_baudrate_hz);
		LOG_ERR("I3C OD_SCL should within 4.17 Mhz, input: %d", od_baudrate_hz);
		return -EINVAL;
	}

	/* Fixed PPLOW = 0 to achieve 50% duty cycle */
	/* pp_freq = ((f_mclkd / 2) / (PPBAUD+1)) */
	freq = i3c_src_clk / 2UL;

	div = freq / pp_baudrate_hz;
	div = (div == 0UL) ? 1UL : div;
	if (freq / div > pp_baudrate_hz) {
		div++;
	}

	if (div > PPBAUD_DIV_MAX) {
		LOG_ERR("PPBAUD out of range");
		return -EINVAL;
	}

	ppbaud = div - 1UL;
	freq /= div;

	/* Check PP low period in spec (should be the same as PPHIGH) */
	pplow_ns = (uint32_t)(NSEC_PER_SEC / (2UL * freq));
	if (pplow_ns < I3C_BUS_TLOW_PP_MIN_NS) {
		LOG_ERR("PPLOW ns out of spec");
		return -EINVAL;
	}

	/* Fixed odhpp = 1 configuration */
	/* odFreq = (2*freq) / (ODBAUD + 2), 1 <= ODBAUD <= 255 */
	div = (2UL * freq) / od_baudrate_hz;
	div = div < 2UL ? 2UL : div;
	if ((2UL * freq / div) > od_baudrate_hz) {
		div++;
	}

	odbaud = div - 2UL;
	freq = (2UL * freq) / div; /* For I2C usage in the future */

	/* Check OD low period in spec */
	odlow_ns = (odbaud + 1UL) * pplow_ns;
	if (odlow_ns < I3C_BUS_TLOW_OD_MIN_NS) {
		LOG_ERR("ODBAUD ns out of spec");
		return -EINVAL;
	}

	cfg->pplow = 0;
	cfg->odhpp = 1;
	cfg->ppbaud = ppbaud;
	cfg->odbaud = odbaud;

	return 0;
}

static int npcx_i3c_freq_init(const struct device *dev)
{
	const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	struct i3c_reg *inst = config->base;
	const struct device *const clk_dev = config->clock_dev;
	struct i3c_config_controller *ctrl_config = &data->common.ctrl_config;
	uint32_t scl_pp = ctrl_config->scl.i3c;
	uint32_t scl_od = data->clocks.i3c_od_scl_hz;
	struct npcx_i3c_freq_cfg freq_cfg;
	uint32_t i3c_rate;
	int ret = 0;

	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)&config->clock_subsys,
				     &i3c_rate);
	if (ret != 0x0) {
		LOG_ERR("Get I3C source clock fail %d", ret);
		return -EINVAL;
	}

	LOG_DBG("MCLKD: %d", i3c_rate);
	LOG_DBG("SCL_PP_FEQ MAX: %d", I3C_SCL_PP_FREQ_MAX);
	LOG_DBG("SCL_OD_FEQ MAX: %d", I3C_SCL_OD_FREQ_MAX);
	LOG_DBG("scl_pp: %d", scl_pp);
	LOG_DBG("scl_od: %d", scl_od);
	LOG_DBG("hdr: %d", ctrl_config->supported_hdr);

	/* MCLKD = MCLK / I3C_DIV(1 or 2)
	 * MCLKD must between 40 mhz to 50 mhz.
	 */
	if (i3c_rate == MHZ(45)) {
		/* Set default I3C_SCL configuration */
		freq_cfg = npcx_def_speed_cfg[NPCX_I3C_BUS_SPEED_45MHZ];
	} else {
		LOG_ERR("Unsupported MCLKD freq for %s.", dev->name);
		return -EINVAL;
	}

	ret = npcx_i3c_get_scl_config(&freq_cfg, i3c_rate, scl_pp, scl_od);
	if (ret != 0x0) {
		LOG_ERR("Adjust I3C frequency fail");
		return -EINVAL;
	}

	/* Apply SCL_PP and SCL_OD */
	SET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_PPBAUD, freq_cfg.ppbaud);
	SET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_PPLOW, freq_cfg.pplow);
	SET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_ODBAUD, freq_cfg.odbaud);
	if (freq_cfg.odhpp != 0) {
		inst->MCONFIG |= BIT(NPCX_I3C_MCONFIG_ODHPP);
	} else {
		inst->MCONFIG &= ~BIT(NPCX_I3C_MCONFIG_ODHPP);
	}

	LOG_DBG("ppbaud: %d", GET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_PPBAUD));
	LOG_DBG("odbaud: %d", GET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_ODBAUD));
	LOG_DBG("pplow: %d", GET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_PPLOW));
	LOG_DBG("odhpp: %d", IS_BIT_SET(inst->MCONFIG, NPCX_I3C_MCONFIG_ODHPP));

	return 0;
}

static int npcx_i3c_cntlr_init(const struct device *dev)
{
	const struct npcx_i3c_config *config = dev->config;
	struct i3c_reg *inst = config->base;
	const struct device *const clk_dev = config->clock_dev;
	uint32_t apb4_rate;
	uint8_t bamatch;
	int ret;

	/* Reset I3c module */
	reset_line_toggle_dt(&config->reset);

	/* Disable all interrupts (W1C) */
	inst->MINTCLR |= BIT(NPCX_I3C_MINTCLR_TGTSTART) | BIT(NPCX_I3C_MINTCLR_MCTRLDONE) |
			 BIT(NPCX_I3C_MINTCLR_COMPLETE) | BIT(NPCX_I3C_MINTCLR_RXPEND) |
			 BIT(NPCX_I3C_MINTCLR_TXNOTFULL) | BIT(NPCX_I3C_MINTCLR_IBIWON) |
			 BIT(NPCX_I3C_MINTCLR_ERRWARN) | BIT(NPCX_I3C_MINTCLR_NOWCNTLR);

	/* Initial baudrate. PPLOW=1, PPBAUD, ODHPP=1, ODBAUD */
	if (npcx_i3c_freq_init(dev) != 0x0) {
		return -EINVAL;
	}

	/* Enable main controller mode */
	SET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_CTRENA, MCONFIG_CTRENA_ON);
	/* Enable external high-keeper */
	SET_FIELD(inst->MCONFIG, NPCX_I3C_MCONFIG_HKEEP, MCONFIG_HKEEP_EXT_SDA_SCL);
	/* Enable open-drain stop */
	inst->MCONFIG |= BIT(NPCX_I3C_MCONFIG_ODSTOP);
	/* Enable timeout */
	inst->MCONFIG &= ~BIT(NPCX_I3C_MCONFIG_DISTO);
	/* Flush tx and tx FIFO buffer */
	npcx_i3c_fifo_flush(inst);

	/* Set bus available match value in target register */
	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)&config->ref_clk_subsys,
				     &apb4_rate);
	LOG_DBG("APB4_CLK: %d", apb4_rate);

	if (ret != 0x0) {
		LOG_ERR("Get APb4 source clock fail %d", ret);
		return -EINVAL;
	}

	bamatch = DIV_ROUND_UP(apb4_rate, MHZ(1));
	bamatch = DIV_ROUND_UP(bamatch, BAMATCH_DIV);
	LOG_DBG("BAMATCH: %d", bamatch);

	SET_FIELD(inst->CONFIG, NPCX_I3C_CONFIG_BAMATCH, bamatch);

	return 0;
}

static int npcx_i3c_configure(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct npcx_i3c_data *dev_data = dev->data;
	struct i3c_config_controller *cntlr_cfg = config;
	int ret = 0;

	if (type != I3C_CONFIG_CONTROLLER) {
		return -EINVAL;
	}

	/*
	 * Check for valid configuration parameters.
	 * Currently, must be the primary controller.
	 */
	if ((cntlr_cfg->is_secondary) || (cntlr_cfg->scl.i3c == 0U)) {
		return -EINVAL;
	}

	/* Save requested config to dev */
	(void)memcpy(&dev_data->common.ctrl_config, cntlr_cfg, sizeof(*cntlr_cfg));

	/* Controller init */
	ret = npcx_i3c_cntlr_init(dev);

	return ret;
}

static int npcx_i3c_config_get(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct npcx_i3c_data *data = dev->data;
	int ret = 0;

	if ((type != I3C_CONFIG_CONTROLLER) || (config == NULL)) {
		ret = -EINVAL;
		goto out_configure;
	}

	(void)memcpy(config, &data->common.ctrl_config, sizeof(data->common.ctrl_config));

out_configure:
	return ret;
}

static int npcx_i3c_init(const struct device *dev)
{
	const struct npcx_i3c_config *config = dev->config;
	struct npcx_i3c_data *data = dev->data;
	struct i3c_config_controller *ctrl_config = &data->common.ctrl_config;
	const struct device *const clk_dev = config->clock_dev;

	int ret = 0;

	LOG_DBG("I3C driver init");

	/* Check clock device ready */
	if (!device_is_ready(clk_dev)) {
		LOG_ERR("%s Clk device not ready", clk_dev->name);
		return -ENODEV;
	}

	/* Set I3C_PD operational */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t)&config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Turn on I3C clock fail %d", ret);
		goto err_out;
	}

	/* Apply pin-muxing */
	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Apply pinctrl fail %d", ret);
		goto err_out;
	}

	k_sem_init(&data->lock_sem, 1, 1);
	k_sem_init(&data->sync_sem, 0, 1);
	k_sem_init(&data->ibi_lock_sem, 1, 1);

	/* Currently can only act as primary controller. */
	ctrl_config->is_secondary = config->secondary_mode;

	/* HDR mode not supported at the moment. */
	ctrl_config->supported_hdr = 0U;

	if(config->target_mode == false) {
		ret = i3c_addr_slots_init(dev);
		if (ret != 0) {
			LOG_ERR("Addr slots init fail %d", ret);
			goto err_out;
		}

		ret = npcx_i3c_configure(dev, I3C_CONFIG_CONTROLLER, ctrl_config);
		if (ret != 0) {
			LOG_ERR("Apply i3c_configure() fail %d", ret);
			goto err_out;
		}

		/* Just in case the bus is not in idle. */
		ret = npcx_i3c_recover_bus(dev);
		if (ret != 0) {
			ret = -EIO;
			LOG_ERR("Apply i3c_recover_bus() fail %d", ret);
			goto err_out;
		}
		
		/* Configure interrupt */
		config->irq_config_func(dev);

		/* Perform bus initialization */
		ret = i3c_bus_init(dev, &config->common.dev_list);
	}
	else {
		npcx_i3c_target_config(dev);
	}

err_out:

	return ret;
}

static const struct i3c_driver_api npcx_i3c_driver_api = {
	.configure = npcx_i3c_configure,
	.config_get = npcx_i3c_config_get,

	.recover_bus = npcx_i3c_recover_bus,

	.do_daa = npcx_i3c_do_daa,
	.do_ccc = npcx_i3c_do_ccc,

	.i3c_device_find = npcx_i3c_device_find,

	.i3c_xfers = npcx_i3c_transfer,

	.target_tx_write = npcx_i3c_target_tx_write,
	.target_register = npcx_i3c_target_register,
	.target_unregister = npcx_i3c_target_unregister,

#ifdef CONFIG_I3C_USE_IBI
	.ibi_enable = npcx_i3c_ibi_enable,
	.ibi_disable = npcx_i3c_ibi_disable,

	.ibi_raise = npcx_i3c_target_ibi_raise,
#endif
};

#define I3C_NPCX_DEVICE(id)                                                                        \
	PINCTRL_DT_INST_DEFINE(id);                                                                \
	static void npcx_i3c_config_func_##id(const struct device *dev);                           \
	static struct i3c_device_desc npcx_i3c_device_array_##id[] = I3C_DEVICE_ARRAY_DT_INST(id); \
	static struct i3c_i2c_device_desc npcx_i3c_i2c_device_array_##id[] =                       \
		I3C_I2C_DEVICE_ARRAY_DT_INST(id);                                                  \
	static const struct npcx_i3c_config npcx_i3c_config_##id = {                               \
		.base = (struct i3c_reg *)DT_INST_REG_ADDR(id),                                    \
		.clock_dev = DEVICE_DT_GET(NPCX_CLK_CTRL_NODE),                                    \
		.reset = RESET_DT_SPEC_INST_GET(id),                                               \
		.clock_subsys = NPCX_DT_CLK_CFG_ITEM_BY_NAME(id, mclkd),                           \
		.ref_clk_subsys = NPCX_DT_CLK_CFG_ITEM_BY_NAME(id, apb4),                          \
		.irq_config_func = npcx_i3c_config_func_##id,                                      \
		.common.dev_list.i3c = npcx_i3c_device_array_##id,                                 \
		.common.dev_list.num_i3c = ARRAY_SIZE(npcx_i3c_device_array_##id),                 \
		.common.dev_list.i2c = npcx_i3c_i2c_device_array_##id,                             \
		.common.dev_list.num_i2c = ARRAY_SIZE(npcx_i3c_i2c_device_array_##id),             \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),                                      \
		.disable_open_drain_high_pp =                                                      \
			DT_INST_PROP_OR(id, disable_open_drain_high_pp, false),                        \
		.target_mode = DT_INST_PROP_OR(id, slave, false),                                  \
		.secondary_mode = DT_INST_PROP_OR(id, secondary, false),                   \
		.instance_id = DT_INST_PROP_OR(id, instance_id, 0),                                \
		.mdma_channel = DT_INST_PROP_OR(id, mdma_channel, 0),                              \
		.static_address = DT_INST_PROP_OR(id, static_address, 0),                      \
		.part_id = DT_INST_PROP_OR(id, part_id, 0),                                        \
		.vendor_id = DT_INST_PROP_OR(id, vendor_id, 0),                                    \
		.bcr = DT_INST_PROP_OR(id, bcr, 0),                                                \
		.dcr = DT_INST_PROP_OR(id, dcr, 0),                                                \
	};                                                                                         \
	static struct npcx_i3c_data npcx_i3c_data_##id = {                                         \
		.common.ctrl_config.scl.i3c = DT_INST_PROP_OR(id, i3c_scl_hz, 0),                  \
		.common.ctrl_config.scl.i2c = DT_INST_PROP_OR(id, i2c_scl_hz, 0),                  \
		.clocks.i3c_od_scl_hz = DT_INST_PROP_OR(id, i3c_od_scl_hz, 0),                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(id, npcx_i3c_init, NULL, &npcx_i3c_data_##id, &npcx_i3c_config_##id, \
			      POST_KERNEL, CONFIG_I3C_CONTROLLER_INIT_PRIORITY,                    \
			      &npcx_i3c_driver_api);                                               \
	static void npcx_i3c_config_func_##id(const struct device *dev)                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(id), DT_INST_IRQ(id, priority), npcx_i3c_isr,             \
			    DEVICE_DT_INST_GET(id), 0);                                            \
		irq_enable(DT_INST_IRQN(id));                                                      \
	};

DT_INST_FOREACH_STATUS_OKAY(I3C_NPCX_DEVICE)
