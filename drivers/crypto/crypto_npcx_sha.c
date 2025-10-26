/*
 * Copyright (c) 2022 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcx_sha

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sha_npcx, CONFIG_CRYPTO_LOG_LEVEL);

#include "soc_ncl.h"

#define NPCX_HASH_CAPS_SUPPORT  (CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)
#define NPCX_SHA256_HANDLE_SIZE DT_INST_PROP(0, context_buffer_size)
#define NPCX_SHA_MAX_SESSION    1

/* The following table holds the function pointer for each SHA API in NPCX ROM. */
struct npcx_ncl_sha {
	/* Get the SHA context size required by SHA APIs. */
	uint32_t (*get_context_size)(void);
	/* Initial SHA context. */
	enum ncl_status (*init_context)(void *ctx);
	/* Finalize SHA context. */
	enum ncl_status (*finalize_context)(void *ctx);
	/* Initiate the SHA hardware module and setups needed parameters. */
	enum ncl_status (*init)(void *ctx);
	/*
	 * Prepare the context buffer for a SHA calculation -  by loading the
	 * initial SHA-256/384/512 parameters.
	 */
	enum ncl_status (*start)(void *ctx, enum ncl_sha_type type);
	/*
	 * Updates the SHA calculation with the additional data. When the
	 * function returns, the hardware and memory buffer shall be ready to
	 * accept new data * buffers for SHA calculation and changes to the data
	 * in data buffer should no longer effect the SHA calculation.
	 */
	enum ncl_status (*update)(void *ctx, const uint8_t *data, uint32_t Len);
	/* Return the SHA result (digest.) */
	enum ncl_status (*finish)(void *ctx, uint8_t *hashDigest);
	/* Perform a complete SHA calculation */
	enum ncl_status (*calc)(void *ctx, int module_num, enum ncl_sha_type type, const uint8_t *data,
				uint32_t Len, uint8_t *hashDigest);
	/* Power on/off the SHA module. */
	enum ncl_status (*power)(void *ctx, uint8_t enable);
	/* Reset the SHA hardware and terminate any in-progress operations. */
	enum ncl_status (*reset)(void *ctx);
	/*
	 * Prepare the context buffer for a HMAC calculation -  by loading the
	 * initial SHA-256/384/512 parameters.
	 */
	enum ncl_status (*hmac_start)(void *ctx, enum ncl_sha_type type, const struct ncl_sha_hmac_key *key,
				       uint8_t *padKey);
	/*
	 * Updates the HMAC calculation with the additional data. When the
	 * function returns, the hardware and memory buffer shall be ready to
	 * accept new data * buffers for HMAC calculation and changes to the data
	 * in data buffer should no longer effect the HMAC calculation.
	 */
	enum ncl_status (*hmac_update)(void *ctx, const uint8_t *data, uint32_t Len);
	/* Return the HMAC result */
	enum ncl_status (*hmac_finish)(void *ctx, uint8_t *padKey, uint8_t *hmac);
	/* Perform a complete HMAC calculation */
	enum ncl_status (*hmac_calc)(void *ctx, enum ncl_sha_type type,
			const struct ncl_sha_hmac_key *key, const uint8_t *data,
			uint32_t Len, uint8_t *hmac);
};

/* The start address of the SHA API table. */
#define NPCX_NCL_SHA ((const struct npcx_ncl_sha *)DT_INST_REG_ADDR(0))

struct npcx_sha_context {
	uint8_t handle[NPCX_SHA256_HANDLE_SIZE];
} __aligned(4);

struct npcx_sha_session {
	struct npcx_sha_context npcx_sha_ctx;
	enum hash_algo algo;
	bool in_use;
};

struct npcx_sha_session npcx_sessions[NPCX_SHA_MAX_SESSION];

static int npcx_get_unused_session_index(void)
{
	int i;

	for (i = 0; i < NPCX_SHA_MAX_SESSION; i++) {
		if (!npcx_sessions[i].in_use) {
			npcx_sessions[i].in_use = true;
			return i;
		}
	}

	return -1;
}
static int npcx_sha_compute(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	enum ncl_status ret;
	struct npcx_sha_session *npcx_session = ctx->drv_sessn_state;
	struct npcx_sha_context *npcx_ctx = &npcx_session->npcx_sha_ctx;
	enum ncl_sha_type sha_type;

	switch (npcx_session->algo) {
	case CRYPTO_HASH_ALGO_SHA256:
		sha_type = NCL_SHA_TYPE_2_256;
		break;
	case CRYPTO_HASH_ALGO_SHA384:
		sha_type = NCL_SHA_TYPE_2_384;
		break;
	case CRYPTO_HASH_ALGO_SHA512:
		sha_type = NCL_SHA_TYPE_2_512;
		break;
	default:
		LOG_ERR("Unexpected algo: %d", npcx_session->algo);
		return -EINVAL;
	}

	if (!ctx->started) {
		ret = NPCX_NCL_SHA->start(npcx_ctx->handle, sha_type);
		if (ret != NCL_STATUS_OK) {
			LOG_ERR("Could not compute the hash, err:%d", ret);
			return -EINVAL;
		}
		ctx->started = true;
	}

	if (pkt->in_len != 0) {
		ret = NPCX_NCL_SHA->update(npcx_ctx->handle, pkt->in_buf, pkt->in_len);
		if (ret != NCL_STATUS_OK) {
			LOG_ERR("Could not update the hash, err:%d", ret);
			ctx->started = false;
			return -EINVAL;
		}
	}

	if (finish) {
		ctx->started = false;
		ret = NPCX_NCL_SHA->finish(npcx_ctx->handle, pkt->out_buf);
		if (ret != NCL_STATUS_OK) {
			LOG_ERR("Could not compute the hash, err:%d", ret);
			return -EINVAL;
		}
	}

	return 0;
}

int npcx_hmac_compute(const struct device *dev,
			struct hash_ctx *ctx,
			const uint8_t *key,
			uint32_t key_len,
			const uint8_t *msg,
			uint8_t msg_len,
			uint8_t *hmac)
{
	struct npcx_sha_session *npcx_session = ctx->drv_sessn_state;
	struct npcx_sha_context *npcx_ctx = &npcx_session->npcx_sha_ctx;
	enum ncl_status ret;
	struct ncl_sha_hmac_key	hmac_key;
	enum ncl_sha_type sha_type;

	switch (npcx_session->algo) {
		case CRYPTO_HASH_ALGO_SHA256:
			sha_type = NCL_SHA_TYPE_2_256;
			break;
		case CRYPTO_HASH_ALGO_SHA384:
			sha_type = NCL_SHA_TYPE_2_384;
			break;
		case CRYPTO_HASH_ALGO_SHA512:
			sha_type = NCL_SHA_TYPE_2_512;
			break;
		default:
			LOG_ERR("Unexpected algo: %d", npcx_session->algo);
			return -EINVAL;
	}

	hmac_key.key = key;
	hmac_key.keyLen = key_len;

	ret = NPCX_NCL_SHA->hmac_calc(npcx_ctx->handle, sha_type,
								(const struct ncl_sha_hmac_key *) &hmac_key,
								msg, msg_len, hmac);
	if (ret != NCL_STATUS_OK) {
		LOG_ERR("Could not compute the hmac, err:%d", ret);
		return -EINVAL;
	}

	ret = NPCX_NCL_SHA->finalize_context(npcx_ctx->handle);
	if (ret != NCL_STATUS_OK) {
		LOG_ERR("Could not finalize the context, err:%d", ret);
		return -EINVAL;
	}

	return 0;
}

static int npcx_hash_session_setup(const struct device *dev, struct hash_ctx *ctx,
				   enum hash_algo algo)
{
	int ctx_idx;
	struct npcx_sha_context *npcx_ctx;
	enum ncl_status status;

	if (ctx->flags & ~(NPCX_HASH_CAPS_SUPPORT)) {
		LOG_ERR("Unsupported flag");
		return -EINVAL;
	}

	if ((algo != CRYPTO_HASH_ALGO_SHA256) && (algo != CRYPTO_HASH_ALGO_SHA384) &&
	    (algo != CRYPTO_HASH_ALGO_SHA512)) {
		LOG_ERR("Unsupported algo: %d", algo);
		return -EINVAL;
	}

	ctx_idx = npcx_get_unused_session_index();
	if (ctx_idx < 0) {
		LOG_ERR("No free session for now");
		return -ENOSPC;
	}

	npcx_sessions[ctx_idx].algo = algo;

	ctx->drv_sessn_state = &npcx_sessions[ctx_idx];
	ctx->started = false;
	ctx->hash_hndlr = npcx_sha_compute;

	npcx_ctx = &npcx_sessions[ctx_idx].npcx_sha_ctx;
	status = NPCX_NCL_SHA->init_context(npcx_ctx->handle);
	if (status != NCL_STATUS_OK) {
		LOG_ERR("fail to initial SHA context, err:%d", status);
		return -EINVAL;
	}

	status = NPCX_NCL_SHA->power(npcx_ctx->handle, 1);
	if (status != NCL_STATUS_OK) {
		LOG_ERR("fail to power on the SHA module, err:%d", status);
		return -ENOSR;
	}

	status = NPCX_NCL_SHA->init(npcx_ctx->handle);
	if (status != NCL_STATUS_OK) {
		LOG_ERR("fail to initiate the SHA hardware module, err:%d", status);
		return -ENOSR;
	}

	status = NPCX_NCL_SHA->reset(npcx_ctx->handle);
	if (status != NCL_STATUS_OK) {
		LOG_ERR("fail to reset the SHA hardware, err:%d", status);
		return -ENOSR;
	}

	return 0;
}

static int npcx_hash_session_free(const struct device *dev, struct hash_ctx *ctx)
{
	int ret = 0;
	enum ncl_status status;
	struct npcx_sha_session *npcx_session = ctx->drv_sessn_state;
	struct npcx_sha_context *npcx_ctx = &npcx_session->npcx_sha_ctx;

	status = NPCX_NCL_SHA->power(npcx_ctx->handle, 0);
	if (status != NCL_STATUS_OK) {
		LOG_ERR("fail to power off the SHA module, err:%d", status);
		ret = -ENOSR;
		goto exit;
	}

	status = NPCX_NCL_SHA->reset(npcx_ctx->handle);
	if (status != NCL_STATUS_OK) {
		LOG_ERR("fail to reset the SHA hardware, err:%d", status);
		ret =  -ENOSR;
		goto exit;
	}

	status = NPCX_NCL_SHA->finalize_context(npcx_ctx->handle);
	if (status != NCL_STATUS_OK) {
		LOG_ERR("fail to finalize SHA context, err:%d", status);
		ret = -ENOSR;
		goto exit;
	}

exit:
	npcx_session->in_use = false;
	return ret;
}

static int npcx_query_caps(const struct device *dev)
{
	return NPCX_HASH_CAPS_SUPPORT;
}

static int npcx_hash_init(const struct device *dev)
{
	uint32_t handle_size_required;

	handle_size_required = NPCX_NCL_SHA->get_context_size();
	if (handle_size_required != NPCX_SHA256_HANDLE_SIZE) {
		LOG_ERR("Pre-alloc buf size doesn't match required buf size (%d)",
			handle_size_required);
		return -ENOSR;
	}

	return 0;
}

static const struct crypto_driver_api npcx_crypto_api = {
	.hash_begin_session = npcx_hash_session_setup,
	.hash_free_session = npcx_hash_session_free,
	.query_hw_caps = npcx_query_caps,
};

DEVICE_DT_INST_DEFINE(0, npcx_hash_init, NULL, NULL, NULL, POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
		      &npcx_crypto_api);
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "only one 'nuvoton,npcx-sha' compatible node can be supported");
