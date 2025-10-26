/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SHA_HMAC_NPCX_H_
#define _SHA_HMAC_NPCX_H_

#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include "soc_ncl.h"

int npcx_hmac_compute(const struct device *dev, 
	struct hash_ctx *ctx,
	const uint8_t *key,
	uint32_t key_len,
	const uint8_t *msg,
	uint8_t msg_len,
	uint8_t *hmac);

#endif /* _SHA_HMAC_NPCX_H_ */
