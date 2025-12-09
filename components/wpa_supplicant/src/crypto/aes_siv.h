/*
 * AES SIV (RFC 5297)
 * Copyright (c) 2013 Cozybit, Inc.
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef AES_SIV_H
#define AES_SIV_H

#include <stdint.h>
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;


int aes_siv_encrypt(const u8 *key, size_t key_len,
		    const u8 *pw, size_t pwlen,
		    size_t num_elem, const u8 *addr[], const size_t *len,
		    u8 *out);
int aes_siv_decrypt(const u8 *key, size_t key_len,
		    const u8 *iv_crypt, size_t iv_c_len,
		    size_t num_elem, const u8 *addr[], const size_t *len,
		    u8 *out);

#endif /* AES_SIV_H */
