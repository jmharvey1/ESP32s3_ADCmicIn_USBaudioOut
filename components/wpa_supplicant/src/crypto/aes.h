/*
 * AES functions
 * Copyright (c) 2003-2006, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef AES_H
#define AES_H

#define AES_BLOCK_SIZE 16
#include <stdint.h>
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

void * aes_encrypt_init(const u8 *key, size_t len);
int aes_encrypt(void *ctx, const u8 *plain, u8 *crypt);
void aes_encrypt_deinit(void *ctx);
void * aes_decrypt_init(const u8 *key, size_t len);
int aes_decrypt(void *ctx, const u8 *crypt, u8 *plain);
void aes_decrypt_deinit(void *ctx);

#endif /* AES_H */
