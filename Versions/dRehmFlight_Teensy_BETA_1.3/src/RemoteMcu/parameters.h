/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include "rmp.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
#define RMP_PARAM_GET 64u
#define RMP_PARAM_SET 65u
#define RMP_PARAM_SAVE 66u
#define RMP_PARAM_RESPONSE 67u
#define RMP_PARAM_MAX 128u
#define RMP_PARAM_BATCH 16u
#define RMP_PARAM_READ_PAGE 70u
#define RMP_PARAM_PAGE_RESPONSE 71u
#define RMP_PARAM_SCHEMA_PAGE 72u
#define RMP_PARAM_SCHEMA_RESPONSE 73u
#define RMP_PARAM_HEADER 20u
#define RMP_PARAM_BYTES 568u
enum { RMP_PARAM_F32=1, RMP_PARAM_U32=2, RMP_PARAM_BOOL=3 };
enum { RMP_APPLIED=0, RMP_SAVED=1, RMP_INVALID_ID=2, RMP_TYPE_MISMATCH=3,
       RMP_OUT_OF_RANGE=4, RMP_BUSY=5, RMP_UNSAFE=6, RMP_INTERNAL=7,
       RMP_REVISION_CONFLICT=8, RMP_REQUEST_CONFLICT=9, RMP_MALFORMED=10 };
typedef struct { uint16_t id; uint8_t type; double minimum, maximum; uint32_t initial; const char *name; } rmp_param_def_t;
typedef bool (*rmp_param_save_fn)(const uint32_t *, unsigned, uint32_t, void *);
typedef struct {
    uint64_t id; uint8_t kind; uint16_t request_length, response_length;
    uint8_t request[RMP_PARAM_BYTES], response[RMP_PARAM_BYTES]; bool valid;
} rmp_param_cache_t;
typedef struct {
    const rmp_param_def_t *definitions; unsigned count;
    uint32_t values[RMP_PARAM_MAX], revision, saved_revision;
    rmp_param_save_fn save; void *context;
    rmp_param_cache_t cache[8]; unsigned next_cache;
} rmp_params_t;
bool rmp_params_init(rmp_params_t *, const rmp_param_def_t *, unsigned, rmp_param_save_fn, void *);
bool rmp_params_valid(const rmp_params_t *, const uint32_t *);
/* Called from application task, never ISR. Response includes actual values/revisions. */
size_t rmp_params_handle(rmp_params_t *, uint8_t, const uint8_t *, size_t, bool, uint8_t *);
#ifdef __cplusplus
}
#endif
