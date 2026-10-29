/* SPDX-License-Identifier: Apache-2.0 */
#ifndef RMP_H
#define RMP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RMP_VERSION 1u
#define RMP_HEADER_SIZE 28u
#define RMP_MAX_PAYLOAD 1024u
#define RMP_MAX_FRAME (RMP_HEADER_SIZE + RMP_MAX_PAYLOAD + 4u)
#define RMP_HELLO 1u
#define RMP_HEARTBEAT 2u
#define RMP_TELEMETRY 16u
#define RMP_PING 32u
#define RMP_PONG 33u

typedef struct {
    uint8_t type;
    uint32_t boot_id;
    uint32_t sequence;
    uint64_t timestamp_us;
    const uint8_t *payload;
    uint16_t payload_length;
} rmp_frame_t;

typedef struct {
    uint8_t buffer[RMP_MAX_FRAME];
    size_t length;
    uint16_t max_payload; /* Application receive bound; init defaults to RMP_MAX_PAYLOAD. */
    uint32_t invalid_frames;
    uint32_t discarded_bytes;
} rmp_decoder_t;

/* Callback pointers are valid only until the callback returns. Copy if retained. */
typedef void (*rmp_frame_callback_t)(const rmp_frame_t *, const uint8_t *, size_t, void *);

uint16_t rmp_get_u16(const uint8_t *p);
uint32_t rmp_get_u32(const uint8_t *p);
uint64_t rmp_get_u64(const uint8_t *p);
void rmp_put_u16(uint8_t *p, uint16_t v);
void rmp_put_u32(uint8_t *p, uint32_t v);
void rmp_put_u64(uint8_t *p, uint64_t v);
void rmp_put_f32(uint8_t *p, float v);
uint32_t rmp_crc32(const uint8_t *data, size_t length);
/* Returns 0 on invalid arguments or insufficient output capacity. */
size_t rmp_encode(const rmp_frame_t *frame, uint8_t *out, size_t capacity);
void rmp_decoder_init(rmp_decoder_t *decoder);
void rmp_decoder_feed(rmp_decoder_t *decoder, const uint8_t *data, size_t length,
                      rmp_frame_callback_t callback, void *context);

#ifdef __cplusplus
}
#endif
#endif
