/* SPDX-License-Identifier: Apache-2.0 */
#include "rmp.h"
#include <string.h>

static const uint8_t magic[4] = {'R', 'M', 'P', '1'};

uint16_t rmp_get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
uint32_t rmp_get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
uint64_t rmp_get_u64(const uint8_t *p) { return rmp_get_u32(p) | ((uint64_t)rmp_get_u32(p + 4) << 32); }
void rmp_put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
void rmp_put_u32(uint8_t *p, uint32_t v) { for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8 * i)); }
void rmp_put_u64(uint8_t *p, uint64_t v) { for (unsigned i = 0; i < 8; ++i) p[i] = (uint8_t)(v >> (8 * i)); }
void rmp_put_f32(uint8_t *p, float v) { uint32_t bits; memcpy(&bits, &v, sizeof(bits)); rmp_put_u32(p, bits); }

uint32_t rmp_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = UINT32_C(0xffffffff);
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & (uint32_t)-(int32_t)(crc & 1u));
        }
    }
    return ~crc;
}

size_t rmp_encode(const rmp_frame_t *frame, uint8_t *out, size_t capacity) {
    if (!frame || !out || frame->payload_length > RMP_MAX_PAYLOAD ||
        (frame->payload_length && !frame->payload)) return 0;
    size_t length = RMP_HEADER_SIZE + frame->payload_length + 4u;
    if (capacity < length) return 0;
    memcpy(out, magic, sizeof(magic));
    out[4] = RMP_VERSION;
    out[5] = frame->type;
    memset(out + 6, 0, 6);
    rmp_put_u16(out + 8, frame->payload_length);
    rmp_put_u32(out + 12, frame->boot_id);
    rmp_put_u32(out + 16, frame->sequence);
    rmp_put_u64(out + 20, frame->timestamp_us);
    if (frame->payload_length) memcpy(out + RMP_HEADER_SIZE, frame->payload, frame->payload_length);
    rmp_put_u32(out + length - 4, rmp_crc32(out, length - 4));
    return length;
}

void rmp_decoder_init(rmp_decoder_t *decoder) {
    memset(decoder, 0, sizeof(*decoder));
    decoder->max_payload = RMP_MAX_PAYLOAD;
}

static void discard(rmp_decoder_t *d, size_t count) {
    memmove(d->buffer, d->buffer + count, d->length - count);
    d->length -= count;
    d->discarded_bytes += (uint32_t)count;
}

static void decode_available(rmp_decoder_t *d, rmp_frame_callback_t callback, void *context) {
    for (;;) {
        while (d->length >= sizeof(magic) && memcmp(d->buffer, magic, sizeof(magic))) discard(d, 1);
        if (d->length < RMP_HEADER_SIZE) return;
        uint16_t payload = rmp_get_u16(d->buffer + 8);
        if (d->buffer[4] != RMP_VERSION || rmp_get_u16(d->buffer + 6) ||
            rmp_get_u16(d->buffer + 10) || payload > RMP_MAX_PAYLOAD || payload > d->max_payload) {
            ++d->invalid_frames;
            discard(d, 1);
            continue;
        }
        size_t length = RMP_HEADER_SIZE + payload + 4u;
        if (d->length < length) return;
        if (rmp_crc32(d->buffer, length - 4) != rmp_get_u32(d->buffer + length - 4)) {
            ++d->invalid_frames;
            discard(d, 1);
            continue;
        }
        rmp_frame_t frame = {
            .type = d->buffer[5], .boot_id = rmp_get_u32(d->buffer + 12),
            .sequence = rmp_get_u32(d->buffer + 16), .timestamp_us = rmp_get_u64(d->buffer + 20),
            .payload = d->buffer + RMP_HEADER_SIZE, .payload_length = payload,
        };
        callback(&frame, d->buffer, length, context);
        memmove(d->buffer, d->buffer + length, d->length - length);
        d->length -= length;
    }
}

void rmp_decoder_feed(rmp_decoder_t *d, const uint8_t *data, size_t length,
                      rmp_frame_callback_t callback, void *context) {
    if (!d || (!data && length) || !callback) return;
    for (size_t i = 0; i < length; ++i) {
        if (d->length == sizeof(d->buffer)) discard(d, 1);
        d->buffer[d->length++] = data[i];
        decode_available(d, callback, context);
    }
}
