/* SPDX-License-Identifier: Apache-2.0
 * Wire constants and allocation-free primitives shared by firmware and host tests.
 * Callers serialize mailbox publication against interrupts. USB queue mutation
 * belongs to the foreground; an in-flight front entry must not be popped early.
 */
#ifndef VT_IMU_COMMON_H
#define VT_IMU_COMMON_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#define VT_FIRMWARE_VERSION 2u
#define VT_STATUS_COMMAND 0xf0u
#define VT_STATUS_BYTES 32u
#define VT_SAMPLE_BYTES 13u
#define VT_BANK_BYTES 84u
#define VT_QUEUE_CAPACITY 8u
#define VT_STATUS_IDENTITY 1u
#define VT_STATUS_CONFIG 2u
#define VT_STATUS_SAMPLE_VALID 4u
#define VT_STATUS_REGISTERS_READ 8u
#define VT_SAMPLE_MAX_AGE_MS 30u

typedef struct {
    uint8_t bytes[VT_SAMPLE_BYTES];
    uint32_t time_ms;
    uint8_t valid;
} vt_sample;
static inline void vt_sample_publish(vt_sample *s, const uint8_t axes[12], uint32_t count,
                                     uint32_t now) {
    s->bytes[0] = (uint8_t)count;
    memcpy(s->bytes + 1, axes, 12);
    s->time_ms = now;
    s->valid = 1;
}
static inline int vt_sample_copy(const vt_sample *s, uint8_t out[VT_SAMPLE_BYTES], uint32_t now) {
    if (!s->valid || (uint32_t)(now - s->time_ms) > VT_SAMPLE_MAX_AGE_MS)
        return 0;
    memcpy(out, s->bytes, VT_SAMPLE_BYTES);
    return 1;
}
typedef struct {
    uint8_t data[VT_QUEUE_CAPACITY][VT_BANK_BYTES];
    uint8_t head, count;
} vt_bank_queue;
static inline int vt_queue_push(vt_bank_queue *q, const uint8_t bank[VT_BANK_BYTES]) {
    if (q->count == VT_QUEUE_CAPACITY)
        return 0;
    memcpy(q->data[(q->head + q->count) % VT_QUEUE_CAPACITY], bank, VT_BANK_BYTES);
    ++q->count;
    return 1;
}
static inline uint8_t *vt_queue_front(vt_bank_queue *q) {
    return q->count ? q->data[q->head] : NULL;
}
static inline void vt_queue_pop(vt_bank_queue *q) {
    if (q->count) {
        q->head = (q->head + 1) % VT_QUEUE_CAPACITY;
        --q->count;
    }
}
static inline void vt_put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static inline uint32_t vt_get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
#endif
