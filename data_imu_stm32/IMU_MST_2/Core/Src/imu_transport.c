/* SPDX-License-Identifier: Apache-2.0
 * Foreground owns the bounded queue; IRQs publish events only. The USB driver
 * retains a pointer until its completion callback or deinitialization. A busy
 * endpoint therefore cannot cause an in-flight buffer to be overwritten.
 */
#include "imu_transport.h"
#include "../../../common/vt_imu_common.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <stdio.h>
extern I2C_HandleTypeDef hi2c1, hi2c2;
static vt_bank_queue queue;
static uint8_t bank[VT_BANK_BYTES];
static char status_line[8192], command_line[32];
/* IRQ mailbox and foreground pending request are separate from the immutable
 * USB response. All 16 nonce bytes are copied while interrupts are masked. */
static char request_id[17], pending_request_id[17];
static size_t status_length, command_length;
static uint8_t command_overflow, streaming = 1, status_pending;
static uint8_t node_verified[12], probe_started;
static uint32_t last_probe_ms;
static volatile uint8_t request, reset_event, completed;
static volatile uint32_t timer_events;
static uint8_t *volatile inflight;
static uint8_t inflight_is_status;
static uint32_t timer_coalesced, bank_failures[2], i2c_failures[12];
static uint32_t queue_overflows, usb_busy, usb_failures, usb_discarded, usb_completed;
static volatile uint32_t command_errors;
static uint32_t status_overflows;
static uint32_t lock(void) {
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    return p;
}
static void unlock(uint32_t p) {
    __set_PRIMASK(p);
}
void imu_transport_timer(void) {
    if (timer_events != UINT32_MAX)
        ++timer_events;
}
void imu_transport_usb_reset(void) {
    reset_event = 1;
    request = 0;
    request_id[0] = '\0';
    command_length = 0;
    command_overflow = 0;
    inflight = NULL;
    completed = 0;
}
void imu_transport_complete(const uint8_t *buffer) {
    if (buffer == inflight)
        completed = 1;
}
static int is_status_command(void) {
    if (command_length != 31 || memcmp(command_line, "VTIMU STATUS 1 ", 15) != 0)
        return 0;
    for (size_t i = 15; i < 31; ++i) {
        char c = command_line[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
            return 0;
    }
    return 1;
}
void imu_transport_receive(const uint8_t *bytes, uint32_t length) {
    for (uint32_t i = 0; i < length; ++i) {
        uint8_t c = bytes[i];
        if (c == '\n') {
            if (!command_overflow) {
                command_line[command_length] = '\0';
                if (is_status_command()) {
                    memcpy(request_id, command_line + 15, 16);
                    request_id[16] = '\0';
                    request = 1;
                } else if (strcmp(command_line, "VTIMU STREAM 1") == 0)
                    request = 2;
                else
                    ++command_errors;
            } else
                ++command_errors;
            command_length = 0;
            command_overflow = 0;
        } else if (!command_overflow) {
            if (command_length + 1 >= sizeof command_line || c < 32 || c > 126)
                command_overflow = 1;
            else
                command_line[command_length++] = (char)c;
        }
    }
}
static int append(const char *format, ...) {
    if (status_length >= sizeof status_line)
        return 0;
    va_list args;
    va_start(args, format);
    int length =
        vsnprintf(status_line + status_length, sizeof status_line - status_length, format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= sizeof status_line - status_length) {
        status_length = sizeof status_line;
        return 0;
    }
    status_length += (size_t)length;
    return 1;
}
static int node_status(unsigned id, uint8_t status[VT_STATUS_BYTES]) {
    I2C_HandleTypeDef *bus = id <= 6 ? &hi2c1 : &hi2c2;
    uint8_t command = VT_STATUS_COMMAND;
    if (HAL_I2C_Master_Transmit(bus, (uint16_t)(id << 1), &command, 1, 2) != HAL_OK)
        return 0;
    /* Foreground SPI register readback on the node completes before its reply. */
    for (unsigned retry = 0; retry < 3; ++retry) {
        HAL_Delay(1);
        if (HAL_I2C_Master_Receive(bus, (uint16_t)(id << 1), status, VT_STATUS_BYTES, 2) == HAL_OK)
            return status[0] == 'V' && status[1] == 'T' && status[2] == 1 &&
                   status[3] == VT_FIRMWARE_VERSION && status[4] == id;
    }
    return 0;
}
static int verify_node(const uint8_t s[VT_STATUS_BYTES]) {
    return (s[5] & (VT_STATUS_IDENTITY | VT_STATUS_CONFIG | VT_STATUS_REGISTERS_READ)) ==
               (VT_STATUS_IDENTITY | VT_STATUS_CONFIG | VT_STATUS_REGISTERS_READ) &&
           s[6] == 0x6b && s[7] == 0x4c && s[8] == 0x48 && s[9] == 0x44;
}
static void probe_unverified_nodes(void) {
    for (unsigned id = 1; id <= 12; ++id) {
        if (!node_verified[id - 1]) {
            uint8_t s[VT_STATUS_BYTES];
            node_verified[id - 1] = node_status(id, s) && verify_node(s);
        }
    }
    probe_started = 1;
    last_probe_ms = HAL_GetTick();
}
static void build_status(void) {
    status_length = 0;
    append("VTIMU STATUS 1 "
           "{\"schema_version\":1,\"request_id\":\"%s\",\"firmware_version\":2,"
           "\"imu_model\":\"LSM6DSR\",\"nominal_sample_rate_hz\":104,\"nodes\":[",
           pending_request_id);
    for (unsigned id = 1; id <= 12; ++id) {
        uint8_t s[VT_STATUS_BYTES];
        int ok = node_status(id, s);
        node_verified[id - 1] = ok && verify_node(s);
        append("%s{\"node_id\":%u,\"reachable\":%s", id == 1 ? "" : ",", id, ok ? "true" : "false");
        if (ok) {
            append(",\"firmware_version\":%u,\"identity_verified\":%s,\"configuration_verified\":%"
                   "s,\"registers_read\":%s,\"sample_valid\":%s",
                   s[3], s[5] & VT_STATUS_IDENTITY ? "true" : "false",
                   s[5] & VT_STATUS_CONFIG ? "true" : "false",
                   s[5] & VT_STATUS_REGISTERS_READ ? "true" : "false",
                   s[5] & VT_STATUS_SAMPLE_VALID ? "true" : "false");
            if (s[5] & VT_STATUS_REGISTERS_READ)
                append(",\"registers\":{\"who_am_i\":%u,\"ctrl1_xl\":%u,\"ctrl2_g\":%u,\"ctrl3_c\":"
                       "%u}",
                       s[6], s[7], s[8], s[9]);
            append(",\"sample_count\":%lu,\"spi_errors\":%lu,\"drdy_coalesced\":%lu,\"i2c_errors\":"
                   "%lu",
                   (unsigned long)vt_get_u32(s + 12), (unsigned long)vt_get_u32(s + 16),
                   (unsigned long)vt_get_u32(s + 20), (unsigned long)vt_get_u32(s + 24));
        }
        append(",\"master_i2c_failures\":%lu}", (unsigned long)i2c_failures[id - 1]);
    }
    append("],\"transport\":{\"queue_capacity_banks\":%u,\"queue_overflows\":%lu,\"usb_busy_"
           "attempts\":%lu,\"usb_failures\":%lu,\"usb_discarded_banks\":%lu,\"usb_completed_"
           "banks\":%lu,\"timer_coalesced\":%lu,\"bank_failures\":[%lu,%lu],\"command_errors\":%lu,"
           "\"status_overflows\":%lu}}\n",
           VT_QUEUE_CAPACITY, (unsigned long)queue_overflows, (unsigned long)usb_busy,
           (unsigned long)usb_failures, (unsigned long)usb_discarded, (unsigned long)usb_completed,
           (unsigned long)timer_coalesced, (unsigned long)bank_failures[0],
           (unsigned long)bank_failures[1], (unsigned long)command_errors,
           (unsigned long)status_overflows);
    if (status_length >= sizeof status_line) {
        ++status_overflows;
        status_length = 0;
        append("VTIMU STATUS 1 {\"schema_version\":1,\"request_id\":\"%s\","
               "\"error\":\"status_overflow\"}\n",
               pending_request_id);
    }
}
static void acquire_bank(unsigned bank_index) {
    I2C_HandleTypeDef *bus = bank_index ? &hi2c2 : &hi2c1;
    int valid = 1;
    for (unsigned n = 0; n < 6; ++n) {
        unsigned id = bank_index * 6 + n + 1;
        bank[n * 14] = (uint8_t)id;
        if (!node_verified[id - 1]) {
            valid = 0;
            continue;
        }
        if (HAL_I2C_Master_Receive(bus, (uint16_t)(id << 1), bank + n * 14 + 1, 13, 2) != HAL_OK) {
            ++i2c_failures[id - 1];
            node_verified[id - 1] = 0;
            valid = 0;
        }
    }
    if (!valid) {
        ++bank_failures[bank_index];
        return;
    }
    if (!vt_queue_push(&queue, bank))
        ++queue_overflows;
}
static void service_usb(void) {
    uint32_t p = lock();
    if (reset_event) {
        unlock(p);
        return;
    }
    if (completed) {
        completed = 0;
        if (inflight && !inflight_is_status) {
            vt_queue_pop(&queue);
            ++usb_completed;
        }
        if (inflight_is_status)
            status_length = 0;
        inflight = NULL;
        inflight_is_status = 0;
    }
    if (!inflight) {
        uint8_t *data = vt_queue_front(&queue);
        uint16_t length = VT_BANK_BYTES;
        if (!data && status_length) {
            data = (uint8_t *)status_line;
            length = (uint16_t)status_length;
            inflight_is_status = 1;
        }
        if (data) {
            inflight = data;
            uint8_t result = CDC_Transmit_FS(data, length);
            if (result != USBD_OK) {
                inflight = NULL;
                inflight_is_status = 0;
                if (result == USBD_BUSY)
                    ++usb_busy;
                else
                    ++usb_failures;
            }
        }
    }
    unlock(p);
}
void imu_transport_poll(void) {
    uint32_t p = lock(), pending = timer_events;
    timer_events = 0;
    uint8_t cmd = request;
    char next_request_id[17];
    memcpy(next_request_id, request_id, sizeof next_request_id);
    request = 0;
    if (reset_event) {
        reset_event = 0;
        usb_discarded += queue.count;
        memset(&queue, 0, sizeof queue);
        inflight = NULL;
        completed = 0;
        inflight_is_status = 0;
        status_length = 0;
        status_pending = 0;
        streaming = 1;
        memset(node_verified, 0, sizeof node_verified);
        probe_started = 0;
    }
    unlock(p);
    if (cmd == 1) {
        memcpy(pending_request_id, next_request_id, sizeof pending_request_id);
        streaming = 0;
        status_pending = 1;
    }
    if (cmd == 2) {
        streaming = 1;
        status_pending = 0;
        /* A timed-out query may leave text waiting for a busy endpoint. Cancel
         * that unsent reply; an already submitted reply keeps its buffer until
         * USB completion, before any new binary bank can be transmitted. */
        if (!inflight_is_status)
            status_length = 0;
    }
    service_usb();
    if (status_pending && !queue.count && !inflight && !status_length) {
        build_status();
        status_pending = 0;
        service_usb();
    }
    if (streaming && (!probe_started || (uint32_t)(HAL_GetTick() - last_probe_ms) >= 1000u))
        probe_unverified_nodes();
    if (streaming && pending) {
        if (pending > 1)
            timer_coalesced += pending - 1;
        acquire_bank(0);
        acquire_bank(1);
        service_usb();
    }
}
