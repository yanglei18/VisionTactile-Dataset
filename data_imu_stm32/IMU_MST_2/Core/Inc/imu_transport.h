/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_TRANSPORT_H
#define IMU_TRANSPORT_H
#include <stdint.h>
void imu_transport_poll(void);
void imu_transport_timer(void);
void imu_transport_receive(const uint8_t *bytes, uint32_t length);
void imu_transport_complete(const uint8_t *buffer);
void imu_transport_usb_reset(void);
#endif
