/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_NODE_H
#define IMU_NODE_H
#include <stdint.h>
void imu_node_init(uint8_t address);
void imu_node_poll(void);
#endif
