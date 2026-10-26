/* SPDX-License-Identifier: Apache-2.0
 * All SPI operations run in foreground. I2C interrupts only copy a committed
 * mailbox into a separate immutable transmit buffer. No DMA/IT transfer ever
 * references SPI scratch or a buffer being published by the foreground.
 */
#include "imu_node.h"
#include "../../../common/vt_imu_common.h"
#include "main.h"
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
static vt_sample latest;
static uint8_t tx_snapshot[VT_STATUS_BYTES], status_snapshot[VT_STATUS_BYTES];
static uint8_t node_id, command, who, xl, gyro, ctrl3, flags;
static volatile uint8_t status_requested, status_ready, status_selected;
static volatile uint32_t drdy_events, i2c_errors;
static uint32_t sample_count, spi_errors, drdy_coalesced;
static uint32_t last_probe_ms;
static uint32_t lock(void) {
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    return p;
}
static void unlock(uint32_t p) {
    __set_PRIMASK(p);
}
static int transfer(uint8_t *tx, uint8_t *rx, uint16_t length) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi1, tx, rx, length, 2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    if (result != HAL_OK) {
        ++spi_errors;
        return 0;
    }
    return 1;
}
static int read_register(uint8_t reg, uint8_t *value) {
    uint8_t tx[2] = {(uint8_t)(reg | 0x80u), 0xff}, rx[2];
    if (!transfer(tx, rx, 2))
        return 0;
    *value = rx[1];
    return 1;
}
static int write_register(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value}, rx[2];
    return transfer(tx, rx, 2);
}
static void verify_registers(void) {
    uint8_t next_who = 0, next_xl = 0, next_g = 0, next_c = 0;
    int ok = read_register(0x0f, &next_who);
    ok = read_register(0x10, &next_xl) && ok;
    ok = read_register(0x11, &next_g) && ok;
    ok = read_register(0x12, &next_c) && ok;
    uint32_t p = lock();
    who = next_who;
    xl = next_xl;
    gyro = next_g;
    ctrl3 = next_c;
    flags = ok ? VT_STATUS_REGISTERS_READ : 0;
    if (ok && who == 0x6b)
        flags |= VT_STATUS_IDENTITY;
    if (ok && who == 0x6b && xl == 0x4c && gyro == 0x48 && ctrl3 == 0x44)
        flags |= VT_STATUS_CONFIG;
    if (!(flags & VT_STATUS_CONFIG))
        latest.valid = 0;
    unlock(p);
    last_probe_ms = HAL_GetTick();
}
static void publish_status(void) {
    uint8_t s[VT_STATUS_BYTES] = {0};
    s[0] = 'V';
    s[1] = 'T';
    s[2] = 1;
    s[3] = VT_FIRMWARE_VERSION;
    s[4] = node_id;
    uint32_t p = lock();
    s[5] = flags;
    if (latest.valid && (uint32_t)(HAL_GetTick() - latest.time_ms) <= VT_SAMPLE_MAX_AGE_MS)
        s[5] |= VT_STATUS_SAMPLE_VALID;
    s[6] = who;
    s[7] = xl;
    s[8] = gyro;
    s[9] = ctrl3;
    vt_put_u32(s + 12, sample_count);
    vt_put_u32(s + 16, spi_errors);
    vt_put_u32(s + 20, drdy_coalesced);
    vt_put_u32(s + 24, i2c_errors);
    vt_put_u32(s + 28, drdy_events);
    memcpy(status_snapshot, s, sizeof s);
    status_ready = 1;
    unlock(p);
}
void imu_node_init(uint8_t address) {
    node_id = address;
    /* IF_INC and BDU explicitly enabled; do not assume power-on defaults. */
    uint8_t identity = 0;
    int ok = read_register(0x0f, &identity) && identity == 0x6b;
    if (ok) {
        ok = write_register(0x12, 0x44);
        ok = write_register(0x10, 0x4c) && ok;
        ok = write_register(0x11, 0x48) && ok;
        ok = write_register(0x0d, 0x02) && ok;
    }
    verify_registers();
    if (!ok)
        flags &= (uint8_t)~VT_STATUS_CONFIG;
    publish_status();
}
/* C0 HAL's rejected-address STOP path may transition LISTEN -> READY
 * without invoking ListenCplt. Re-arm only after the transaction has ended;
 * never replace the buffer/state of a BUSY transfer. */
static void ensure_listening(I2C_HandleTypeDef *i2c) {
    uint32_t p = lock();
    if (HAL_I2C_GetState(i2c) == HAL_I2C_STATE_READY && HAL_I2C_EnableListen_IT(i2c) != HAL_OK)
        ++i2c_errors;
    unlock(p);
}
void imu_node_poll(void) {
    ensure_listening(&hi2c1);
    uint32_t p = lock(), pending = drdy_events;
    drdy_events = 0;
    uint8_t requested = status_requested;
    status_requested = 0;
    unlock(p);
    if (requested || (uint32_t)(HAL_GetTick() - last_probe_ms) >= 1000u)
        verify_registers();
    if (pending) {
        /* There is no FIFO: older DRDY events cannot be recovered by rereading. */
        if (pending > 1)
            drdy_coalesced += pending - 1;
        uint8_t tx[13] = {0xa2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                          0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
                rx[13];
        if ((flags & VT_STATUS_CONFIG) && transfer(tx, rx, sizeof tx)) {
            p = lock();
            vt_sample_publish(&latest, rx + 1, sample_count++, HAL_GetTick());
            unlock(p);
        } else {
            p = lock();
            latest.valid = 0;
            unlock(p);
        }
    }
    if (requested)
        publish_status();
}
void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin) {
    if (pin == GPIO_PIN_0 && drdy_events != UINT32_MAX)
        ++drdy_events;
}
static void reject_address(I2C_HandleTypeDef *i2c) {
    i2c->Instance->CR2 |= I2C_CR2_NACK;
    __HAL_I2C_CLEAR_FLAG(i2c, I2C_FLAG_ADDR);
}
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *i2c, uint8_t direction, uint16_t address) {
    (void)address;
    if (i2c->Instance != I2C1)
        return;
    HAL_StatusTypeDef result;
    if (direction == I2C_DIRECTION_TRANSMIT) {
        result = HAL_I2C_Slave_Seq_Receive_IT(i2c, &command, 1, I2C_FIRST_AND_LAST_FRAME);
    } else {
        uint16_t length = VT_SAMPLE_BYTES;
        if (status_selected) {
            if (!status_ready) {
                reject_address(i2c);
                return;
            }
            memcpy(tx_snapshot, status_snapshot, VT_STATUS_BYTES);
            length = VT_STATUS_BYTES;
            status_selected = 0;
        } else if (!vt_sample_copy(&latest, tx_snapshot, HAL_GetTick())) {
            reject_address(i2c);
            return;
        }
        result = HAL_I2C_Slave_Seq_Transmit_IT(i2c, tx_snapshot, length, I2C_FIRST_AND_LAST_FRAME);
    }
    if (result != HAL_OK) {
        ++i2c_errors;
        reject_address(i2c);
    }
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *i2c) {
    if (i2c->Instance == I2C1) {
        status_selected = (command == VT_STATUS_COMMAND);
        if (status_selected) {
            status_ready = 0;
            status_requested = 1;
        }
    }
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *i2c) {
    if (i2c->Instance == I2C1)
        ensure_listening(i2c);
}
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *i2c) {
    if (i2c->Instance == I2C1 && HAL_I2C_EnableListen_IT(i2c) != HAL_OK)
        ++i2c_errors;
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c) {
    if (i2c->Instance == I2C1) {
        ++i2c_errors;
        if (HAL_I2C_EnableListen_IT(i2c) != HAL_OK)
            ++i2c_errors;
    }
}
