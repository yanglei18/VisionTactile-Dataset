/* Host HAL mock: exercise the actual node callbacks and foreground code. */
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
I2C_TypeDef test_bus1, test_bus2;
I2C_HandleTypeDef hi2c1 = {I2C1};
SPI_HandleTypeDef hspi1;
static uint32_t ticks, i2c_state = HAL_I2C_STATE_LISTEN;
static unsigned listen_restarts, register_writes;
uint32_t HAL_I2C_GetState(I2C_HandleTypeDef *h) {
    (void)h;
    return i2c_state;
}
static uint8_t registers[128], payload_value = 10, *sent, *received;
static uint16_t sent_length;
static int spi_fail, interleave;
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *, uint8_t, uint16_t);
void HAL_GPIO_EXTI_Rising_Callback(uint16_t);
uint32_t HAL_GetTick(void) {
    return ticks;
}
void HAL_Delay(uint32_t d) {
    ticks += d;
}
void HAL_GPIO_WritePin(int a, int b, int c) {
    (void)a;
    (void)b;
    (void)c;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx,
                                          uint16_t n, uint32_t t) {
    (void)h;
    (void)t;
    if (spi_fail)
        return HAL_ERROR;
    if (n == 2) {
        if (tx[0] & 128)
            rx[1] = registers[tx[0] & 127];
        else {
            registers[tx[0]] = tx[1];
            ++register_writes;
        }
    } else {
        memset(rx, payload_value, n);
        if (interleave) {
            interleave = 0;
            HAL_I2C_AddrCallback(&hi2c1, I2C_DIRECTION_RECEIVE, 1);
            HAL_GPIO_EXTI_Rising_Callback(GPIO_PIN_0);
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *h, uint8_t *p, uint16_t n,
                                               uint32_t o) {
    (void)h;
    (void)n;
    (void)o;
    received = p;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *h, uint8_t *p, uint16_t n,
                                                uint32_t o) {
    (void)h;
    (void)o;
    sent = p;
    sent_length = n;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *h) {
    (void)h;
    i2c_state = HAL_I2C_STATE_LISTEN;
    ++listen_restarts;
    return HAL_OK;
}
#include "../IMU_SLV_MCU/Core/Src/imu_node.c"
int main(void) {
    registers[15] = 0x6b;
    imu_node_init(1);
    assert(flags == (VT_STATUS_IDENTITY | VT_STATUS_CONFIG | VT_STATUS_REGISTERS_READ));
    assert(registers[0x12] == 0x44);
    HAL_GPIO_EXTI_Rising_Callback(GPIO_PIN_0);
    imu_node_poll();
    assert(sample_count == 1 && latest.bytes[0] == 0);
    payload_value = 20;
    interleave = 1;
    HAL_GPIO_EXTI_Rising_Callback(GPIO_PIN_0);
    imu_node_poll();
    assert(sent_length == 13 && sent[1] == 10 && latest.bytes[1] == 20);
    assert(drdy_events == 1);               /* An interrupt during SPI survives foreground work. */
    assert(sent[0] == 0 && sent[12] == 10); /* TX snapshot survives later publication. */
    imu_node_poll();
    assert(sample_count == 3);
    spi_fail = 1;
    HAL_GPIO_EXTI_Rising_Callback(GPIO_PIN_0);
    imu_node_poll();
    sent = NULL;
    HAL_I2C_AddrCallback(&hi2c1, I2C_DIRECTION_RECEIVE, 1);
    assert(!sent && !latest.valid && spi_errors == 1 && sample_count == 3);
    spi_fail = 0;
    HAL_GPIO_EXTI_Rising_Callback(GPIO_PIN_0);
    imu_node_poll();
    ticks = 31;
    sent = NULL;
    HAL_I2C_AddrCallback(&hi2c1, I2C_DIRECTION_RECEIVE, 1);
    assert(!sent);
    /* Model C0 HAL I2C_ITSlaveCplt: rejected read, NO_OPTION, STOP -> READY;
       the HAL does not issue ListenCplt for this branch. */
    i2c_state = HAL_I2C_STATE_READY;
    imu_node_poll();
    assert(i2c_state == HAL_I2C_STATE_LISTEN && listen_restarts == 1);
    i2c_state = HAL_I2C_STATE_READY;
    HAL_I2C_SlaveTxCpltCallback(&hi2c1);
    assert(i2c_state == HAL_I2C_STATE_LISTEN && listen_restarts == 2);
    HAL_I2C_AddrCallback(&hi2c1, I2C_DIRECTION_TRANSMIT, 1);
    *received = VT_STATUS_COMMAND;
    HAL_I2C_SlaveRxCpltCallback(&hi2c1);
    registers[0x11] = 0x40;
    imu_node_poll();
    HAL_I2C_AddrCallback(&hi2c1, I2C_DIRECTION_RECEIVE, 1);
    assert(sent_length == 32 && sent[0] == 'V' && sent[8] == 0x40 && !(sent[5] & VT_STATUS_CONFIG));
    assert(!latest.valid);
    registers[15] = 0;
    unsigned writes_before = register_writes;
    imu_node_init(1);
    assert(!(flags & VT_STATUS_IDENTITY));
    assert(register_writes == writes_before);
    puts(
        "node: snapshot isolation, DRDY wakeup, SPI failure, stale expiry, register readback PASS");
}
