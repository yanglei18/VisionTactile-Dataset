/* Host HAL mock: execute real queue, USB callbacks and status command parser. */
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
I2C_TypeDef test_bus1, test_bus2;
I2C_HandleTypeDef hi2c1 = {I2C1}, hi2c2 = {I2C2};
static uint32_t ticks;
static const char *receive_during_delay;
void imu_transport_receive(const uint8_t *, uint32_t);
static uint8_t fail_node, legacy_node, sample_value = 10, usb_result, *usb_pointer;
static uint16_t usb_length;
uint32_t HAL_GetTick(void) {
    return ticks;
}
void HAL_Delay(uint32_t d) {
    ticks += d;
    if (receive_during_delay) {
        const char *command = receive_during_delay;
        receive_during_delay = NULL;
        imu_transport_receive((const uint8_t *)command, (uint32_t)strlen(command));
    }
}
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t a, uint8_t *p, uint16_t n,
                                          uint32_t t) {
    (void)h;
    (void)p;
    (void)n;
    (void)t;
    return (a >> 1) == fail_node ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t a, uint8_t *p, uint16_t n,
                                         uint32_t t) {
    (void)h;
    (void)t;
    unsigned id = a >> 1;
    if (id == fail_node)
        return HAL_ERROR;
    memset(p, sample_value, n);
    if (n == 32) {
        memset(p, 0, n);
        p[0] = 'V';
        p[1] = 'T';
        p[2] = 1;
        p[3] = id == legacy_node ? 1 : 2;
        p[4] = id;
        p[5] = 15;
        p[6] = 0x6b;
        p[7] = 0x4c;
        p[8] = 0x48;
        p[9] = 0x44;
    }
    return HAL_OK;
}
uint8_t CDC_Transmit_FS(uint8_t *p, uint16_t n) {
    if (!usb_result) {
        usb_pointer = p;
        usb_length = n;
    }
    return usb_result;
}
#include "../IMU_MST_2/Core/Src/imu_transport.c"
static void tick(void) {
    imu_transport_timer();
    imu_transport_poll();
}
static void complete(void) {
    imu_transport_complete(usb_pointer);
    imu_transport_poll();
}
int main(void) {
    tick();
    assert(queue.count == 2 && inflight && usb_length == 84);
    uint8_t original[84];
    memcpy(original, usb_pointer, 84);
    sample_value = 20;
    for (int i = 0; i < 10; ++i)
        tick();
    assert(queue.count == 8 && queue_overflows == 14 && !memcmp(original, usb_pointer, 84));
    complete();
    assert(queue.count == 7 && usb_completed == 1);
    imu_transport_usb_reset();
    imu_transport_poll();
    assert(!queue.count && !inflight && usb_discarded == 7);
    fail_node = 3;
    tick();
    assert(queue.count == 1 && bank_failures[0] == 1 && i2c_failures[2] == 1);
    assert(usb_pointer[0] == 7); /* No partial first bank or stale failed node. */
    complete();
    assert(!node_verified[2]);
    fail_node = 0;
    legacy_node = 3;
    ticks += 1000;
    tick();
    assert(!node_verified[2] && queue.count == 1 && usb_pointer[0] == 7);
    complete();
    legacy_node = 0;
    ticks += 1000;
    usb_result = USBD_BUSY;
    tick();
    assert(queue.count == 2 && !inflight && usb_busy);
    usb_result = USBD_OK;
    imu_transport_poll();
    assert(inflight);
    const uint8_t status_a[] = "VTIMU STA", status_b[] = "TUS 1 0011223344556677\n";
    imu_transport_receive(status_a, sizeof status_a - 1);
    imu_transport_receive(status_b, sizeof status_b - 1);
    imu_transport_poll();
    assert(!streaming);
    complete();
    /* Inject a second request from the USB IRQ while the first response is
       collecting node register readbacks. The first response must retain A. */
    receive_during_delay = "VTIMU STATUS 1 fedcba9876543210\n";
    complete();
    assert(inflight_is_status && usb_length < 8192);
    assert(!strncmp((char *)usb_pointer, "VTIMU STATUS 1 {", 16));
    assert(strstr((char *)usb_pointer, "\"ctrl3_c\":68") &&
           strstr((char *)usb_pointer, "\"firmware_version\":2"));
    assert(strstr((char *)usb_pointer, "\"request_id\":\"0011223344556677\""));
    printf("%.*s", usb_length, usb_pointer); /* Python runner validates JSON independently. */
    uint8_t first_reply[8192];
    uint16_t first_reply_length = usb_length;
    memcpy(first_reply, usb_pointer, first_reply_length);
    assert(request == 1 && !strcmp(request_id, "fedcba9876543210"));
    imu_transport_poll();
    assert(status_pending && inflight_is_status);
    assert(usb_length == first_reply_length &&
           !memcmp(first_reply, usb_pointer, first_reply_length));
    complete();
    assert(inflight_is_status && !status_pending);
    assert(strstr((char *)usb_pointer, "\"request_id\":\"fedcba9876543210\""));
    assert(!strstr((char *)usb_pointer, "0011223344556677"));
    printf("%.*s", usb_length, usb_pointer);
    complete();
    assert(!inflight && !status_length);
    imu_transport_receive((const uint8_t *)"VTIMU STREAM 1\n", 15);
    tick();
    assert(streaming && inflight);
    imu_transport_usb_reset();
    imu_transport_poll();
    const uint8_t bad[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxVTIMU STATUS 1\n";
    imu_transport_receive(bad, sizeof bad - 1);
    imu_transport_poll();
    assert(streaming && command_errors == 1);
    usb_result = USBD_BUSY;
    imu_transport_receive((const uint8_t *)"VTIMU STATUS 1 0123456789abcdef\n", 32);
    imu_transport_poll();
    assert(status_length > 0 && !inflight && !streaming);
    imu_transport_receive((const uint8_t *)"VTIMU STREAM 1\n", 15);
    imu_transport_poll();
    assert(status_length == 0 && !status_pending && streaming);
    const uint8_t invalid_nonce[] = "VTIMU STATUS 1\n"
                                    "VTIMU STATUS 1 0123456789abcdeF\n"
                                    "VTIMU STATUS 1 0123456789abcde\n";
    imu_transport_receive(invalid_nonce, sizeof invalid_nonce - 1);
    imu_transport_poll();
    assert(!status_pending && streaming && command_errors == 4);
    puts("transport: immutable in-flight bank, bounded overflow, completion, reset, bank "
         "rejection, busy retry, commands PASS");
}
