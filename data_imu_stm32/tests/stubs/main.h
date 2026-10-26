#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stddef.h>
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct {
    uint32_t CR2;
} I2C_TypeDef;
typedef struct {
    I2C_TypeDef *Instance;
} I2C_HandleTypeDef;
#define HAL_I2C_STATE_READY 0x20u
#define HAL_I2C_STATE_LISTEN 0x28u
uint32_t HAL_I2C_GetState(I2C_HandleTypeDef *);
typedef struct {
    int unused;
} SPI_HandleTypeDef;
extern I2C_TypeDef test_bus1, test_bus2;
#define I2C1 (&test_bus1)
#define I2C2 (&test_bus2)
#define GPIOA 0
#define GPIO_PIN_0 1
#define GPIO_PIN_3 8
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define I2C_CR2_NACK 1u
#define I2C_FLAG_ADDR 1u
#define I2C_DIRECTION_TRANSMIT 0u
#define I2C_DIRECTION_RECEIVE 1u
#define I2C_FIRST_AND_LAST_FRAME 0u
#define __HAL_I2C_CLEAR_FLAG(i, f) ((void)(i), (void)(f))
static inline uint32_t __get_PRIMASK(void) {
    return 0;
}
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t p) {
    (void)p;
}
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void HAL_GPIO_WritePin(int, int, int);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t,
                                          uint32_t);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t,
                                          uint32_t);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t,
                                         uint32_t);
#endif
