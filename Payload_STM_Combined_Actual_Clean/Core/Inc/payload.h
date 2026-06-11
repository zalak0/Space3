#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "stm32h7xx_hal.h"

typedef enum {
    MODE_SCIENCE,
    MODE_DEPLOYMENT
} PayloadMode;

typedef struct {
    // Extend with any context state you need (e.g. tick counters, flags)
    uint8_t reserved;
} PayloadContext;

void payload_task(PayloadContext *ctx, PayloadMode mode,
                  I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

#endif
