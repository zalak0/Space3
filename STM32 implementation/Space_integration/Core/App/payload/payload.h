#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "stm32h7xx_hal.h"

void payload_task(sat_mode_t mode, fsw_ctx_t *ctx,
        I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

#endif
