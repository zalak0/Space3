#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "stm32h7xx_hal.h"
#include "stm/modes.h"
#include "common/fsw_ctx.h"
#include "comms/comms.h"

void payload_task(sat_mode_t mode, fsw_ctx_t *ctx,
        I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

        typedef struct {
    uint32_t timestamp_ms;

    // Langmuir probe sweep
    float meas_voltage[100];
    float meas_current[100];

    // GPS
    GPS_t gps;
    uint8_t gps_error;  // 0 = OK, nonzero = error code
} GOOSE_Payload;

#endif
