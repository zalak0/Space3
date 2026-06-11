#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "stm32h7xx_hal.h"
#include "gps.h"  // for GPS_t

typedef enum {
    MODE_SCIENCE,
    MODE_DEPLOYMENT
} PayloadMode;

typedef struct {
    // Extend with any context state you need (e.g. tick counters, flags)
    uint8_t reserved;
} PayloadContext;

typedef struct {
    uint32_t timestamp_ms;

    // Langmuir probe sweep
    float meas_voltage[100];
    float meas_current[100];

    // GPS
    GPS_t gps;
    uint8_t gps_error;  // 0 = OK, nonzero = error code
} GOOSE_Payload;

void payload_task(PayloadContext *ctx, PayloadMode mode,
                  I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart,
                  GOOSE_Payload *out);

#endif
