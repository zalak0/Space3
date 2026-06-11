#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "main.h"
#include "gps.h"

/* Operating modes */
typedef enum {
    MODE_SCIENCE    = 0,
    MODE_DEPLOYMENT = 1,
} PayloadMode;

/* Payload context — holds all state that used to be global in main.c */
typedef struct {
    float voltage_triangle;
    float voltage_langmuir;
    float voltage;
    float current;

    float measurements_voltage[100];
    float measurements_current[100];
    uint8_t meas_index;
} PayloadCtx;

/* Call once at startup */
void payload_init(PayloadCtx *ctx, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

/* Call every loop iteration */
void payload_task(PayloadCtx *ctx, PayloadMode mode, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

#endif /* PAYLOAD_H */
