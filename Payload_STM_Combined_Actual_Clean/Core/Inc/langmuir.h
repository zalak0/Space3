#ifndef LANGMUIR_H
#define LANGMUIR_H

#include "stm32h7xx_hal.h"

#define MCP3021_ADDR        (0x4D << 1)
#define MEAS_BUFFER_SIZE    100

typedef struct {
    float voltage[MEAS_BUFFER_SIZE];
    float current[MEAS_BUFFER_SIZE];
    uint8_t index;
} LangmuirData;

extern LangmuirData langmuir_data;  // Expose if other modules need it

void langmuir_init(void);           // Call once before the loop
void langmuir_task(I2C_HandleTypeDef *hi2c);

#endif
