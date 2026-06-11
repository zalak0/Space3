#ifndef GPS_TASK_H
#define GPS_TASK_H

#include "stm32h7xx_hal.h"

void gps_task_init(void);           // Call once before the loop
void gps_task(UART_HandleTypeDef *huart);

#endif
