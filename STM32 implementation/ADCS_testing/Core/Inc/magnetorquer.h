#ifndef MAGNETORQUER_H
#define MAGNETORQUER_H

#include "main.h"          /* brings in stm32f3xx_hal.h → TIM_HandleTypeDef, uint32_t, TIM_CHANNEL_x */
#include "aplqr.h"         /* for adcs_dipole_t used by torquer_apply */

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch_in1;
    uint32_t ch_in2;
} mtq_bridge_t;

void  drv8833_init(void);
void  drv8833_set_axis(int axis, float duty);
void  drv8833_coast_all(void);
void  torquer_apply(const adcs_dipole_t *m);   /* declare it so main.c/aplqr.c see it */

#endif
