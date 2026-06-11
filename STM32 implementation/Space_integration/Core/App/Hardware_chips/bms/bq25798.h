#ifndef BQ25798_H
#define BQ25798_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BQ25798_I2C_ADDR_7BIT   0x6B
#define BQ25798_I2C_ADDR        (BQ25798_I2C_ADDR_7BIT << 1)

/* BQ25798 register map */
#define BQ25798_REG_CHARGER_STATUS_0   0x1B
#define BQ25798_REG_CHARGER_STATUS_1   0x1C
#define BQ25798_REG_CHARGER_STATUS_2   0x1D
#define BQ25798_REG_CHARGER_STATUS_3   0x1E
#define BQ25798_REG_CHARGER_STATUS_4   0x1F

#define BQ25798_REG_FAULT_STATUS_0     0x20
#define BQ25798_REG_FAULT_STATUS_1     0x21

#define BQ25798_REG_ADC_CONTROL        0x2E

#define BQ25798_REG_IBUS_ADC           0x31
#define BQ25798_REG_IBAT_ADC           0x33
#define BQ25798_REG_VBUS_ADC           0x35
#define BQ25798_REG_VAC1_ADC           0x37
#define BQ25798_REG_VAC2_ADC           0x39
#define BQ25798_REG_VBAT_ADC           0x3B
#define BQ25798_REG_VSYS_ADC           0x3D
#define BQ25798_REG_TS_ADC             0x3F
#define BQ25798_REG_TDIE_ADC           0x41

#define BQ25798_REG_PART_INFORMATION   0x48

typedef struct
{
    bool i2c_ok;

    float vbus_v;
    float vac1_v;
    float vac2_v;
    float vbat_v;
    float vsys_v;

    float ibus_a;
    float ibat_a;

    float ts_percent;
    float die_temp_c;

    uint8_t charger_status_0;
    uint8_t charger_status_1;
    uint8_t charger_status_2;
    uint8_t charger_status_3;
    uint8_t charger_status_4;

    uint8_t fault_status_0;
    uint8_t fault_status_1;

    uint8_t part_info;

} BQ25798_Telemetry;

bool BQ25798_IsConnected(void);

HAL_StatusTypeDef BQ25798_Read8(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef BQ25798_Write8(uint8_t reg, uint8_t value);
HAL_StatusTypeDef BQ25798_Read16(uint8_t reg, uint16_t *value);

bool BQ25798_EnableADC(void);
bool BQ25798_ReadTelemetry(BQ25798_Telemetry *t);

#endif
