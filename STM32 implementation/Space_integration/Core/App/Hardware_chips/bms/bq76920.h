#ifndef BQ76920_H
#define BQ76920_H

#include "stm32h7xx_hal.h"

#include <stdint.h>
#include <stdbool.h>

#define BQ76920_I2C_ADDR_7BIT       0x18
#define BQ76920_I2C_ADDR            (BQ76920_I2C_ADDR_7BIT << 1)

#define BQ76920_CELL_COUNT          5

#define BQ76920_SYS_STAT            0x00
#define BQ76920_CELLBAL1            0x01
#define BQ76920_SYS_CTRL1           0x04
#define BQ76920_SYS_CTRL2           0x05
#define BQ76920_CC_CFG              0x0B

#define BQ76920_VC1_HI              0x0C
#define BQ76920_ADCGAIN1            0x50
#define BQ76920_ADCOFFSET           0x51
#define BQ76920_ADCGAIN2            0x59

#define BQ76920_SYS_CTRL1_ADC_EN    (1U << 4)

#define BQ76920_SYS_STAT_OCD        (1U << 0)
#define BQ76920_SYS_STAT_SCD        (1U << 1)
#define BQ76920_SYS_STAT_OV         (1U << 2)
#define BQ76920_SYS_STAT_UV         (1U << 3)
#define BQ76920_SYS_STAT_DEVICE_XREADY  (1U << 5)

typedef struct {
    uint16_t adc_gain_uv_per_lsb;
    int8_t adc_offset_mv;
} BQ76920_Calibration;

typedef struct {
    bool i2c_ok;

    BQ76920_Calibration cal;

    uint8_t sys_stat;

    float cell_v[BQ76920_CELL_COUNT];
    float pack_voltage_v;
    float cell_min_v;
    float cell_max_v;
    float cell_delta_v;

    bool fault_ocd;
    bool fault_scd;
    bool fault_ov;
    bool fault_uv;
    bool fault_device_xready;

} BQ76920_Telemetry;

bool BQ76920_IsConnected(void);
HAL_StatusTypeDef BQ76920_Read8(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef BQ76920_Write8(uint8_t reg, uint8_t value);
HAL_StatusTypeDef BQ76920_ReadBlock(uint8_t start_reg, uint8_t *buffer, uint8_t length);

bool BQ76920_ReadCalibration(BQ76920_Calibration *cal);
bool BQ76920_EnableADC(void);
bool BQ76920_Init(void);

bool BQ76920_ReadCellVoltage(uint8_t cell_index, float *voltage_v);
bool BQ76920_ClearFaults(uint8_t fault_mask);
bool BQ76920_ReadTelemetry(BQ76920_Telemetry *t);

#endif