#ifndef BQ76920_H
#define BQ76920_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BQ76920_I2C_ADDR_7BIT   0x18
#define BQ76920_I2C_ADDR        (BQ76920_I2C_ADDR_7BIT << 1)

#define BQ76920_CELL_COUNT      4

/* Registers */
#define BQ76920_SYS_STAT        0x00
#define BQ76920_CELLBAL1        0x01
#define BQ76920_SYS_CTRL1       0x04
#define BQ76920_SYS_CTRL2       0x05
#define BQ76920_PROTECT1        0x06
#define BQ76920_PROTECT2        0x07
#define BQ76920_PROTECT3        0x08
#define BQ76920_OV_TRIP         0x09
#define BQ76920_UV_TRIP         0x0A
#define BQ76920_CC_CFG          0x0B

#define BQ76920_VC1_HI          0x0C
#define BQ76920_VC1_LO          0x0D
#define BQ76920_VC2_HI          0x0E
#define BQ76920_VC2_LO          0x0F
#define BQ76920_VC3_HI          0x10
#define BQ76920_VC3_LO          0x11
#define BQ76920_VC4_HI          0x12
#define BQ76920_VC4_LO          0x13
#define BQ76920_VC5_HI          0x14
#define BQ76920_VC5_LO          0x15

#define BQ76920_BAT_HI          0x2A
#define BQ76920_BAT_LO          0x2B
#define BQ76920_TS1_HI          0x2C
#define BQ76920_TS1_LO          0x2D

#define BQ76920_ADCGAIN1        0x50
#define BQ76920_ADCOFFSET       0x51
#define BQ76920_ADCGAIN2        0x59

/* SYS_STAT bits */
#define BQ76920_SYS_STAT_OCD        (1 << 0)
#define BQ76920_SYS_STAT_SCD        (1 << 1)
#define BQ76920_SYS_STAT_OV         (1 << 2)
#define BQ76920_SYS_STAT_UV         (1 << 3)
#define BQ76920_SYS_STAT_OVRD_ALERT (1 << 4)
#define BQ76920_SYS_STAT_DEVICE_XREADY (1 << 5)

/* SYS_CTRL1 bits */
#define BQ76920_SYS_CTRL1_ADC_EN    (1 << 4)
#define BQ76920_SYS_CTRL1_TEMP_SEL  (1 << 3)

typedef struct
{
    int adc_gain_uv_per_lsb;
    int adc_offset_mv;
} BQ76920_Calibration;

typedef struct
{
    bool i2c_ok;

    float cell_v[BQ76920_CELL_COUNT];
    float pack_voltage_v;
    float cell_min_v;
    float cell_max_v;
    float cell_delta_v;

    uint8_t sys_stat;

    bool fault_ov;
    bool fault_uv;
    bool fault_ocd;
    bool fault_scd;
    bool fault_device_xready;

    BQ76920_Calibration cal;

} BQ76920_Telemetry;

bool BQ76920_IsConnected(void);

HAL_StatusTypeDef BQ76920_Read8(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef BQ76920_Write8(uint8_t reg, uint8_t value);
HAL_StatusTypeDef BQ76920_ReadBlock(uint8_t start_reg, uint8_t *buffer, uint8_t length);

bool BQ76920_EnableADC(void);
bool BQ76920_Init(void);

bool BQ76920_ReadCellVoltage(uint8_t cell_index, float *voltage_v);
bool BQ76920_ReadTelemetry(BQ76920_Telemetry *t);


#endif
