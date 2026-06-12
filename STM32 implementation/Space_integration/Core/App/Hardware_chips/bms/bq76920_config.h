#ifndef BQ76920_CONFIG_H
#define BQ76920_CONFIG_H

#include "stm32h7xx_hal.h"   // Change if using another STM32 family
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * BQ76920 I2C ADDRESS
 * ============================================================ */

#define BQ76920_I2C_ADDR_7BIT      0x18
#define BQ76920_I2C_ADDR_HAL       (BQ76920_I2C_ADDR_7BIT << 1)

/* ============================================================
 * BQ76920 REGISTER MAP
 * ============================================================ */

#define BQ76920_REG_SYS_STAT       0x00
#define BQ76920_REG_CELLBAL1       0x01
#define BQ76920_REG_SYS_CTRL1      0x04
#define BQ76920_REG_SYS_CTRL2      0x05
#define BQ76920_REG_PROTECT1       0x06
#define BQ76920_REG_PROTECT2       0x07
#define BQ76920_REG_PROTECT3       0x08
#define BQ76920_REG_OV_TRIP        0x09
#define BQ76920_REG_UV_TRIP        0x0A
#define BQ76920_REG_CC_CFG         0x0B

#define BQ76920_REG_VC1_HI         0x0C
#define BQ76920_REG_VC1_LO         0x0D
#define BQ76920_REG_VC2_HI         0x0E
#define BQ76920_REG_VC2_LO         0x0F
#define BQ76920_REG_VC3_HI         0x10
#define BQ76920_REG_VC3_LO         0x11
#define BQ76920_REG_VC4_HI         0x12
#define BQ76920_REG_VC4_LO         0x13
#define BQ76920_REG_VC5_HI         0x14
#define BQ76920_REG_VC5_LO         0x15

#define BQ76920_REG_BAT_HI         0x2A
#define BQ76920_REG_BAT_LO         0x2B
#define BQ76920_REG_TS1_HI         0x2C
#define BQ76920_REG_TS1_LO         0x2D
#define BQ76920_REG_CC_HI          0x32
#define BQ76920_REG_CC_LO          0x33

#define BQ76920_REG_ADCGAIN1       0x50
#define BQ76920_REG_ADCOFFSET      0x51
#define BQ76920_REG_ADCGAIN2       0x59

/* ============================================================
 * BIT DEFINITIONS
 * ============================================================ */

#define BQ76920_SYS_CTRL1_ADC_EN   0x10
#define BQ76920_SYS_CTRL1_TEMP_SEL 0x08

#define BQ76920_SYS_CTRL2_CHG_ON   0x01
#define BQ76920_SYS_CTRL2_DSG_ON   0x02
#define BQ76920_SYS_CTRL2_CC_EN    0x40

/* ============================================================
 * DEVICE HANDLE
 * ============================================================ */

typedef struct
{
    I2C_HandleTypeDef *hi2c;

    GPIO_TypeDef *ts1_boot_gpio_port;
    uint16_t      ts1_boot_gpio_pin;

    uint16_t adc_gain_uV_per_lsb;
    int8_t   adc_offset_mV;

} BQ76920_HandleTypeDef;

/* ============================================================
 * FUNCTION DECLARATIONS
 * ============================================================ */

HAL_StatusTypeDef BQ76920_Boot_TS1(BQ76920_HandleTypeDef *dev);

HAL_StatusTypeDef BQ76920_WriteReg(BQ76920_HandleTypeDef *dev,
                                   uint8_t reg,
                                   uint8_t value);

HAL_StatusTypeDef BQ76920_ReadReg(BQ76920_HandleTypeDef *dev,
                                  uint8_t reg,
                                  uint8_t *value);

HAL_StatusTypeDef BQ76920_ReadWord14(BQ76920_HandleTypeDef *dev,
                                     uint8_t high_reg,
                                     uint16_t *raw14);

HAL_StatusTypeDef BQ76920_ReadCalibration(BQ76920_HandleTypeDef *dev);

HAL_StatusTypeDef BQ76920_Init_MonitoringOnly(BQ76920_HandleTypeDef *dev);

HAL_StatusTypeDef BQ76920_ReadCellRaw(BQ76920_HandleTypeDef *dev,
                                      uint8_t cell_number,
                                      uint16_t *raw14);

HAL_StatusTypeDef BQ76920_ReadCellVoltage_mV(BQ76920_HandleTypeDef *dev,
                                             uint8_t cell_number,
                                             int32_t *voltage_mV);

HAL_StatusTypeDef BQ76920_ReadAllCells_mV(BQ76920_HandleTypeDef *dev,
                                          int32_t cell_mV[5]);

HAL_StatusTypeDef BQ76920_ClearFaults(BQ76920_HandleTypeDef *dev);

#endif
