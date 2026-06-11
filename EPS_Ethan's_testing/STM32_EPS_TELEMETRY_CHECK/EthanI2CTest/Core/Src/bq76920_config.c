#include "bq76920_config.h"

/* ============================================================
 * PRIVATE GPIO HELPERS
 * ============================================================ */

static void BQ76920_TS1_GPIO_Output(BQ76920_HandleTypeDef *dev)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = dev->ts1_boot_gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(dev->ts1_boot_gpio_port, &GPIO_InitStruct);
}

static void BQ76920_TS1_GPIO_HiZ(BQ76920_HandleTypeDef *dev)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = dev->ts1_boot_gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(dev->ts1_boot_gpio_port, &GPIO_InitStruct);
}

/* ============================================================
 * BOOT FROM SHIP MODE USING TS1
 * ============================================================ */

HAL_StatusTypeDef BQ76920_Boot_TS1(BQ76920_HandleTypeDef *dev)
{
    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    BQ76920_TS1_GPIO_Output(dev);

    HAL_GPIO_WritePin(dev->ts1_boot_gpio_port,
                      dev->ts1_boot_gpio_pin,
                      GPIO_PIN_SET);

    HAL_Delay(10);

    HAL_GPIO_WritePin(dev->ts1_boot_gpio_port,
                      dev->ts1_boot_gpio_pin,
                      GPIO_PIN_RESET);

    BQ76920_TS1_GPIO_HiZ(dev);

    HAL_Delay(20);

    return HAL_OK;
}

/* ============================================================
 * BASIC I2C REGISTER ACCESS
 * ============================================================ */

HAL_StatusTypeDef BQ76920_WriteReg(BQ76920_HandleTypeDef *dev,
                                   uint8_t reg,
                                   uint8_t value)
{
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(dev->hi2c,
                             BQ76920_I2C_ADDR_HAL,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             HAL_MAX_DELAY);
}

HAL_StatusTypeDef BQ76920_ReadReg(BQ76920_HandleTypeDef *dev,
                                  uint8_t reg,
                                  uint8_t *value)
{
    if (dev == NULL || dev->hi2c == NULL || value == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            BQ76920_I2C_ADDR_HAL,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1,
                            HAL_MAX_DELAY);
}

/* ============================================================
 * READ 14-BIT ADC WORD
 * ============================================================ */

HAL_StatusTypeDef BQ76920_ReadWord14(BQ76920_HandleTypeDef *dev,
                                     uint8_t high_reg,
                                     uint16_t *raw14)
{
    HAL_StatusTypeDef status;
    uint8_t data[2];

    if (dev == NULL || dev->hi2c == NULL || raw14 == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(dev->hi2c,
                              BQ76920_I2C_ADDR_HAL,
                              high_reg,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              2,
                              HAL_MAX_DELAY);

    if (status != HAL_OK)
    {
        return status;
    }

    *raw14 = (((uint16_t)data[0] << 8) | data[1]) & 0x3FFF;

    return HAL_OK;
}

/* ============================================================
 * READ ADC CALIBRATION VALUES
 * ============================================================ */

HAL_StatusTypeDef BQ76920_ReadCalibration(BQ76920_HandleTypeDef *dev)
{
    HAL_StatusTypeDef status;
    uint8_t adc_gain1;
    uint8_t adc_gain2;
    uint8_t adc_offset;

    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    status = BQ76920_ReadReg(dev, BQ76920_REG_ADCGAIN1, &adc_gain1);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ76920_ReadReg(dev, BQ76920_REG_ADCGAIN2, &adc_gain2);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ76920_ReadReg(dev, BQ76920_REG_ADCOFFSET, &adc_offset);
    if (status != HAL_OK)
    {
        return status;
    }

    dev->adc_gain_uV_per_lsb =
        365u +
        ((adc_gain1 & 0x0C) << 1) +
        ((adc_gain2 & 0xE0) >> 5);

    dev->adc_offset_mV = (int8_t)adc_offset;

    return HAL_OK;
}

/* ============================================================
 * CLEAR STATUS / FAULT FLAGS
 * ============================================================ */

HAL_StatusTypeDef BQ76920_ClearFaults(BQ76920_HandleTypeDef *dev)
{
    return BQ76920_WriteReg(dev, BQ76920_REG_SYS_STAT, 0xFF);
}

/* ============================================================
 * MONITORING-ONLY INITIALISATION
 * ============================================================ */

HAL_StatusTypeDef BQ76920_Init_MonitoringOnly(BQ76920_HandleTypeDef *dev)
{
    HAL_StatusTypeDef status;

    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    status = BQ76920_Boot_TS1(dev);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ76920_ClearFaults(dev);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ76920_WriteReg(dev, BQ76920_REG_CC_CFG, 0x19);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ76920_WriteReg(dev, BQ76920_REG_SYS_CTRL2, 0x00);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ76920_WriteReg(dev, BQ76920_REG_CELLBAL1, 0x00);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ76920_WriteReg(dev,
                              BQ76920_REG_SYS_CTRL1,
                              BQ76920_SYS_CTRL1_ADC_EN);

    if (status != HAL_OK)
    {
        return status;
    }

    status = BQ76920_ReadCalibration(dev);
    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(250);

    return HAL_OK;
}

/* ============================================================
 * READ SINGLE CELL RAW ADC VALUE
 * ============================================================ */

HAL_StatusTypeDef BQ76920_ReadCellRaw(BQ76920_HandleTypeDef *dev,
                                      uint8_t cell_number,
                                      uint16_t *raw14)
{
    uint8_t high_reg;

    if (dev == NULL || raw14 == NULL)
    {
        return HAL_ERROR;
    }

    switch (cell_number)
    {
        case 1:
            high_reg = BQ76920_REG_VC1_HI;
            break;

        case 2:
            high_reg = BQ76920_REG_VC2_HI;
            break;

        case 3:
            high_reg = BQ76920_REG_VC3_HI;
            break;

        case 4:
            high_reg = BQ76920_REG_VC4_HI;
            break;

        case 5:
            high_reg = BQ76920_REG_VC5_HI;
            break;

        default:
            return HAL_ERROR;
    }

    return BQ76920_ReadWord14(dev, high_reg, raw14);
}

/* ============================================================
 * READ SINGLE CELL VOLTAGE IN mV
 * ============================================================ */

HAL_StatusTypeDef BQ76920_ReadCellVoltage_mV(BQ76920_HandleTypeDef *dev,
                                             uint8_t cell_number,
                                             int32_t *voltage_mV)
{
    HAL_StatusTypeDef status;
    uint16_t raw14;

    if (dev == NULL || voltage_mV == NULL)
    {
        return HAL_ERROR;
    }

    status = BQ76920_ReadCellRaw(dev, cell_number, &raw14);
    if (status != HAL_OK)
    {
        return status;
    }

    *voltage_mV =
        ((int32_t)raw14 * (int32_t)dev->adc_gain_uV_per_lsb) / 1000L
        + (int32_t)dev->adc_offset_mV;

    return HAL_OK;
}

/* ============================================================
 * READ ALL CELL VOLTAGES
 * ============================================================ */

HAL_StatusTypeDef BQ76920_ReadAllCells_mV(BQ76920_HandleTypeDef *dev,
                                          int32_t cell_mV[5])
{
    HAL_StatusTypeDef status;

    if (dev == NULL || cell_mV == NULL)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < 5; i++)
    {
        status = BQ76920_ReadCellVoltage_mV(dev, i + 1, &cell_mV[i]);

        if (status != HAL_OK)
        {
            return status;
        }
    }

    return HAL_OK;
}