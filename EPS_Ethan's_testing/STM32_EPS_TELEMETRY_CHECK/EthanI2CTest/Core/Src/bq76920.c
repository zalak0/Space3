#include "bq76920.h"

extern I2C_HandleTypeDef hi2c2;

static BQ76920_Calibration g_cal = {
    .adc_gain_uv_per_lsb = 365,
    .adc_offset_mv = 0
};

bool BQ76920_IsConnected(void)
{
    return HAL_I2C_IsDeviceReady(
        &hi2c2,
        BQ76920_I2C_ADDR,
        3,
        100
    ) == HAL_OK;
}

HAL_StatusTypeDef BQ76920_Read8(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(
        &hi2c2,
        BQ76920_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        100
    );
}

HAL_StatusTypeDef BQ76920_Write8(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(
        &hi2c2,
        BQ76920_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100
    );
}

HAL_StatusTypeDef BQ76920_ReadBlock(uint8_t start_reg, uint8_t *buffer, uint8_t length)
{
    return HAL_I2C_Mem_Read(
        &hi2c2,
        BQ76920_I2C_ADDR,
        start_reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        length,
        100
    );
}

//bool BQ76920_ReadCalibration(BQ76920_Calibration *cal)
//{
//    uint8_t gain1;
//    uint8_t gain2;
//    uint8_t offset_raw;
//
//    if (cal == 0)
//        return false;
//
//    if (BQ76920_Read8(BQ76920_ADCGAIN1, &gain1) != HAL_OK)
//        return false;
//
//    if (BQ76920_Read8(BQ76920_ADCGAIN2, &gain2) != HAL_OK)
//        return false;
//
//    if (BQ76920_Read8(BQ76920_ADCOFFSET, &offset_raw) != HAL_OK)
//        return false;
//
//    /*
//        BQ769x0 ADC gain formula:
//        ADCGAIN[4:0] is assembled from ADCGAIN1 and ADCGAIN2.
//        Final gain is nominal 365 uV/LSB plus trim value.
//    */
//    uint8_t adc_gain_bits =
//        ((gain1 & 0x0C) << 1) |
//        ((gain2 & 0xE0) >> 5);
//
//    cal->adc_gain_uv_per_lsb = 365 + adc_gain_bits;
//    cal->adc_offset_mv = (int8_t)offset_raw;
//
//    g_cal = *cal;
//
//    return true;
//}

bool BQ76920_EnableADC(void)
{
    uint8_t value;

    if (BQ76920_Read8(BQ76920_SYS_CTRL1, &value) != HAL_OK)
        return false;

    value |= BQ76920_SYS_CTRL1_ADC_EN;

    return BQ76920_Write8(BQ76920_SYS_CTRL1, value) == HAL_OK;
}

bool BQ76920_Init(void)
{
    if (!BQ76920_IsConnected())
        return false;

    /*
        Datasheet recommendation: CC_CFG should be written to 0x19.
        Even if you are not using coulomb counting yet, this is commonly set during bring-up.
    */
    if (BQ76920_Write8(BQ76920_CC_CFG, 0x19) != HAL_OK)
        return false;

    if (!BQ76920_ReadCalibration(&g_cal))
        return false;

    if (!BQ76920_EnableADC())
        return false;

    return true;
}

static uint16_t BQ76920_Combine14Bit(uint8_t hi, uint8_t lo)
{
    return (((uint16_t)hi << 8) | lo) & 0x3FFF;
}

static float BQ76920_AdcToCellVoltage(uint16_t adc)
{
    /*
        Cell voltage equation:
        VCELL = ADC_reading × ADCGAIN + ADCOFFSET

        adc_gain is in uV/LSB.
        offset is in mV.
    */
    float mv =
        ((float)adc * (float)g_cal.adc_gain_uv_per_lsb) / 1000.0f
        + (float)g_cal.adc_offset_mv;

    return mv / 1000.0f;
}

bool BQ76920_ReadCellVoltage(uint8_t cell_index, float *voltage_v)
{
    uint8_t start_reg;
    uint8_t data[2];

    if (voltage_v == 0)
        return false;

    if (cell_index >= BQ76920_CELL_COUNT)
        return false;

    start_reg = BQ76920_VC1_HI + (cell_index * 2);

    if (BQ76920_ReadBlock(start_reg, data, 2) != HAL_OK)
        return false;

    uint16_t adc = BQ76920_Combine14Bit(data[0], data[1]);

    *voltage_v = BQ76920_AdcToCellVoltage(adc);

    return true;
}

//bool BQ76920_ClearFaults(uint8_t fault_mask)
//{
//    /*
//        SYS_STAT fault bits are cleared by writing 1 to the active bit.
//    */
//    return BQ76920_Write8(BQ76920_SYS_STAT, fault_mask) == HAL_OK;
//}

bool BQ76920_ReadTelemetry(BQ76920_Telemetry *t)
{
    if (t == 0)
        return false;

    t->i2c_ok = BQ76920_IsConnected();

    if (!t->i2c_ok)
        return false;

    t->cal = g_cal;

    if (BQ76920_Read8(BQ76920_SYS_STAT, &t->sys_stat) != HAL_OK)
        return false;

    t->pack_voltage_v = 0.0f;
    t->cell_min_v = 99.0f;
    t->cell_max_v = 0.0f;

    for (uint8_t i = 0; i < BQ76920_CELL_COUNT; i++)
    {
        if (!BQ76920_ReadCellVoltage(i, &t->cell_v[i]))
            return false;

        t->pack_voltage_v += t->cell_v[i];

        if (t->cell_v[i] < t->cell_min_v)
            t->cell_min_v = t->cell_v[i];

        if (t->cell_v[i] > t->cell_max_v)
            t->cell_max_v = t->cell_v[i];
    }

    t->cell_delta_v = t->cell_max_v - t->cell_min_v;

    t->fault_ocd = (t->sys_stat & BQ76920_SYS_STAT_OCD) != 0;
    t->fault_scd = (t->sys_stat & BQ76920_SYS_STAT_SCD) != 0;
    t->fault_ov  = (t->sys_stat & BQ76920_SYS_STAT_OV)  != 0;
    t->fault_uv  = (t->sys_stat & BQ76920_SYS_STAT_UV)  != 0;
    t->fault_device_xready =
        (t->sys_stat & BQ76920_SYS_STAT_DEVICE_XREADY) != 0;

    return true;
}
