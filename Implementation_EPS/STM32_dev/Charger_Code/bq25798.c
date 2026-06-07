#include "main.h"
#include "bq25798.h"
#include "fsw_ctx.h"

extern I2C_HandleTypeDef hi2c1;

static uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max)
{
    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}

bool BQ25798_IsConnected(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c1, BQ25798_I2C_ADDR, 3, 100) == HAL_OK;
}

HAL_StatusTypeDef BQ25798_Read8(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(
        &hi2c1,
        BQ25798_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        100
    );
}

HAL_StatusTypeDef BQ25798_Write8(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(
        &hi2c1,
        BQ25798_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100
    );
}

HAL_StatusTypeDef BQ25798_Read16(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        BQ25798_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        2,
        100
    );

    if (status == HAL_OK) {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }

    return status;
}

HAL_StatusTypeDef BQ25798_Write16(uint8_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)((value >> 8) & 0xFF);
    data[1] = (uint8_t)(value & 0xFF);

    return HAL_I2C_Mem_Write(
        &hi2c1,
        BQ25798_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        2,
        100
    );
}

bool BQ25798_SetMinSystemVoltage_mV(uint16_t mv)
{
    mv = clamp_u16(mv, 2500, 16000);

    uint8_t reg_value = (uint8_t)((mv - 2500) / 250);
    reg_value &= 0x3F;

    return BQ25798_Write8(BQ25798_REG_VSYSMIN, reg_value) == HAL_OK;
}

bool BQ25798_SetChargeVoltage_mV(uint16_t mv)
{
    mv = clamp_u16(mv, 3000, 18800);

    uint16_t reg_value = mv / 10;
    reg_value &= 0x07FF;

    return BQ25798_Write16(BQ25798_REG_CHARGE_VOLTAGE, reg_value) == HAL_OK;
}

bool BQ25798_SetChargeCurrent_mA(uint16_t ma)
{
    ma = clamp_u16(ma, 50, 5000);

    uint16_t reg_value = ma / 10;
    reg_value &= 0x01FF;

    return BQ25798_Write16(BQ25798_REG_CHARGE_CURRENT, reg_value) == HAL_OK;
}

bool BQ25798_SetInputVoltageLimit_mV(uint16_t mv)
{
    mv = clamp_u16(mv, 3600, 22000);

    uint8_t reg_value = (uint8_t)(mv / 100);

    return BQ25798_Write8(BQ25798_REG_INPUT_VOLTAGE_LIMIT, reg_value) == HAL_OK;
}

bool BQ25798_SetInputCurrentLimit_mA(uint16_t ma)
{
    ma = clamp_u16(ma, 100, 3300);

    uint16_t reg_value = ma / 10;
    reg_value &= 0x01FF;

    return BQ25798_Write16(BQ25798_REG_INPUT_CURRENT_LIMIT, reg_value) == HAL_OK;
}

bool BQ25798_EnableCharging(bool enable)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_CONTROL_0, &reg) != HAL_OK) {
        return false;
    }

    if (enable) {
        reg |= BQ25798_EN_CHG_BIT;
    } else {
        reg &= (uint8_t)(~BQ25798_EN_CHG_BIT);
    }

    return BQ25798_Write8(BQ25798_REG_CHARGER_CONTROL_0, reg) == HAL_OK;
}

bool BQ25798_EnableChargerHardware(bool enable)
{
    bool ok;

    if (enable) {
        HAL_GPIO_WritePin(BQ25798_CE_GPIO_Port, BQ25798_CE_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);

        ok = BQ25798_EnableCharging(true);

        if (!ok) {
            HAL_GPIO_WritePin(BQ25798_CE_GPIO_Port, BQ25798_CE_Pin, GPIO_PIN_SET);
        }

        return ok;
    }

    ok = BQ25798_EnableCharging(false);
    HAL_GPIO_WritePin(BQ25798_CE_GPIO_Port, BQ25798_CE_Pin, GPIO_PIN_SET);

    return ok;
}

bool BQ25798_EnableTermination(bool enable)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_CONTROL_0, &reg) != HAL_OK) {
        return false;
    }

    if (enable) {
        reg |= BQ25798_EN_TERM_BIT;
    } else {
        reg &= (uint8_t)(~BQ25798_EN_TERM_BIT);
    }

    return BQ25798_Write8(BQ25798_REG_CHARGER_CONTROL_0, reg) == HAL_OK;
}

bool BQ25798_SetWatchdog(BQ25798_WatchdogSetting setting)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_CONTROL_1, &reg) != HAL_OK) {
        return false;
    }

    reg &= (uint8_t)(~BQ25798_WATCHDOG_MASK);
    reg |= ((uint8_t)setting & BQ25798_WATCHDOG_MASK);

    return BQ25798_Write8(BQ25798_REG_CHARGER_CONTROL_1, reg) == HAL_OK;
}

bool BQ25798_ResetWatchdog(void)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_CONTROL_1, &reg) != HAL_OK) {
        return false;
    }

    reg |= BQ25798_WD_RST_BIT;

    return BQ25798_Write8(BQ25798_REG_CHARGER_CONTROL_1, reg) == HAL_OK;
}

bool BQ25798_EnableADC(bool enable)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_ADC_CONTROL, &reg) != HAL_OK) {
        return false;
    }

    if (enable) {
        reg |= 0x80;
    } else {
        reg &= (uint8_t)(~0x80);
    }

    return BQ25798_Write8(BQ25798_REG_ADC_CONTROL, reg) == HAL_OK;
}

bool BQ25798_InitCharger4S(void)
{
    if (!BQ25798_IsConnected()) {
        return false;
    }

    if (!BQ25798_SetWatchdog(BQ25798_WATCHDOG_DISABLED)) {
        return false;
    }

    if (!BQ25798_EnableADC(true)) {
        return false;
    }

    if (!BQ25798_SetMinSystemVoltage_mV(12000)) {
        return false;
    }

    if (!BQ25798_SetChargeVoltage_mV(16800)) {
        return false;
    }

    if (!BQ25798_SetChargeCurrent_mA(500)) {
        return false;
    }

    if (!BQ25798_SetInputVoltageLimit_mV(5000)) {
        return false;
    }

    if (!BQ25798_SetInputCurrentLimit_mA(1000)) {
        return false;
    }

    return BQ25798_EnableChargerHardware(true);
}

void BQ25798_ReadRailGPIOStatus(fsw_ctx_t *ctx)
{
    if (ctx == 0) {
        return;
    }

    ctx->rail_3v3_pg = HAL_GPIO_ReadPin(EPS_3V3_PG_GPIO_Port, EPS_3V3_PG_Pin) == GPIO_PIN_SET;
    ctx->rail_3v3_flt = HAL_GPIO_ReadPin(EPS_3V3_FLT_GPIO_Port, EPS_3V3_FLT_Pin) == GPIO_PIN_RESET;

    ctx->rail_5v_pg = HAL_GPIO_ReadPin(EPS_5V_PG_GPIO_Port, EPS_5V_PG_Pin) == GPIO_PIN_SET;
    ctx->rail_5v_flt = HAL_GPIO_ReadPin(EPS_5V_FLT_GPIO_Port, EPS_5V_FLT_Pin) == GPIO_PIN_RESET;

    ctx->vbus_flt = HAL_GPIO_ReadPin(EPS_VBUS_FLT_GPIO_Port, EPS_VBUS_FLT_Pin) == GPIO_PIN_RESET;
    ctx->vbus_pg = HAL_GPIO_ReadPin(EPS_VBUS_PG_GPIO_Port, EPS_VBUS_PG_Pin) == GPIO_PIN_SET;
}

static float adc_voltage_to_v(uint16_t raw)
{
    return raw / 1000.0f;
}

static float adc_current_to_a(uint16_t raw)
{
    return ((int16_t)raw) / 1000.0f;
}

static float adc_temp_to_c(uint16_t raw)
{
    return ((int16_t)raw) * 0.5f;
}

bool BQ25798_ReadTelemetry(BQ25798_Telemetry *t)
{
    uint16_t raw;

    if (t == 0) {
        return false;
    }

    t->i2c_ok = BQ25798_IsConnected();

    if (!t->i2c_ok) {
        return false;
    }

    t->vbus_v = 0.0f;
    t->vbat_v = 0.0f;
    t->vsys_v = 0.0f;
    t->ibus_a = 0.0f;
    t->ibat_a = 0.0f;
    t->die_temp_c = 0.0f;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_0, &t->charger_status_0) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_1, &t->charger_status_1) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_2, &t->charger_status_2) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_3, &t->charger_status_3) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_4, &t->charger_status_4) != HAL_OK) return false;

    if (BQ25798_Read8(BQ25798_REG_FAULT_STATUS_0, &t->fault_status_0) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_FAULT_STATUS_1, &t->fault_status_1) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_PART_INFORMATION, &t->part_info) != HAL_OK) return false;

    if (BQ25798_Read16(BQ25798_REG_VBUS_ADC, &raw) == HAL_OK) {
        t->vbus_v = adc_voltage_to_v(raw);
    }

    if (BQ25798_Read16(BQ25798_REG_VBAT_ADC, &raw) == HAL_OK) {
        t->vbat_v = adc_voltage_to_v(raw);
    }

    if (BQ25798_Read16(BQ25798_REG_VSYS_ADC, &raw) == HAL_OK) {
        t->vsys_v = adc_voltage_to_v(raw);
    }

    if (BQ25798_Read16(BQ25798_REG_IBUS_ADC, &raw) == HAL_OK) {
        t->ibus_a = adc_current_to_a(raw);
    }

    if (BQ25798_Read16(BQ25798_REG_IBAT_ADC, &raw) == HAL_OK) {
        t->ibat_a = adc_current_to_a(raw);
    }

    if (BQ25798_Read16(BQ25798_REG_TDIE_ADC, &raw) == HAL_OK) {
        t->die_temp_c = adc_temp_to_c(raw);
    }

    return true;
}