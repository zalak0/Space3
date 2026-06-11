#include "bq25798.h"
#include <stdio.h>
extern I2C_HandleTypeDef hi2c2;

static uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

bool BQ25798_IsConnected(void)
{
    return HAL_I2C_IsDeviceReady(
        &hi2c2,
        BQ25798_I2C_ADDR,
        3,
        100
    ) == HAL_OK;
}

HAL_StatusTypeDef BQ25798_Read8(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(
        &hi2c2,
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
        &hi2c2,
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
        &hi2c2,
        BQ25798_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        2,
        100
    );

    if (status == HAL_OK)
    {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }

    return status;
}

HAL_StatusTypeDef BQ25798_Write16(uint8_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;

    return HAL_I2C_Mem_Write(
        &hi2c2,
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
    /*
        VSYSMIN:
        Range: 2500 mV to 16000 mV
        Step: 250 mV
        Register = (mV - 2500) / 250
    */
    mv = clamp_u16(mv, 2500, 16000);

    uint8_t reg_value = (uint8_t)((mv - 2500) / 250);
    reg_value &= 0x3F;

    return BQ25798_Write8(BQ25798_REG_VSYSMIN, reg_value) == HAL_OK;
}

bool BQ25798_SetChargeVoltage_mV(uint16_t mv)
{
    /*
        VREG:
        Range: 3000 mV to 18800 mV
        Step: 10 mV
        Register = mV / 10
    */
    mv = clamp_u16(mv, 3000, 18800);

    uint16_t reg_value = mv / 10;
    reg_value &= 0x07FF;

    return BQ25798_Write16(BQ25798_REG_CHARGE_VOLTAGE, reg_value) == HAL_OK;
}

bool BQ25798_SetChargeCurrent_mA(uint16_t ma)
{
    /*
        ICHG:
        Range: 50 mA to 5000 mA
        Step: 10 mA
        Register = mA / 10
    */
    ma = clamp_u16(ma, 50, 5000);

    uint16_t reg_value = ma / 10;
    reg_value &= 0x01FF;

    return BQ25798_Write16(BQ25798_REG_CHARGE_CURRENT, reg_value) == HAL_OK;
}

bool BQ25798_SetInputVoltageLimit_mV(uint16_t mv)
{
    /*
        VINDPM:
        Range: 3600 mV to 22000 mV
        Step: 100 mV
        Register = mV / 100
    */
    mv = clamp_u16(mv, 3600, 22000);

    uint8_t reg_value = (uint8_t)(mv / 100);

    return BQ25798_Write8(BQ25798_REG_INPUT_VOLTAGE_LIMIT, reg_value) == HAL_OK;
}

bool BQ25798_SetInputCurrentLimit_mA(uint16_t ma)
{
    /*
        IINDPM:
        Range: 100 mA to 3300 mA
        Step: 10 mA
        Register = mA / 10
    */
    ma = clamp_u16(ma, 100, 3300);

    uint16_t reg_value = ma / 10;
    reg_value &= 0x01FF;

    return BQ25798_Write16(BQ25798_REG_INPUT_CURRENT_LIMIT, reg_value) == HAL_OK;
}

bool BQ25798_EnableCharging(bool enable)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_CONTROL_0, &reg) != HAL_OK)
        return false;

    if (enable)
        reg |= BQ25798_EN_CHG_BIT;
    else
        reg &= ~BQ25798_EN_CHG_BIT;

    return BQ25798_Write8(BQ25798_REG_CHARGER_CONTROL_0, reg) == HAL_OK;
}

bool BQ25798_EnableTermination(bool enable)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_CONTROL_0, &reg) != HAL_OK)
        return false;

    if (enable)
        reg |= BQ25798_EN_TERM_BIT;
    else
        reg &= ~BQ25798_EN_TERM_BIT;

    return BQ25798_Write8(BQ25798_REG_CHARGER_CONTROL_0, reg) == HAL_OK;
}

bool BQ25798_SetWatchdog(BQ25798_WatchdogSetting setting)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_CONTROL_1, &reg) != HAL_OK)
        return false;

    reg &= ~BQ25798_WATCHDOG_MASK;
    reg |= ((uint8_t)setting & BQ25798_WATCHDOG_MASK);

    return BQ25798_Write8(BQ25798_REG_CHARGER_CONTROL_1, reg) == HAL_OK;
}

bool BQ25798_ResetWatchdog(void)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_CONTROL_1, &reg) != HAL_OK)
        return false;

    reg |= BQ25798_WD_RST_BIT;

    return BQ25798_Write8(BQ25798_REG_CHARGER_CONTROL_1, reg) == HAL_OK;
}

bool BQ25798_EnableADC(bool enable)
{
    uint8_t reg;

    if (BQ25798_Read8(BQ25798_REG_ADC_CONTROL, &reg) != HAL_OK)
        return false;

    if (enable)
        reg |= 0x80;
    else
        reg &= ~0x80;

    return BQ25798_Write8(BQ25798_REG_ADC_CONTROL, reg) == HAL_OK;
}

static float adc_voltage_to_v(uint16_t raw)
{
    return raw / 1000.0f;
}

static float adc_current_to_a(uint16_t raw)
{
    int16_t signed_raw = (int16_t)raw;
    return signed_raw / 1000.0f;
}

static float adc_temp_to_c(uint16_t raw)
{
    int16_t signed_raw = (int16_t)raw;
    return signed_raw * 0.5f;
}

bool BQ25798_ReadTelemetry(BQ25798_Telemetry *t)
{
    uint16_t raw;

    if (t == 0)
        return false;

    /*
     * Clear the struct first so unused fields are never random.
     */
    t->i2c_ok = false;

    t->vbus_v = 0.0f;
    t->vbat_v = 0.0f;
    t->vsys_v = 0.0f;
    t->ibus_a = 0.0f;
    t->ibat_a = 0.0f;
    t->die_temp_c = 0.0f;

    t->charger_status_0 = 0;
    t->charger_status_1 = 0;
    t->charger_status_2 = 0;
    t->charger_status_3 = 0;
    t->charger_status_4 = 0;

    t->fault_status_0 = 0;
    t->fault_status_1 = 0;
    t->part_info = 0;

    t->i2c_ok = BQ25798_IsConnected();

    if (!t->i2c_ok)
        return false;

    /*
     * Read all charger status registers.
     */
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_0, &t->charger_status_0) != HAL_OK)
        return false;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_1, &t->charger_status_1) != HAL_OK)
        return false;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_2, &t->charger_status_2) != HAL_OK)
        return false;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_3, &t->charger_status_3) != HAL_OK)
        return false;

    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_4, &t->charger_status_4) != HAL_OK)
        return false;

    /*
     * Read fault status registers.
     */
    if (BQ25798_Read8(BQ25798_REG_FAULT_STATUS_0, &t->fault_status_0) != HAL_OK)
        return false;

    if (BQ25798_Read8(BQ25798_REG_FAULT_STATUS_1, &t->fault_status_1) != HAL_OK)
        return false;

    if (BQ25798_Read8(BQ25798_REG_PART_INFORMATION, &t->part_info) != HAL_OK)
        return false;

    /*
     * Read ADC telemetry.
     */
    if (BQ25798_Read16(BQ25798_REG_VBUS_ADC, &raw) == HAL_OK)
        t->vbus_v = adc_voltage_to_v(raw);

    if (BQ25798_Read16(BQ25798_REG_VBAT_ADC, &raw) == HAL_OK)
        t->vbat_v = adc_voltage_to_v(raw);

    if (BQ25798_Read16(BQ25798_REG_VSYS_ADC, &raw) == HAL_OK)
        t->vsys_v = adc_voltage_to_v(raw);

    if (BQ25798_Read16(BQ25798_REG_IBUS_ADC, &raw) == HAL_OK)
        t->ibus_a = adc_current_to_a(raw);

    if (BQ25798_Read16(BQ25798_REG_IBAT_ADC, &raw) == HAL_OK)
        t->ibat_a = adc_current_to_a(raw);

    if (BQ25798_Read16(BQ25798_REG_TDIE_ADC, &raw) == HAL_OK)
        t->die_temp_c = adc_temp_to_c(raw);

    return true;
}


/* ============================================================
 * READ BQ25798 INTERRUPT INFORMATION
 * ============================================================ */

bool BQ25798_ReadInterruptInfo(BQ25798_InterruptInfo *info)
{
    if (info == 0)
    {
        return false;
    }

    if (!BQ25798_IsConnected())
    {
        return false;
    }

    /*
     * Read current STATUS registers first.
     * These show the current state of the charger.
     */
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_0, &info->charger_status_0) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_1, &info->charger_status_1) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_2, &info->charger_status_2) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_3, &info->charger_status_3) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_STATUS_4, &info->charger_status_4) != HAL_OK) return false;

    if (BQ25798_Read8(BQ25798_REG_FAULT_STATUS_0, &info->fault_status_0) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_FAULT_STATUS_1, &info->fault_status_1) != HAL_OK) return false;

    /*
     * Read FLAG registers.
     * These show which source produced the INT pulse.
     *
     * Important:
     * The BQ25798 clears FLAG bits after the host reads them.
     * So read these once, store them, then decode from the stored struct.
     */
    if (BQ25798_Read8(BQ25798_REG_CHARGER_FLAG_0, &info->charger_flag_0) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_FLAG_1, &info->charger_flag_1) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_FLAG_2, &info->charger_flag_2) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_CHARGER_FLAG_3, &info->charger_flag_3) != HAL_OK) return false;

    if (BQ25798_Read8(BQ25798_REG_FAULT_FLAG_0, &info->fault_flag_0) != HAL_OK) return false;
    if (BQ25798_Read8(BQ25798_REG_FAULT_FLAG_1, &info->fault_flag_1) != HAL_OK) return false;

    return true;
}

/* ============================================================
 * PRINT DECODED BQ25798 INTERRUPT INFORMATION
 * ============================================================ */

void BQ25798_PrintInterruptInfo(const BQ25798_InterruptInfo *info)
{
    if (info == 0)
    {
        return;
    }

    printf("\n========== BQ25798 INTERRUPT INFO ==========\n");

    printf("STATUS REGISTERS:\n");
    printf("  CHARGER_STATUS_0 0x1B = 0x%02X\n", info->charger_status_0);
    printf("  CHARGER_STATUS_1 0x1C = 0x%02X\n", info->charger_status_1);
    printf("  CHARGER_STATUS_2 0x1D = 0x%02X\n", info->charger_status_2);
    printf("  CHARGER_STATUS_3 0x1E = 0x%02X\n", info->charger_status_3);
    printf("  CHARGER_STATUS_4 0x1F = 0x%02X\n", info->charger_status_4);
    printf("  FAULT_STATUS_0   0x20 = 0x%02X\n", info->fault_status_0);
    printf("  FAULT_STATUS_1   0x21 = 0x%02X\n", info->fault_status_1);

    printf("\nFLAG REGISTERS THAT CAUSED INT:\n");
    printf("  CHARGER_FLAG_0   0x22 = 0x%02X\n", info->charger_flag_0);
    printf("  CHARGER_FLAG_1   0x23 = 0x%02X\n", info->charger_flag_1);
    printf("  CHARGER_FLAG_2   0x24 = 0x%02X\n", info->charger_flag_2);
    printf("  CHARGER_FLAG_3   0x25 = 0x%02X\n", info->charger_flag_3);
    printf("  FAULT_FLAG_0     0x26 = 0x%02X\n", info->fault_flag_0);
    printf("  FAULT_FLAG_1     0x27 = 0x%02X\n", info->fault_flag_1);

    printf("\nDECODED INT EVENTS:\n");

    if (info->charger_flag_0 & BQ25798_FLAG0_IINDPM)
        printf("  IINDPM / IOTG regulation event\n");

    if (info->charger_flag_0 & BQ25798_FLAG0_VINDPM)
        printf("  VINDPM / VOTG regulation event\n");

    if (info->charger_flag_0 & BQ25798_FLAG0_WD)
        printf("  I2C watchdog timer expired\n");

    if (info->charger_flag_0 & BQ25798_FLAG0_POORSRC)
        printf("  Poor input source detected\n");

    if (info->charger_flag_0 & BQ25798_FLAG0_PG)
        printf("  Power-good status changed\n");

    if (info->charger_flag_0 & BQ25798_FLAG0_AC2_PRESENT)
        printf("  VAC2 present status changed\n");

    if (info->charger_flag_0 & BQ25798_FLAG0_AC1_PRESENT)
        printf("  VAC1 present status changed\n");

    if (info->charger_flag_0 & BQ25798_FLAG0_VBUS_PRESENT)
        printf("  VBUS present status changed\n");

    if (info->charger_flag_1 & BQ25798_FLAG1_CHG)
        printf("  Charge status changed\n");

    if (info->charger_flag_1 & BQ25798_FLAG1_ICO)
        printf("  ICO status changed\n");

    if (info->charger_flag_1 & BQ25798_FLAG1_VBUS)
        printf("  VBUS status changed\n");

    if (info->charger_flag_1 & BQ25798_FLAG1_TREG)
        printf("  Thermal regulation entered\n");

    if (info->charger_flag_1 & BQ25798_FLAG1_VBAT_PRESENT)
        printf("  Battery present status changed\n");

    if (info->charger_flag_1 & BQ25798_FLAG1_BC12_DONE)
        printf("  BC1.2 detection status changed\n");

    if (info->charger_flag_2 & BQ25798_FLAG2_DPDM_DONE)
        printf("  D+/D- detection completed\n");

    if (info->charger_flag_2 & BQ25798_FLAG2_ADC_DONE)
        printf("  ADC one-shot conversion completed\n");

    if (info->charger_flag_2 & BQ25798_FLAG2_VSYS)
        printf("  Entered or exited VSYSMIN regulation\n");

    if (info->charger_flag_2 & BQ25798_FLAG2_CHG_TMR)
        printf("  Fast charge safety timer expired\n");

    if (info->charger_flag_2 & BQ25798_FLAG2_TRICHG_TMR)
        printf("  Trickle charge safety timer expired\n");

    if (info->charger_flag_2 & BQ25798_FLAG2_PRECHG_TMR)
        printf("  Pre-charge safety timer expired\n");

    if (info->charger_flag_2 & BQ25798_FLAG2_TOPOFF_TMR)
        printf("  Top-off timer expired\n");

    if (info->charger_flag_3 & BQ25798_FLAG3_VBATOTG_LOW)
        printf("  Battery voltage too low for OTG\n");

    if (info->charger_flag_3 & BQ25798_FLAG3_TS_COLD)
        printf("  TS cold temperature detected\n");

    if (info->charger_flag_3 & BQ25798_FLAG3_TS_COOL)
        printf("  TS cool temperature detected\n");

    if (info->charger_flag_3 & BQ25798_FLAG3_TS_WARM)
        printf("  TS warm temperature detected\n");

    if (info->charger_flag_3 & BQ25798_FLAG3_TS_HOT)
        printf("  TS hot temperature detected\n");

    if (info->fault_flag_0 & BQ25798_FAULT0_IBAT_REG)
        printf("  IBAT regulation event\n");

    if (info->fault_flag_0 & BQ25798_FAULT0_VBUS_OVP)
        printf("  VBUS overvoltage detected\n");

    if (info->fault_flag_0 & BQ25798_FAULT0_VBAT_OVP)
        printf("  Battery overvoltage detected\n");

    if (info->fault_flag_0 & BQ25798_FAULT0_IBUS_OCP)
        printf("  IBUS overcurrent detected\n");

    if (info->fault_flag_0 & BQ25798_FAULT0_IBAT_OCP)
        printf("  IBAT overcurrent detected\n");

    if (info->fault_flag_0 & BQ25798_FAULT0_CONV_OCP)
        printf("  Converter overcurrent detected\n");

    if (info->fault_flag_0 & BQ25798_FAULT0_VAC2_OVP)
        printf("  VAC2 overvoltage detected\n");

    if (info->fault_flag_0 & BQ25798_FAULT0_VAC1_OVP)
        printf("  VAC1 overvoltage detected\n");

    if (info->fault_flag_1 & BQ25798_FAULT1_VSYS_SHORT)
        printf("  VSYS short circuit detected\n");

    if (info->fault_flag_1 & BQ25798_FAULT1_VSYS_OVP)
        printf("  VSYS overvoltage detected\n");

    if (info->fault_flag_1 & BQ25798_FAULT1_OTG_OVP)
        printf("  OTG overvoltage detected\n");

    if (info->fault_flag_1 & BQ25798_FAULT1_OTG_UVP)
        printf("  OTG undervoltage detected\n");

    if (info->fault_flag_1 & BQ25798_FAULT1_TSHUT)
        printf("  Thermal shutdown detected\n");

    if (info->charger_flag_0 == 0 &&
        info->charger_flag_1 == 0 &&
        info->charger_flag_2 == 0 &&
        info->charger_flag_3 == 0 &&
        info->fault_flag_0 == 0 &&
        info->fault_flag_1 == 0)
    {
        printf("  No latched INT flags found\n");
    }

    printf("==========================================\n\n");
}
