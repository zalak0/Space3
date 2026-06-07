#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "bq25798.h"
#include "charger_manager.h"

/* Fake HAL helper functions from fake_hal_i2c.c */
void FakeBQ25798_Reset(void);
void FakeBQ25798_Set8(uint8_t reg, uint8_t value);
void FakeBQ25798_Set16(uint8_t reg, uint16_t value);

static uint16_t read_fake_16(uint8_t reg)
{
    uint16_t value = 0;

    assert(BQ25798_Read16(reg, &value) == HAL_OK);

    return value;
}

static uint8_t read_fake_8(uint8_t reg)
{
    uint8_t value = 0;

    assert(BQ25798_Read8(reg, &value) == HAL_OK);

    return value;
}

int main(void)
{
    FakeBQ25798_Reset();

    assert(BQ25798_IsConnected() == true);

    assert(ChargerManager_Init() == true);

    /*
        Expected config from charger_manager.c default_config:

        charge_voltage_mV       = 16800
        charge_current_mA       = 1000
        input_voltage_limit_mV  = 12000
        input_current_limit_mA  = 500
        min_system_voltage_mV   = 12000
        charging enabled        = true
        termination enabled     = true
        watchdog disabled       = true
    */

    uint8_t vsysmin_reg = read_fake_8(BQ25798_REG_VSYSMIN);
    uint16_t vreg_reg   = read_fake_16(BQ25798_REG_CHARGE_VOLTAGE);
    uint16_t ichg_reg   = read_fake_16(BQ25798_REG_CHARGE_CURRENT);
    uint8_t vindpm_reg  = read_fake_8(BQ25798_REG_INPUT_VOLTAGE_LIMIT);
    uint16_t iindpm_reg = read_fake_16(BQ25798_REG_INPUT_CURRENT_LIMIT);

    uint8_t chg_ctrl_0  = read_fake_8(BQ25798_REG_CHARGER_CONTROL_0);
    uint8_t chg_ctrl_1  = read_fake_8(BQ25798_REG_CHARGER_CONTROL_1);
    uint8_t adc_ctrl    = read_fake_8(BQ25798_REG_ADC_CONTROL);

    printf("VSYSMIN register      = 0x%02X\n", vsysmin_reg);
    printf("Charge voltage reg    = %u\n", vreg_reg);
    printf("Charge current reg    = %u\n", ichg_reg);
    printf("Input voltage reg     = %u\n", vindpm_reg);
    printf("Input current reg     = %u\n", iindpm_reg);
    printf("Charger control 0     = 0x%02X\n", chg_ctrl_0);
    printf("Charger control 1     = 0x%02X\n", chg_ctrl_1);
    printf("ADC control           = 0x%02X\n", adc_ctrl);

    /*
        Conversion checks based on our bq25798.c formulas.
    */

    assert(vsysmin_reg == ((12000 - 2500) / 250));
    assert(vreg_reg == (16800 / 10));
    assert(ichg_reg == (1000 / 10));
    assert(vindpm_reg == (12000 / 100));
    assert(iindpm_reg == (500 / 10));

    assert((chg_ctrl_0 & BQ25798_EN_CHG_BIT) != 0);
    assert((chg_ctrl_0 & BQ25798_EN_TERM_BIT) != 0);

    assert((chg_ctrl_1 & BQ25798_WATCHDOG_MASK) == BQ25798_WATCHDOG_DISABLED);

    assert((adc_ctrl & 0x80) != 0);

    /*
        Test telemetry readback using fake ADC registers.
    */

    FakeBQ25798_Set16(BQ25798_REG_VBUS_ADC, 18000);
    FakeBQ25798_Set16(BQ25798_REG_VBAT_ADC, 14800);
    FakeBQ25798_Set16(BQ25798_REG_VSYS_ADC, 14750);
    FakeBQ25798_Set16(BQ25798_REG_IBUS_ADC, 450);
    FakeBQ25798_Set16(BQ25798_REG_IBAT_ADC, 380);
    FakeBQ25798_Set16(BQ25798_REG_TDIE_ADC, 50);

    BQ25798_Telemetry t = {0};;

    assert(ChargerManager_ReadTelemetry(&t) == true);

    printf("\nTelemetry:\n");
    printf("VBUS = %.3f V\n", telemetry.vbus_v);
    printf("VBAT = %.3f V\n", telemetry.vbat_v);
    printf("VSYS = %.3f V\n", telemetry.vsys_v);
    printf("IBUS = %.3f A\n", telemetry.ibus_a);
    printf("IBAT = %.3f A\n", telemetry.ibat_a);
    printf("TDIE = %.3f C\n", telemetry.die_temp_c);

    assert(telemetry.vbus_v > 17.99f && telemetry.vbus_v < 18.01f);
    assert(telemetry.vbat_v > 14.79f && telemetry.vbat_v < 14.81f);
    assert(telemetry.vsys_v > 14.74f && telemetry.vsys_v < 14.76f);
    assert(telemetry.ibus_a > 0.44f && telemetry.ibus_a < 0.46f);
    assert(telemetry.ibat_a > 0.37f && telemetry.ibat_a < 0.39f);

    /*
        Test watchdog service.
        Since watchdog is disabled in default config, this only proves the register write path works.
    */

    assert(ChargerManager_ServiceWatchdog() == true);

    chg_ctrl_1 = read_fake_8(BQ25798_REG_CHARGER_CONTROL_1);

    assert((chg_ctrl_1 & BQ25798_WD_RST_BIT) != 0);

    printf("\nCharger manager test passed\n");

    return 0;
}