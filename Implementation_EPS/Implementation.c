#include <stdint.h>
#include <stdbool.h>

#include "common/adcs_types.h"
#include "subsystems/eps/eps.h"
#include "subsystems/eps/charger_manager.h"
#include "subsystems/eps/bq25798.h"
#include "subsystems/eps/bq76920.h"

#define SOC_SAFE_ENTER  25.0f
#define SOC_SAFE_EXIT   30.0f

/*
 * CHANGE GPIOx TO THE ACTUAL STM32 GPIO PORT.
 *
 * GPIO20 = 3V3 disable control
 * GPIO19 = VBUS disable control
 * GPIO18 = 5V disable control
 *
 * GPIO SET   = NMOS on  = TPS259813 EN shorted to GND = rail OFF
 * GPIO RESET = NMOS off = TPS259813 EN pulled up       = rail ON
 */
#define RAIL_3V3_DISABLE_GPIO_Port   GPIOx
#define RAIL_3V3_DISABLE_Pin         GPIO_PIN_20

#define VBUS_DISABLE_GPIO_Port       GPIOx
#define VBUS_DISABLE_Pin             GPIO_PIN_19

#define RAIL_5V_DISABLE_GPIO_Port    GPIOx
#define RAIL_5V_DISABLE_Pin          GPIO_PIN_18

typedef struct {
    adcs_state_t est;
    float B_lvlh[3];
    float B_body[3];

    uint8_t soc_low;
    uint8_t science_window;
    uint8_t ground_contact;

    uint8_t eps_ok;
    uint8_t eps_fault;
    uint8_t charger_ok;
    uint8_t battery_ok;

    float soc_percent;
    float pack_voltage_v;
    float vbat_v;
    float vsys_v;
    float ibat_a;

    BQ25798_Telemetry charger;
    BQ76920_Telemetry battery;

    uint8_t rail_3v3_pg;
    uint8_t rail_3v3_flt;
    uint8_t rail_5v_pg;
    uint8_t rail_5v_flt;
    uint8_t vbus_flt;
    uint8_t vbus_pg;

} fsw_ctx_t;

static void eps_disable_3v3_rail(void)
{
    HAL_GPIO_WritePin(
        RAIL_3V3_DISABLE_GPIO_Port,
        RAIL_3V3_DISABLE_Pin,
        GPIO_PIN_SET
    );
}

static void eps_disable_5v_rail(void)
{
    HAL_GPIO_WritePin(
        RAIL_5V_DISABLE_GPIO_Port,
        RAIL_5V_DISABLE_Pin,
        GPIO_PIN_SET
    );
}

static void eps_disable_vbus_rail(void)
{
    HAL_GPIO_WritePin(
        VBUS_DISABLE_GPIO_Port,
        VBUS_DISABLE_Pin,
        GPIO_PIN_SET
    );
}

static void eps_enable_3v3_rail(void)
{
    HAL_GPIO_WritePin(
        RAIL_3V3_DISABLE_GPIO_Port,
        RAIL_3V3_DISABLE_Pin,
        GPIO_PIN_RESET
    );
}

static void eps_enable_5v_rail(void)
{
    HAL_GPIO_WritePin(
        RAIL_5V_DISABLE_GPIO_Port,
        RAIL_5V_DISABLE_Pin,
        GPIO_PIN_RESET
    );
}

static void eps_enable_vbus_rail(void)
{
    HAL_GPIO_WritePin(
        VBUS_DISABLE_GPIO_Port,
        VBUS_DISABLE_Pin,
        GPIO_PIN_RESET
    );
}

void eps_shed_nonessential(void)
{
    eps_disable_3v3_rail();
    eps_disable_vbus_rail();
    eps_disable_5v_rail();
}

void eps_nominal_rails(void)
{
    eps_enable_3v3_rail();
    eps_enable_vbus_rail();
    eps_enable_5v_rail();
}

void eps_enable_payload_rail(void)
{
    eps_nominal_rails();
}

static void eps_apply_individual_fault_shutdown(fsw_ctx_t *ctx)
{
    if (ctx == 0) {
        return;
    }

    /*
     * Individual rail protection.
     *
     * If a specific FLT line is active, only that rail is turned off.
     *
     * FLT stored logic:
     * 1 = fault active
     * 0 = no fault
     */

    if (ctx->rail_3v3_flt) {
        eps_disable_3v3_rail();
    }

    if (ctx->rail_5v_flt) {
        eps_disable_5v_rail();
    }

    if (ctx->vbus_flt) {
        eps_disable_vbus_rail();
    }

    /*
     * If PG drops, also turn off that individual rail.
     *
     * PG stored logic:
     * 1 = power good
     * 0 = power not good
     */

    if (!ctx->rail_3v3_pg) {
        eps_disable_3v3_rail();
    }

    if (!ctx->rail_5v_pg) {
        eps_disable_5v_rail();
    }

    if (!ctx->vbus_pg) {
        eps_disable_vbus_rail();
    }
}

bool eps_init(fsw_ctx_t *ctx)
{
    bool ok;

    if (ctx == 0) {
        return false;
    }

    ctx->charger_ok = 0;
    ctx->battery_ok = 0;
    ctx->eps_ok = 0;
    ctx->eps_fault = 0;
    ctx->soc_low = 0;

    ctx->rail_3v3_pg = 0;
    ctx->rail_3v3_flt = 0;
    ctx->rail_5v_pg = 0;
    ctx->rail_5v_flt = 0;
    ctx->vbus_flt = 0;
    ctx->vbus_pg = 0;

    eps_shed_nonessential();

    ok = BQ25798_IsConnected();

    if (!ok) {
        ctx->charger_ok = 0;
        ctx->eps_ok = 0;
        ctx->eps_fault = 1;
        ctx->soc_low = 1;
        return false;
    }

    BQ25798_SetWatchdog(BQ25798_WATCHDOG_DISABLED);
    BQ25798_EnableADC(true);

    BQ25798_SetMinSystemVoltage_mV(12000);
    BQ25798_SetChargeVoltage_mV(16800);
    BQ25798_SetChargeCurrent_mA(500);
    BQ25798_SetInputVoltageLimit_mV(5000);
    BQ25798_SetInputCurrentLimit_mA(1000);

    ok = BQ25798_EnableChargerHardware(true);

    ctx->charger_ok = ok;
    ctx->eps_ok = ok;
    ctx->eps_fault = !ok;

    if (!ok) {
        ctx->soc_low = 1;
        eps_shed_nonessential();
        return false;
    }

    return true;
}

static float eps_estimate_soc(float pack_voltage_v)
{
    const float empty_v = 12.0f;
    const float full_v = 16.8f;

    float soc = ((pack_voltage_v - empty_v) / (full_v - empty_v)) * 100.0f;

    if (soc < 0.0f) {
        soc = 0.0f;
    }

    if (soc > 100.0f) {
        soc = 100.0f;
    }

    return soc;
}

void eps_task(adcs_mode_t mode, fsw_ctx_t *ctx)
{
    bool charger_ok;
    bool battery_ok;

    if (ctx == 0) {
        return;
    }

    BQ25798_ReadRailGPIOStatus(ctx);

    eps_apply_individual_fault_shutdown(ctx);

    if (ctx->rail_3v3_flt ||
        ctx->rail_5v_flt ||
        ctx->vbus_flt ||
        !ctx->rail_3v3_pg ||
        !ctx->rail_5v_pg ||
        !ctx->vbus_pg) {

        ctx->eps_fault = 1;
        ctx->eps_ok = 0;
        return;
    }

    charger_ok = BQ25798_ReadTelemetry(&ctx->charger);
    battery_ok = BQ76920_ReadTelemetry(&ctx->battery);

    ctx->charger_ok = charger_ok;
    ctx->battery_ok = battery_ok;
    ctx->eps_ok = charger_ok && battery_ok;

    if (!ctx->eps_ok) {
        ctx->eps_fault = 1;
        ctx->soc_low = 1;
        eps_shed_nonessential();
        return;
    }

    ctx->eps_fault = 0;

    ctx->pack_voltage_v = ctx->battery.pack_voltage_v;
    ctx->vbat_v = ctx->charger.vbat_v;
    ctx->vsys_v = ctx->charger.vsys_v;
    ctx->ibat_a = ctx->charger.ibat_a;

    ctx->soc_percent = eps_estimate_soc(ctx->battery.pack_voltage_v);

    if (ctx->soc_percent < SOC_SAFE_ENTER) {
        ctx->soc_low = 1;
    } else if (ctx->soc_percent > SOC_SAFE_EXIT) {
        ctx->soc_low = 0;
    }

    if (ctx->battery.fault_ov ||
        ctx->battery.fault_uv ||
        ctx->battery.fault_ocd ||
        ctx->battery.fault_scd ||
        ctx->battery.fault_device_xready ||
        ctx->charger.fault_status_0 ||
        ctx->charger.fault_status_1) {

        ctx->eps_fault = 1;
        ctx->soc_low = 1;
        ctx->eps_ok = 0;
        eps_shed_nonessential();
        return;
    }

    switch (mode) {
        case MODE_SAFE:
            eps_shed_nonessential();
            break;

        case MODE_SCIENCE:
            if (!ctx->soc_low && !ctx->eps_fault) {
                eps_enable_payload_rail();
            } else {
                eps_shed_nonessential();
            }
            break;

        default:
            if (!ctx->soc_low && !ctx->eps_fault) {
                eps_nominal_rails();
            } else {
                eps_shed_nonessential();
            }
            break;
    }
}