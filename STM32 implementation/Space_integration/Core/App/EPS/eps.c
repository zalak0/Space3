#include "main.h"
#include "eps.h"

#include "../Hardware_chips/bms/bq76920.h"
#include "../Hardware_chips/charger/bq25798.h"

#define SOC_SAFE_ENTER          25.0f
#define SOC_SAFE_EXIT           30.0f

#define CELL_LOW_ENTER_V        3.00f
#define CELL_LOW_EXIT_V         3.30f

#define CELL_HIGH_ENTER_V       4.20f
#define CELL_HIGH_EXIT_V        4.10f

#define PACK_EMPTY_V            12.0f
#define PACK_FULL_V             16.8f

static void eps_disable_3v3_rail(void)
{
    HAL_GPIO_WritePin(EPS_3V3_DISABLE_GPIO_Port, EPS_3V3_DISABLE_Pin, GPIO_PIN_SET);
}

static void eps_disable_5v_rail(void)
{
    HAL_GPIO_WritePin(EPS_5V_DISABLE_GPIO_Port, EPS_5V_DISABLE_Pin, GPIO_PIN_SET);
}

static void eps_disable_vbus_rail(void)
{
    HAL_GPIO_WritePin(EPS_VBUS_DISABLE_GPIO_Port, EPS_VBUS_DISABLE_Pin, GPIO_PIN_SET);
}

static void eps_enable_3v3_rail(void)
{
    HAL_GPIO_WritePin(EPS_3V3_DISABLE_GPIO_Port, EPS_3V3_DISABLE_Pin, GPIO_PIN_RESET);
}

static void eps_enable_5v_rail(void)
{
    HAL_GPIO_WritePin(EPS_5V_DISABLE_GPIO_Port, EPS_5V_DISABLE_Pin, GPIO_PIN_RESET);
}

static void eps_enable_vbus_rail(void)
{
    HAL_GPIO_WritePin(EPS_VBUS_DISABLE_GPIO_Port, EPS_VBUS_DISABLE_Pin, GPIO_PIN_RESET);
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

static float eps_estimate_soc(float pack_voltage_v)
{
    float soc = ((pack_voltage_v - PACK_EMPTY_V) /
                 (PACK_FULL_V - PACK_EMPTY_V)) * 100.0f;

    if (soc < 0.0f) {
        soc = 0.0f;
    }

    if (soc > 100.0f) {
        soc = 100.0f;
    }

    return soc;
}

static void eps_apply_individual_fault_shutdown(fsw_ctx_t *ctx)
{
    if (ctx->rail_3v3_flt || !ctx->rail_3v3_pg) {
        eps_disable_3v3_rail();
    }

    if (ctx->rail_5v_flt || !ctx->rail_5v_pg) {
        eps_disable_5v_rail();
    }

    if (ctx->vbus_flt || !ctx->vbus_pg) {
        eps_disable_vbus_rail();
    }
}

static void eps_apply_battery_limits(fsw_ctx_t *ctx)
{
    if (ctx->battery.cell_max_v >= CELL_HIGH_ENTER_V) {
        ctx->battery_high = 1;
        BQ25798_EnableChargerHardware(false);
    } else if (ctx->battery.cell_max_v <= CELL_HIGH_EXIT_V) {
        ctx->battery_high = 0;
        BQ25798_EnableChargerHardware(true);
    }

    if (ctx->battery.cell_min_v <= CELL_LOW_ENTER_V) {
        ctx->battery_low = 1;
        ctx->soc_low = 1;
        eps_shed_nonessential();
    } else if (ctx->battery.cell_min_v >= CELL_LOW_EXIT_V) {
        ctx->battery_low = 0;
    }
}

bool eps_init(fsw_ctx_t *ctx)
{
    bool charger_ok;
    bool battery_ok;

    if (ctx == 0) {
        return false;
    }

    ctx->charger_ok = 0;
    ctx->battery_ok = 0;
    ctx->eps_ok = 0;
    ctx->eps_fault = 0;
    ctx->soc_low = 0;
    ctx->battery_low = 0;
    ctx->battery_high = 0;

    eps_shed_nonessential();

    charger_ok = BQ25798_InitCharger4S();
    battery_ok = BQ76920_Init();

    ctx->charger_ok = charger_ok;
    ctx->battery_ok = battery_ok;
    ctx->eps_ok = charger_ok && battery_ok;
    ctx->eps_fault = !ctx->eps_ok;

    if (!ctx->eps_ok) {
        ctx->soc_low = 1;
        eps_shed_nonessential();
        return false;
    }

    return true;
}

void eps_task(sat_mode_t mode, fsw_ctx_t *ctx)
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
        ctx->vbus_flt) {

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

    eps_apply_battery_limits(ctx);

    if (ctx->battery.fault_ov ||
        ctx->battery.fault_uv ||
        ctx->battery.fault_ocd ||
        ctx->battery.fault_scd ||
        ctx->battery.fault_device_xready ||
        ctx->charger.fault_status_0 ||
        ctx->charger.fault_status_1) {

        ctx->eps_fault = 1;
        ctx->eps_ok = 0;

        if (ctx->battery.fault_ov) {
            ctx->battery_high = 1;
            BQ25798_EnableChargerHardware(false);
        }

        if (ctx->battery.fault_uv ||
            ctx->battery.fault_ocd ||
            ctx->battery.fault_scd ||
            ctx->battery.fault_device_xready) {

            ctx->battery_low = 1;
            ctx->soc_low = 1;
            eps_shed_nonessential();
        }

        return;
    }

    ctx->eps_fault = 0;

    switch (mode) {
        case EPS_MODE_SAFE:
            eps_shed_nonessential();
            break;

        case EPS_MODE_SCIENCE:
            if (!ctx->soc_low && !ctx->battery_low && !ctx->eps_fault) {
                eps_enable_payload_rail();
            } else {
                eps_shed_nonessential();
            }
            break;

        default:
            if (!ctx->soc_low && !ctx->battery_low && !ctx->eps_fault) {
                eps_nominal_rails();
            } else {
                eps_shed_nonessential();
            }
            break;
    }
}
