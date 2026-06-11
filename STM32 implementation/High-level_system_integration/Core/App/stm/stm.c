#include "stm/stm.h"
#include "common/goose_config.h"
#include "bq25798.h"
#include "charger_manager.h"
#include <math.h>

static float rate_norm(const adcs_state_t *x)
{
    return sqrtf((x->phi_dot   * x->phi_dot) +
                 (x->theta_dot * x->theta_dot) +
                 (x->psi_dot   * x->psi_dot));
}

static int pointing_converged(const adcs_state_t *x)
{
    float e = sqrtf((x->phi   * x->phi) +
                    (x->theta * x->theta) +
                    (x->psi   * x->psi));

    return x->valid && (e < POINT_CONVERGED_RAD);
}

void sm_eps_init(fsw_ctx_t *ctx)
{
    uint8_t part_info = 0;
    uint8_t adc_cfg = 0xC0;
    uint8_t reg15 = 0;

    if (ctx == 0)
        return;

    ctx->charger_present = 0U;
    ctx->charger_configured = 0U;
    ctx->eps_i2c_ok = 0U;
    ctx->eps_telem_valid = 0U;
    ctx->eps_telem_request = 0U;

    if (BQ25798_Read8(0x48, &part_info))
    {
        ctx->part_info = part_info;
        ctx->charger_present = 1U;
        ctx->eps_i2c_ok = 1U;
    }
    else
    {
        ctx->eps_i2c_ok = 0U;
        return;
    }

    if (ChargerManager_Init())
        ctx->charger_configured = 1U;
    else
        ctx->charger_configured = 0U;

    BQ25798_Write8(BQ25798_REG_ADC_CONTROL, adc_cfg);

    if (BQ25798_Read8(0x15, &reg15))
    {
        reg15 |= (1U << 5);
        BQ25798_Write8(0x15, reg15);
    }
}

void sm_eps_request_telemetry(fsw_ctx_t *ctx)
{
    if (ctx == 0)
        return;

    ctx->eps_telem_request = 1U;
}

void sm_eps_periodic_task(fsw_ctx_t *ctx)
{
    BQ25798_Telemetry telem = {0};

    if (ctx == 0)
        return;

    if (!ctx->eps_telem_request)
        return;

    ctx->eps_telem_request = 0U;

    if (BQ25798_ReadTelemetry(&telem))
    {
        ctx->vbus_v = telem.vbus_v;
        ctx->vbat_v = telem.vbat_v;
        ctx->vsys_v = telem.vsys_v;

        ctx->ibus_a = telem.ibus_a;
        ctx->ibat_a = telem.ibat_a;
        ctx->eps_die_temp_c = telem.die_temp_c;

        ctx->charger_status_0 = telem.charger_status_0;
        ctx->charger_status_1 = telem.charger_status_1;
        ctx->charger_pg = (telem.charger_status_0 & 0x08U) ? 1U : 0U;

        ctx->eps_last_update_ms = HAL_GetTick();
        ctx->eps_telem_valid = 1U;
        ctx->eps_i2c_ok = 1U;
    }
    else
    {
        ctx->eps_telem_valid = 0U;
        ctx->eps_i2c_ok = 0U;
    }
}

sat_mode_t sm_update(const fsw_ctx_t *ctx, sat_mode_t mode)
{
    if (ctx == 0)
        return MODE_SAFE;

    if (ctx->soc_low)
        return MODE_SAFE;

    switch (mode)
    {
        case MODE_SAFE:
            if (rate_norm(&ctx->est) < OMEGA_CAPTURE)
                return MODE_NOMINAL;
            return MODE_DETUMBLE;

        case MODE_NOMINAL:
            if (rate_norm(&ctx->est) > OMEGA_CAPTURE)
                return MODE_DETUMBLE;

            if (ctx->ground_contact || ctx->science_due)
                return MODE_POINTING;

            break;

        case MODE_DETUMBLE:
            if (rate_norm(&ctx->est) < OMEGA_CAPTURE)
                return MODE_NOMINAL;
            break;

        case MODE_POINTING:
            if (pointing_converged(&ctx->est) && ctx->science_due)
                return MODE_SCIENCE;

            if (ctx->ground_contact && ctx->downlink_due)
                return MODE_DOWNLINK;

            if (!ctx->science_due && !ctx->ground_contact)
                return MODE_NOMINAL;

            break;

        case MODE_SCIENCE:
            if (!ctx->science_due)
                return MODE_NOMINAL;

            if (ctx->ground_contact && ctx->downlink_due)
                return MODE_DOWNLINK;

            break;

        case MODE_DOWNLINK:
            if (!ctx->ground_contact)
                return MODE_POINTING;

            if (ctx->uplink_pending)
                return MODE_UPLINK;

            break;

        case MODE_UPLINK:
            if (!ctx->ground_contact || ctx->uplink_done)
                return MODE_NOMINAL;

            break;

        default:
            return MODE_SAFE;
    }

    return mode;
}