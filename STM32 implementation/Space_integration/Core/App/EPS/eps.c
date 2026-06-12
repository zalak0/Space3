#include "eps.h"
#include "hardware_chips/charger/bq25798.h"
#include "hardware_chips/charger/charger_manager.h"
#include "hardware_chips/bms/bq76920.h"
#include "common/fsw_ctx.h"
#include "stm/modes.h"

#define SOC_SAFE_ENTER          25.0f
#define SOC_SAFE_EXIT           30.0f

#define CELL_LOW_ENTER_V        3.00f
#define CELL_LOW_EXIT_V         3.30f

#define CELL_HIGH_ENTER_V       4.20f
#define CELL_HIGH_EXIT_V        4.10f

#define PACK_EMPTY_V            12.0f
#define PACK_FULL_V             16.8f

void eps_init(fsw_ctx_t *ctx){
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

void eps_request_telemetry(fsw_ctx_t *ctx){
    if (ctx == 0)
        return;

    ctx->eps_telem_request = 1U;
}

void eps_task(fsw_ctx_t *ctx){
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
