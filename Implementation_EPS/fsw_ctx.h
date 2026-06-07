#ifndef FSW_CTX_H
#define FSW_CTX_H

#include <stdint.h>

#include "STM32_dev/Charger_Code/bq25798.h"
#include "STM32_dev/BMS_code/bq76920.h"

typedef enum {
    EPS_MODE_SAFE = 0,
    EPS_MODE_NOMINAL,
    EPS_MODE_SCIENCE
} eps_mode_t;
typedef struct fsw_ctx_t {


    uint8_t soc_low;
    uint8_t science_window;
    uint8_t ground_contact;

    uint8_t eps_ok;
    uint8_t eps_fault;
    uint8_t charger_ok;
    uint8_t battery_ok;

    uint8_t battery_low;
    uint8_t battery_high;

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

#endif