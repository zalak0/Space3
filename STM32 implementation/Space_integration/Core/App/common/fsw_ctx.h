#ifndef FSW_CTX_H
#define FSW_CTX_H
#include <stdint.h>
#include "common/adcs_types.h"

typedef struct {
    /* attitude pipeline:  ADS writes  ->  ACS reads */
    adcs_state_t est;          /* attitude error estimate, LVLH       */
    float        B_lvlh[3];    /* geomagnetic field, LVLH    [Tesla]  */
    float        B_body[3];    /* raw magnetometer, body     [Tesla]  */

    /* EPS SUBSYSTEM CONTEXT */
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

    /* PAYLOAD CONTEXTS */
    uint8_t has_burned;

    // Comms flags
    uint8_t downlink_due;      /* scheduler writes */
    uint8_t uplink_pending;    /* comms writes — it knows its rx/command-queue state */
    uint8_t uplink_done;       /* comms writes */


} fsw_ctx_t;
#endif
