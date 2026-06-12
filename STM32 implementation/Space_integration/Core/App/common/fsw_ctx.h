#ifndef FSW_CTX_H
#define FSW_CTX_H
#include <hardware_chips/bms/bq76920.h>
#include <hardware_chips/charger/bq25798.h>
#include <stdint.h>
#include "adcs_types.h"

typedef struct {
    /* attitude pipeline:  ADS writes  ->  ACS reads */
    adcs_state_t est;          /* attitude error estimate, LVLH       */
    float        B_lvlh[3];    /* geomagnetic field, LVLH    [Tesla]  */
    float        B_body[3];    /* raw magnetometer, body     [Tesla]  */


    /* EPS SUBSYSTEM CONTEXT */
    uint8_t soc_low;

    float vbus_v;
    float vbat_v;
    float vsys_v;
    float ibus_a;
    float ibat_a;
    float eps_die_temp_c;

    uint8_t charger_pg;
    uint8_t charger_status_0;
    uint8_t charger_status_1;

    uint8_t part_info;
    uint8_t charger_present;
    uint8_t charger_configured;
    uint8_t eps_i2c_ok;

    uint32_t eps_last_update_ms;
    uint8_t  eps_telem_valid;
    volatile uint8_t eps_telem_request;

    /* COMMS CONTEXTS */
    uint8_t ground_contact;
    uint8_t science_due;

    // Comms flags
    uint8_t downlink_due;      /* scheduler writes */
    uint8_t uplink_pending;    /* comms writes — it knows its rx/command-queue state */
    uint8_t uplink_done;       /* comms writes */


} fsw_ctx_t;
#endif
