#ifndef FSW_CTX_H
#define FSW_CTX_H

#include <stdint.h>
#include "common/adcs_types.h"

typedef struct {
    /* attitude pipeline: ADS writes -> ACS reads */
    adcs_state_t est;
    float        B_lvlh[3];
    float        B_body[3];

    /* scheduler flags */
    uint8_t science_due;
    uint8_t deploy_elapsed;

    /* EPS writes -> STM / COMMS reads */
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

    /* payload / comms flags */
    uint8_t science_window;
    uint8_t ground_contact;

    uint8_t downlink_due;
    uint8_t uplink_pending;
    uint8_t uplink_done;

} fsw_ctx_t;

#endif