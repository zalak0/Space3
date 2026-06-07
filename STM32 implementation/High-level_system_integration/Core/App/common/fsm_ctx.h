#ifndef FSW_CTX_H
#define FSW_CTX_H
#include <stdint.h>
#include "common/adcs_types.h"

typedef struct {
    /* attitude pipeline:  ADS writes  ->  ACS reads */
    adcs_state_t est;          /* attitude error estimate, LVLH       */
    float        B_lvlh[3];    /* geomagnetic field, LVLH    [Tesla]  */
    float        B_body[3];    /* raw magnetometer, body     [Tesla]  */

    // Task scheduler flags
    uint8_t science_due;       /* scheduler (orbit/ground-pass schedule) writes */
    /* cross-subsystem flags (producer noted) */
    uint8_t soc_low;           /* EPS writes     -> SM reads (gate)   */
    uint8_t science_window;    /* payload writes -> ACS reads         */
    uint8_t ground_contact;    /* comms writes   -> SM reads          */

    /* fsm_ctx.h additions — each owned by ONE producer, written before STM reads */
    uint8_t deploy_elapsed;    /* scheduler/boot-timer writes */

    // Comms flags
    uint8_t downlink_due;      /* scheduler writes */
    uint8_t uplink_pending;    /* comms writes — it knows its rx/command-queue state */
    uint8_t uplink_done;       /* comms writes */


} fsw_ctx_t;
#endif
