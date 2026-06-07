#ifndef STM_H
#define STM_H

#include "stm/modes.h"
#include "common/fsm_ctx.h"      /* defines fsw_ctx_t, adcs_state_t */

sat_mode_t sm_update(const fsw_ctx_t *ctx, sat_mode_t mode);

#endif
