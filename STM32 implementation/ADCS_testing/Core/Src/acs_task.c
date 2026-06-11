#include "adcs_types.h"
#include "acs_task.h"
#include "modes.h"
#include "fsw_ctx.h"

#include "aplqr.h"
#include "bdot.h"
#include "magnetorquer.h"

#include <stdint.h>

uint8_t dt_ctrl =  0.1f;

void acs_task(sat_mode_t mode, fsw_ctx_t *ctx)
{
    const adcs_state_t *x = &ctx->est;
    adcs_dipole_t m = {0,0,0};

    switch (mode) {
		case MODE_DETUMBLE:
			static bdot_state_t bd;          /* persists across superloop cycles */
			static int bd_init = 0;
			if (!bd_init){
				bdot_init(&bd);
				bd_init=1;
			}
			bdot_step(&bd, ctx->B_body, dt_ctrl, &m);   /* DT_CTRL_S = 0.1f */
			torquer_apply(&m);
			break;

        case MODE_POINTING:
            aplqr_step(x, ctx->B_lvlh, &m);
            torquer_apply(&m);
            break;

        default:
            drv8833_coast_all();
            break;
    }
}
