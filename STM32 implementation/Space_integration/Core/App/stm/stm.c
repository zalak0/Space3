#include "stm.h"
#include "goose_config.h"
#include <math.h>

static float rate_norm(const adcs_state_t *x) {
    return sqrtf(x->phi_dot*x->phi_dot + x->theta_dot*x->theta_dot
               + x->psi_dot*x->psi_dot);
}
static int pointing_converged(const adcs_state_t *x) {
    float e = sqrtf(x->phi*x->phi + x->theta*x->theta + x->psi*x->psi);
    return x->valid && (e < POINT_CONVERGED_RAD);   /* e.g. 2 deg in rad */
}

sat_mode_t sm_update(const fsw_ctx_t *ctx, sat_mode_t mode) {
    /* highest priority: SoC safety gate overrides every mode */
    if (ctx->soc_low)
    	return MODE_SAFE;

    switch (mode) {
		case MODE_SAFE:
			// If attitude is stable and SOC > threshold
		    if (rate_norm(&ctx->est) < OMEGA_CAPTURE) {
		    	return MODE_NOMINAL;
		    }
		    else {
		    	return MODE_DETUMBLE;
		    }
			break;

        case MODE_NOMINAL:
        	// De-tumble satellite if attitude unstable
            if (rate_norm(&ctx->est) > OMEGA_CAPTURE){
            	return MODE_DETUMBLE;
            }

            if (ctx->ground_contact || sm_science_due()) {
            	return MODE_POINTING;
            }

            break;

        case MODE_DETUMBLE:
        	if (rate_norm(&ctx->est) < OMEGA_CAPTURE){
				return MODE_NOMINAL;
			}
            break;

        case MODE_POINTING:
            if (pointing_converged(&ctx->est) && sm_science_due()) {
            	return MODE_SCIENCE;
            }
            if (ctx->ground_contact && sm_downlink_due()) {
            	return MODE_DOWNLINK;
            }
            if (!ctx->science_due && !ctx->ground_contact) {
            	return MODE_NOMINAL;  /* fallback */
            }
            break;
            break;

        case MODE_SCIENCE:
            if (!sm_science_due())  {
            	return MODE_NOMINAL;
            }
            if (ctx->ground_contact && sm_downlink_due()) {
            	return MODE_DOWNLINK;
            }
            break;

        case MODE_DOWNLINK:
            if (!ctx->ground_contact){
            	return MODE_POINTING;
            }
            if (sm_uplink_pending()){
            	return MODE_UPLINK;
            }
            break;

        case MODE_UPLINK:
            if (!ctx->ground_contact || sm_uplink_done()) {
            	return MODE_NOMINAL;
            }
            break;

        default:
        	return MODE_SAFE;
    }
    return mode;   /* no guard fired -> stay */
}
