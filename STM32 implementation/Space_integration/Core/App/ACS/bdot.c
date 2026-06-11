#include "bdot.h"
#include <math.h>

/* match your MATLAB: k_bdot = 2*omega_o*(1+sin(inc))*min(I) */
static const float K_BDOT  = 1.467e-05f;   /* <-- compute from your constants, see note */
static const float M_MAX   = 0.046f;    /* per-axis dipole limit, A*m^2 */

void bdot_init(bdot_state_t *s)
{
    s->B_prev[0] = s->B_prev[1] = s->B_prev[2] = 0.f;
    s->primed = 0;
}

void bdot_step(bdot_state_t *s, const float B_body[3], float dt, adcs_dipole_t *m)
{
    if (!s->primed) {                 /* first call: no derivative yet, command zero */
        s->B_prev[0]=B_body[0]; s->B_prev[1]=B_body[1]; s->B_prev[2]=B_body[2];
        s->primed = 1;
        m->mx = m->my = m->mz = 0.f;
        return;
    }

    /* Bdot = (B - B_prev)/dt */
    float Bdot[3] = {
        (B_body[0]-s->B_prev[0])/dt,
        (B_body[1]-s->B_prev[1])/dt,
        (B_body[2]-s->B_prev[2])/dt
    };

    /* normalise by |B|^2  (the factor-of-|B| error you flagged before) */
    float B2 = B_body[0]*B_body[0]+B_body[1]*B_body[1]+B_body[2]*B_body[2];
    if (B2 < 1e-18f) {
    	B2 = 1e-18f;     /* guard, == your max(.,eps) */
    }

    float u[3] = {
        -K_BDOT * Bdot[0] / B2,
        -K_BDOT * Bdot[1] / B2,
        -K_BDOT * Bdot[2] / B2
    };

    /* proportional (direction-preserving) saturation — same as APLQR */
    float beta = 0.f;
    for (int i=0;i<3;i++){
    	float a = fabsf(u[i])/M_MAX;
    	if (a>beta) {
    		beta=a;
    	}
    }
    float sc = (beta>1.f) ? 1.f/beta : 1.f;

    m->mx = u[0]*sc;
    m->my = u[1]*sc;
    m->mz = u[2]*sc;

    s->B_prev[0]=B_body[0];
    s->B_prev[1]=B_body[1];
    s->B_prev[2]=B_body[2];
}
