#ifndef BDOT_H
#define BDOT_H
#include "aplqr.h"   /* reuse adcs_dipole_t */

typedef struct {
    float B_prev[3];   /* last body-frame field sample [T] */
    int   primed;      /* 0 until first sample taken */
} bdot_state_t;

void bdot_init(bdot_state_t *s);
/* B_body in Tesla, dt in seconds. Outputs commanded dipole m [A*m^2]. */
void bdot_step(bdot_state_t *s, const float B_body[3], float dt, adcs_dipole_t *m);

#endif
