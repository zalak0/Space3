#ifndef APLQR_H
#define APLQR_H
#include <acs/adcs_types.h>

/* aplqr.h */
void aplqr_step(const adcs_state_t *x, const float B_lvlh[3], adcs_dipole_t *m_cmd);

#endif /* APLQR_H */
