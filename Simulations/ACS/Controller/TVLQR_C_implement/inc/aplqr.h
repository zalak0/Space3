#ifndef APLQR_H
#define APLQR_H

/* aplqr.h */
typedef struct { float phi,phi_dot, theta,theta_dot, psi,psi_dot; } adcs_state_t;
typedef struct { float mx,my,mz; } adcs_dipole_t;   /* commanded dipole [A*m^2] */
void aplqr_step(const adcs_state_t *x, const float B_lvlh[3], adcs_dipole_t *m_cmd);

#endif /* APLQR_H */