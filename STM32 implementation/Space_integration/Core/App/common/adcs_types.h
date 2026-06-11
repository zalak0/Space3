/* adcs_types.h — the ADS↔ACS contract. FROZEN once agreed.
   Owned by neither subsystem; changing it requires both owners to sign off. */
#ifndef ADCS_TYPES_H
#define ADCS_TYPES_H
#include <stdint.h>

/* ADS output / ACS input: attitude error in LVLH, rad and rad/s */
typedef struct {
    float phi,   phi_dot;     /* roll  */
    float theta, theta_dot;   /* pitch */
    float psi,   psi_dot;     /* yaw   */
    uint8_t valid;            /* ADS sets 0 if estimate is stale/diverged */
} adcs_state_t;

/* ACS output: commanded magnetic dipole, A*m^2, body frame */
typedef struct { float mx, my, mz; } adcs_dipole_t;
#endif
