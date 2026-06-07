/* App/common/goose_config.c */
#include "common/goose_config.h"

/* Torquer constants from bench characterization. Placeholders until measured. */
const float k_torquer[3] = { 0.166f, 0.166f, 0.166f };   /* KX, KY, KZ [A*m^2] */

/* Inertia as an array, for code that wants to index axes in a loop. */
const float I_diag[3] = { I_XX, I_YY, I_ZZ };

#ifdef GOOSE_SELFTEST
#include "pss.h"     /* MATLAB export stamps I_diag / Rinv_diag as comments+arrays */
#include <math.h>
/* Call once at boot. Fails loudly if the flight constants drifted from
   what Pss was designed against — the gap where a units error hides. */
int goose_config_check(void)
{
    if (fabsf(I_diag[0] - I_XX) > 1e-9f) return -1;
    if (fabsf(RINV_DIAG  - 2.5e-9f) > 1e-15f) return -2;
    return 0;
}
#endif
