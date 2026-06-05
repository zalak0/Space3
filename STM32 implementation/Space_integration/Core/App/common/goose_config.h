/* App/common/goose_config.h */
#ifndef GOOSE_CONFIG_H
#define GOOSE_CONFIG_H

/* ---- Timing -------------------------------------------------------- */
#define DT_CTRL_S        0.1f      /* control timestep [s] — MUST match Pss design */
#define DT_CTRL_MS       100u      /* same, in ms, for the loop anchor            */

/* ---- Inertia [kg*m^2] — SINGLE SOURCE. Also stamped in pss.h;        */
/*      assert these match at startup (see goose_config.c).             */
#define I_XX             3.3333e-3f
#define I_YY             1.01793e-2f
#define I_ZZ             1.01793e-2f

/* ---- APLQR weights ------------------------------------------------- */
#define M_MAX_DES        5.0e-5f    /* design dipole weight (decoupled from HW)    */
#define RINV_DIAG        2.5e-9f    /* = M_MAX_DES^2                                */
#define M_MAX_HW         0.166f     /* hardware dipole limit — saturator only      */

/* ---- STM transition thresholds ------------------------------------- */
#define OMEGA_CAPTURE       (0.5f * 0.01745329f)  /* rate to exit DETUMBLE [rad/s] */
#define POINT_CONVERGED_RAD (2.0f * 0.01745329f)  /* pointing-error gate [rad]     */

/* ---- EPS SoC gate (hysteresis: enter low, exit high) --------------- */
#define SOC_SAFE_ENTER   0.25f     /* trip soc_low below this   */
#define SOC_SAFE_EXIT    0.50f     /* clear soc_low above this  */

/* ---- Magnetorquer characterization (aggregates -> goose_config.c) -- */
extern const float k_torquer[3];   /* A*m^2 at 100% duty, per axis */
extern const float I_diag[3];      /* {I_XX, I_YY, I_ZZ} as an array */

#endif
