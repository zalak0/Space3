/* acs_test.c  —  APLQR unit / integration test harness
 * Compile:  gcc main_test.c aplqr.c -lm -o aplqr_test
 */

#ifndef ACS_TEST_H
#define ACS_TEST_H
/* ------------------------------------------------------------------ */
/*  Test cases                                                          */
/* ------------------------------------------------------------------ */

/* 1. Zero state → should produce zero (or near-zero) dipole */
void test_zero_state(void);

/* 2. Pure roll error, no rates */
void test_roll_only(void);

/* 3. Saturation check — large error should clamp to M_MAX_HW */
void test_saturation(void);

/* 4. Simple open-loop simulation — watch error decay over N steps */
void test_simulation(void);

/* 5. B-field sensitivity — same state, vary B direction */
void test_bfield_sensitivity(void);

void test_orbital_convergence(void);

void test_lyapunov_descent(void);

void test_bdot_detumble(void);

#endif /* ACS_TEST_H */

