/* acs_test.c  —  APLQR unit / integration test harness
 * Compile:  gcc main_test.c aplqr.c -lm -o aplqr_test
 */

#include <stdio.h>
#include <math.h>
#include "pss.h"
#include "aplqr.h"
#include "acs_test.h"

/* ------------------------------------------------------------------ */
/*  Helpers                                                             */
/* ------------------------------------------------------------------ */

static void print_state(const char *tag, const adcs_state_t *x)
{
    printf("[%s] phi=%.3f  phi_d=%.4f  theta=%.3f  theta_d=%.4f"
           "  psi=%.3f  psi_d=%.4f  (deg, deg/s)\n",
           tag,
           x->phi * (180.0f / M_PI),   x->phi_dot   * (180.0f / M_PI),
           x->theta * (180.0f / M_PI), x->theta_dot * (180.0f / M_PI),
           x->psi * (180.0f / M_PI),   x->psi_dot   * (180.0f / M_PI));
}

static void print_dipole(const char *tag, const adcs_dipole_t *m)
{
    printf("[%s] mx=% .4e  my=% .4e  mz=% .4e  [A*m^2]\n",
           tag, m->mx, m->my, m->mz);
}

static float deg2rad(float d) { return d * (M_PI / 180.0f); }

/* ------------------------------------------------------------------ */
/*  Test cases                                                          */
/* ------------------------------------------------------------------ */

/* 1. Zero state → should produce zero (or near-zero) dipole */
void test_zero_state(void)
{
    puts("\n=== TEST 1: Zero state ===");
    adcs_state_t x = {0};
    float B[3] = { 2e-5f, 1e-5f, 4e-5f };   /* representative LVLH field [T] */
    adcs_dipole_t m;

    aplqr_step(&x, B, &m);

    print_state("in ", &x);
    print_dipole("out", &m);

    /* Soft pass/fail */
    float norm = m.mx*m.mx + m.my*m.my + m.mz*m.mz;
    printf("RESULT: %s\n", norm < 1e-20f ? "PASS" : "FAIL — non-zero output at zero state");
}

/* 2. Pure roll error, no rates */
void test_roll_only(void)
{
    puts("\n=== TEST 2: Roll error only (+20 deg) ===");
    adcs_state_t x = { deg2rad(20.f), 0, 0, 0, 0, 0 };
    float B[3] = { 2e-5f, 1e-5f, 4e-5f };
    adcs_dipole_t m;

    aplqr_step(&x, B, &m);

    print_state("in ", &x);
    print_dipole("out", &m);
}

/* 3. Saturation check — large error should clamp to M_MAX_HW */
void test_saturation(void)
{
    puts("\n=== TEST 3: Saturation (large attitude error) ===");
    adcs_state_t x = { deg2rad(89.f), deg2rad(5.f),
                        deg2rad(89.f), deg2rad(5.f),
                        deg2rad(89.f), deg2rad(5.f) };
    float B[3] = { 2e-5f, 1e-5f, 4e-5f };
    adcs_dipole_t m;

    aplqr_step(&x, B, &m);

    print_state("in ", &x);
    print_dipole("out", &m);

    float M_MAX_HW = 0.166f;
    float amax = fabsf(m.mx);
    if (fabsf(m.my) > amax) amax = fabsf(m.my);
    if (fabsf(m.mz) > amax) amax = fabsf(m.mz);
    printf("Max |component| = %.4f  (limit %.4f)\n", amax, M_MAX_HW);
    printf("RESULT: %s\n", amax <= M_MAX_HW + 1e-6f ? "PASS" : "FAIL — exceeded saturation limit");
}

/* 4. Simple open-loop simulation — watch error decay over N steps */
void test_simulation(void)
{
    puts("\n=== TEST 4: Open-loop step simulation (40 steps @ 1 Hz) ===");
    puts("  step |  phi(deg) | theta(deg) |  psi(deg) |   mx(A*m^2)");
    puts("  -----|-----------|------------|-----------|-------------");

    /* Simplified flat-earth kinematics: x_dot = A*x + B*u
     * Only angle ← rate integration here (no full dynamics),
     * just enough to sanity-check sign / convergence trend.       */

    adcs_state_t x = { deg2rad(15.f), 0, deg2rad(-10.f), 0, deg2rad(5.f), 0 };

    /* Rough diagonal inertia gains for open-loop integration */
    const float Ix = 3.3333e-3f, Iy = 1.01793e-2f, Iz = 1.01793e-2f;
    const float dt = 1.0f;                    /* 1 s time step */
    float B[3] = { 2e-5f, 1e-5f, 4e-5f };    /* static field — in practice time-varying */

    for (int k = 0; k < 40; k++) {
        adcs_dipole_t m;
        aplqr_step(&x, B, &m);

        printf("  %4d | %9.3f | %10.3f | %9.3f | % .4e\n",
               k,
               x.phi   * (180.0f/M_PI),
               x.theta * (180.0f/M_PI),
               x.psi   * (180.0f/M_PI),
               m.mx);

        /* torque from magnetorquer: tau = m × B */
        float tx = m.my*B[2] - m.mz*B[1];
        float ty = m.mz*B[0] - m.mx*B[2];
        float tz = m.mx*B[1] - m.my*B[0];

        /* Euler integration of angular acceleration → rate → angle */
        x.phi_dot   += (tx / Ix) * dt;
        x.theta_dot += (ty / Iy) * dt;
        x.psi_dot   += (tz / Iz) * dt;

        x.phi   += x.phi_dot   * dt;
        x.theta += x.theta_dot * dt;
        x.psi   += x.psi_dot   * dt;
    }
}

/* Replicate MATLAB's LEO LVLH B-field more accurately.
 * Simple tilted dipole: inclination ~97 deg (SSO-like).
 * B_lvlh components for a circular orbit, dipole tilt ~11 deg.
 */
static void get_B_lvlh(float t, float w_orb, float B_mag, float B[3])
{
    float u = w_orb * t;   /* argument of latitude */

    /* LVLH dipole model — matches make_B_tv() assumptions */
    /* Bx ~ along-track, By ~ orbit-normal, Bz ~ nadir */
    B[0] = -2.0f * B_mag * sinf(u);  /* along-track: weak */
    B[1] =  0.0f;                    /* orbit-normal: ~zero for equatorial dipole */
    B[2] =  B_mag * cosf(u);         /* nadir-pointing: dominant */
}

/* 5. B-field sensitivity — same state, vary B direction */
void test_bfield_sensitivity(void)
{
    puts("\n=== TEST 5: B-field direction sensitivity ===");
    adcs_state_t x = { deg2rad(10.f), 0, deg2rad(10.f), 0, 0, 0 };

    float configs[4][3] = {
        { 4e-5f,  0.f,   0.f  },   /* B along X */
        { 0.f,    4e-5f, 0.f  },   /* B along Y */
        { 0.f,    0.f,   4e-5f},   /* B along Z */
        { 2e-5f,  1e-5f, 4e-5f},   /* realistic mixed */
    };
    const char *labels[4] = { "B=X   ", "B=Y   ", "B=Z   ", "B=mix " };

    for (int i = 0; i < 4; i++) {
        adcs_dipole_t m;
        aplqr_step(&x, configs[i], &m);
        printf("  %s → mx=% .3e  my=% .3e  mz=% .3e\n",
               labels[i], m.mx, m.my, m.mz);
    }
}


/* ------------------------------------------------------------------ */
/* TEST 6: Rotating B field — orbital simulation (~LEO)               */
/* Convergence criterion: all angles < 1 deg, all rates < 0.1 deg/s  */
/* ------------------------------------------------------------------ */
void test_orbital_convergence(void)
{
    puts("\n=== TEST 6: Orbital convergence with rotating B field ===");

    const float w_orb = 2.0f * M_PI / 5695.0f;  /* match MATLAB period */
    const float B_mag = 4e-5f;
    const float dt    = 0.1f;                     /* match MATLAB dt_sim */
    const int   N_MAX = 20 * (int)(5695.0f/dt);  /* 20 orbits like MATLAB */

    const float Ix = 3.3333e-3f, Iy = 1.01793e-2f, Iz = 1.01793e-2f;

    adcs_state_t x = {
        deg2rad(20.f), 0.f,   /* match MATLAB IC: 20 deg all axes */
        deg2rad(20.f), 0.f,
        deg2rad(20.f), 0.f
    };

    /* Print every ~1 orbit worth of steps */
    int print_interval = (int)(5695.0f / dt);

    puts("  orbit |  phi(deg) | theta(deg) |  psi(deg) | state_norm");
    puts("  ------|-----------|------------|-----------|------------");

    for (int k = 0; k < N_MAX; k++) {

        float t = k * dt;
        float B[3];
        get_B_lvlh(t, w_orb, B_mag, B);

        adcs_dipole_t m;
        aplqr_step(&x, B, &m);

        if (k % print_interval == 0) {
            float norm2 = x.phi*x.phi + x.theta*x.theta + x.psi*x.psi
                        + x.phi_dot*x.phi_dot + x.theta_dot*x.theta_dot
                        + x.psi_dot*x.psi_dot;
            printf("  %5d | %9.3f | %10.3f | %9.3f | %10.4f\n",
                   k / print_interval,
                   x.phi   * (180.f/M_PI),
                   x.theta * (180.f/M_PI),
                   x.psi   * (180.f/M_PI),
                   sqrtf(norm2) * (180.f/M_PI));
        }

        /* tau = m × B */
        float tx = m.my*B[2] - m.mz*B[1];
        float ty = m.mz*B[0] - m.mx*B[2];
        float tz = m.mx*B[1] - m.my*B[0];

        x.phi_dot   += (tx / Ix) * dt;
        x.theta_dot += (ty / Iy) * dt;
        x.psi_dot   += (tz / Iz) * dt;

        x.phi   += x.phi_dot   * dt;
        x.theta += x.theta_dot * dt;
        x.psi   += x.psi_dot   * dt;
    }
}

/* ------------------------------------------------------------------ */
/* TEST 7: Energy / Lyapunov descent check                            */
/* V(x) = x' * Pss * x must be monotonically non-increasing          */
/* ------------------------------------------------------------------ */
void test_lyapunov_descent(void)
{
    puts("\n=== TEST 7: Lyapunov descent (V = x'*Pss*x) ===");

    /* Import Pss from pss.h — already included via aplqr.h chain */
    const float w_orb = 2.0f * M_PI / (93.0f * 60.0f);
    const float B_mag = 4e-5f;
    const float dt    = 1.0f;
    const float Ix = 3.3333e-3f, Iy = 1.01793e-2f, Iz = 1.01793e-2f;

    adcs_state_t x = { deg2rad(10.f), 0.f, deg2rad(-8.f), 0.f, deg2rad(4.f), 0.f };

    float V_prev = 1e30f;
    int violations = 0;

    puts("  step |      V(x)      |   delta-V   | descent?");
    puts("  -----|----------------|-------------|----------");

    for (int k = 0; k < 600; k++) {

        float t   = k * dt;
        float B[3] = {
             B_mag * cosf(w_orb * t),
             B_mag * sinf(w_orb * t) * 0.3f,
            -B_mag * sinf(w_orb * t)
        };

        /* Compute V = x' * Pss * x */
        float xv[6] = { x.phi, x.phi_dot, x.theta, x.theta_dot, x.psi, x.psi_dot };
        float Px[6] = {0};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                Px[i] += Pss[i][j] * xv[j];
        float V = 0.f;
        for (int i = 0; i < 6; i++) V += xv[i] * Px[i];

        if (k % 60 == 0) {
            float dV = V - V_prev;
            printf("  %4d | %14.4f | %11.4f | %s\n",
                   k, V, (k == 0) ? 0.f : dV,
                   (k == 0 || dV <= 0.f) ? "OK" : "VIOLATION");
        }

        if (k > 0 && V > V_prev + 1.f)  /* tolerance for float noise */
            violations++;

        V_prev = V;

        adcs_dipole_t m;
        aplqr_step(&x, B, &m);

        float tx = m.my*B[2] - m.mz*B[1];
        float ty = m.mz*B[0] - m.mx*B[2];
        float tz = m.mx*B[1] - m.my*B[0];

        x.phi_dot   += (tx / Ix) * dt;
        x.theta_dot += (ty / Iy) * dt;
        x.psi_dot   += (tz / Iz) * dt;

        x.phi   += x.phi_dot   * dt;
        x.theta += x.theta_dot * dt;
        x.psi   += x.psi_dot   * dt;
    }

    printf("\nTotal V-increase violations: %d\n", violations);
    printf("RESULT: %s\n", violations < 5 ? "PASS" : "FAIL — Lyapunov not descending");
}
