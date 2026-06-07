/* ------------------------------------------------------------------ */
/*  Entry point                                                         */
/* ------------------------------------------------------------------ */

#include <stdio.h>
#include <math.h>
#include "inc/aplqr.h"
#include "inc/acs_test.h"


int main(void)
{
    puts("============================================");
    puts(" APLQR Controller — C Test Harness");
    puts("============================================");

    test_zero_state();
    test_roll_only();
    test_saturation();
    test_simulation();
    test_bfield_sensitivity();
    test_orbital_convergence();
    test_lyapunov_descent();

    puts("\nAll tests complete.");
    return 0;
}