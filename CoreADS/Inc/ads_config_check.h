#ifndef ADS_CONFIG_CHECK_H
#define ADS_CONFIG_CHECK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/*
 * ads_config_check.h
 *
 * Runtime validation of float-valued ADS configuration constants.
 *
 * Purpose:
 * - Check config values that cannot be checked by the C preprocessor.
 * - Prevent ADS from reporting healthy if configuration constants are invalid.
 * - Keep config validation separate from main.c.
 *
 * This module is hardware-free.
 */

typedef struct
{
    bool task_dt_ok;
    bool task_dt_tolerance_ok;
    bool quaternion_bounds_ok;
    bool sun_norm_bounds_ok;
    bool mag_norm_bounds_ok;
    bool sun_vector_signal_threshold_ok;
    bool config_ok;

} ADS_ConfigCheckResult;

void ADS_ConfigCheck_Init(void);

ADS_ConfigCheckResult ADS_ConfigCheck_Evaluate(void);

ADS_ConfigCheckResult ADS_ConfigCheck_GetLastResult(void);

bool ADS_ConfigCheck_IsOk(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_CONFIG_CHECK_H */