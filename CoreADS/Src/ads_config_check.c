#include "ads_config_check.h"

#include "ads_config.h"

#include <math.h>
#include <string.h>

static ADS_ConfigCheckResult s_last_result;

static bool ADS_ConfigCheck_IsFiniteFloat(float value)
{
    return isfinite(value);
}

void ADS_ConfigCheck_Init(void)
{
    memset(&s_last_result, 0, sizeof(s_last_result));

    s_last_result = ADS_ConfigCheck_Evaluate();
}

ADS_ConfigCheckResult ADS_ConfigCheck_Evaluate(void)
{
    ADS_ConfigCheckResult result;

    result.task_dt_ok =
        ADS_ConfigCheck_IsFiniteFloat(ADS_TASK_DT_S) &&
        ADS_TASK_DT_S > 0.0f;

    result.task_dt_tolerance_ok =
        ADS_ConfigCheck_IsFiniteFloat(ADS_TASK_DT_TOLERANCE_S) &&
        ADS_TASK_DT_TOLERANCE_S >= 0.0f;

    result.quaternion_bounds_ok =
        ADS_ConfigCheck_IsFiniteFloat(ADS_QUATERNION_NORM_MIN) &&
        ADS_ConfigCheck_IsFiniteFloat(ADS_QUATERNION_NORM_MAX) &&
        ADS_QUATERNION_NORM_MIN > 0.0f &&
        ADS_QUATERNION_NORM_MAX > ADS_QUATERNION_NORM_MIN;

    /*
    * Gyro validation limit is checked as part of the sun bounds flag group
    * only if no separate result field exists. Keep it simple for now by
    * folding the gyro limit into config_ok below.
    */
    const bool gyro_limit_ok =
        ADS_ConfigCheck_IsFiniteFloat(ADS_SENSOR_GYRO_ABS_MAX_RAD_S) &&
        ADS_SENSOR_GYRO_ABS_MAX_RAD_S > 0.0f;

    result.sun_norm_bounds_ok =
        ADS_ConfigCheck_IsFiniteFloat(ADS_SENSOR_SUN_NORM_MIN) &&
        ADS_ConfigCheck_IsFiniteFloat(ADS_SENSOR_SUN_NORM_MAX) &&
        ADS_SENSOR_SUN_NORM_MIN > 0.0f &&
        ADS_SENSOR_SUN_NORM_MAX > ADS_SENSOR_SUN_NORM_MIN;

    result.mag_norm_bounds_ok =
        ADS_ConfigCheck_IsFiniteFloat(ADS_SENSOR_MAG_NORM_MIN_T) &&
        ADS_ConfigCheck_IsFiniteFloat(ADS_SENSOR_MAG_NORM_MAX_T) &&
        ADS_SENSOR_MAG_NORM_MIN_T > 0.0f &&
        ADS_SENSOR_MAG_NORM_MAX_T > ADS_SENSOR_MAG_NORM_MIN_T;

    result.sun_vector_signal_threshold_ok =
        ADS_ConfigCheck_IsFiniteFloat(ADS_SUN_VECTOR_MIN_TOTAL_SIGNAL) &&
        ADS_SUN_VECTOR_MIN_TOTAL_SIGNAL > 0.0f;


    result.config_ok =
        result.task_dt_ok &&
        result.task_dt_tolerance_ok &&
        result.quaternion_bounds_ok &&
        gyro_limit_ok &&
        result.sun_norm_bounds_ok &&
        result.mag_norm_bounds_ok &&
        result.sun_vector_signal_threshold_ok;

    s_last_result = result;
    return s_last_result;
}

ADS_ConfigCheckResult ADS_ConfigCheck_GetLastResult(void)
{
    return s_last_result;
}

bool ADS_ConfigCheck_IsOk(void)
{
    return s_last_result.config_ok;
}