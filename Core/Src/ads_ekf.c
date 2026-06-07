#include "ads_ekf.h"

static ADS_EKF_State g_ads_ekf_state;

void ADS_EKF_Init(void)
{
    g_ads_ekf_state.q_body_to_inertial.w = 1.0f;
    g_ads_ekf_state.q_body_to_inertial.x = 0.0f;
    g_ads_ekf_state.q_body_to_inertial.y = 0.0f;
    g_ads_ekf_state.q_body_to_inertial.z = 0.0f;

    g_ads_ekf_state.gyro_bias_rad_s.x = 0.0f;
    g_ads_ekf_state.gyro_bias_rad_s.y = 0.0f;
    g_ads_ekf_state.gyro_bias_rad_s.z = 0.0f;

    g_ads_ekf_state.covariance_trace = 0.0f;
    g_ads_ekf_state.update_count = 0u;

    g_ads_ekf_state.initialized = true;
    g_ads_ekf_state.healthy = true;
}

bool ADS_EKF_PredictUpdate(const ADS_EKF_Input* input)
{
    if (input == 0)
    {
        g_ads_ekf_state.healthy = false;
        return false;
    }

    if (input->dt_s <= 0.0f)
    {
        g_ads_ekf_state.healthy = false;
        return false;
    }

    /*
     * EKF implementation intentionally not active yet.
     *
     * Final ADS EKF responsibilities:
     * - propagate quaternion using gyro measurements
     * - estimate gyro bias
     * - maintain covariance matrix
     * - correct attitude using magnetometer vector observations
     * - optionally use a dedicated sun sensor if later added
     *
     * Photodiode inputs are intentionally not part of ADS.
     */

    g_ads_ekf_state.update_count++;
    g_ads_ekf_state.healthy = true;

    return true;
}

ADS_EKF_State ADS_EKF_GetState(void)
{
    return g_ads_ekf_state;
}