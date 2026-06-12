#include "ads_output.h"
#include "ads_config.h"

#include <math.h>
#include <string.h>

static ADS_Output s_latest_output;

static bool ADS_Output_IsFiniteFloat(float value)
{
    return isfinite(value);
}

static bool ADS_Output_QuaternionLooksValid(Quaternion q)
{
    const float norm = sqrtf(
        q.w*q.w +
        q.x*q.x +
        q.y*q.y +
        q.z*q.z
    );

    return ADS_Output_IsFiniteFloat(norm) &&
        norm > ADS_QUATERNION_NORM_MIN &&
        norm < ADS_QUATERNION_NORM_MAX;
}

void ADS_Output_Init(void)
{
    memset(&s_latest_output, 0, sizeof(s_latest_output));

    s_latest_output.attitude_q.w = 1.0f;
    s_latest_output.attitude_q.x = 0.0f;
    s_latest_output.attitude_q.y = 0.0f;
    s_latest_output.attitude_q.z = 0.0f;

    s_latest_output.euler_rad.roll_rad = 0.0f;
    s_latest_output.euler_rad.pitch_rad = 0.0f;
    s_latest_output.euler_rad.yaw_rad = 0.0f;

    s_latest_output.euler_rates_rad_s.x = 0.0f;
    s_latest_output.euler_rates_rad_s.y = 0.0f;
    s_latest_output.euler_rates_rad_s.z = 0.0f;

    s_latest_output.valid = false;
}

void ADS_Output_UpdateFromTelemetry(const ADCS_Telemetry *telemetry)
{
    if (telemetry == 0) {
        s_latest_output.valid = false;
        return;
    }

    s_latest_output.attitude_q = telemetry->attitude_q;
    s_latest_output.euler_rad = telemetry->euler_rad;
    s_latest_output.euler_rates_rad_s = telemetry->euler_rates_rad_s;
    s_latest_output.gyro_rad_s = telemetry->gyro_rad_s;

    s_latest_output.accel_body_measured_m_s2 = telemetry->accel_body_measured_m_s2;
    s_latest_output.accel_body_predicted_m_s2 = telemetry->accel_body_predicted_m_s2;
    s_latest_output.accel_error = telemetry->accel_error;

    s_latest_output.sun_body_measured = telemetry->sun_body_measured;
    s_latest_output.sun_body_predicted = telemetry->sun_body_predicted;
    s_latest_output.sun_error = telemetry->sun_error;

    s_latest_output.mag_body_measured_uT = telemetry->mag_body_measured_uT;
    s_latest_output.mag_body_predicted_uT = telemetry->mag_body_predicted_uT;
    s_latest_output.mag_error = telemetry->mag_error;

    s_latest_output.sensor_status = telemetry->sensor_status;

    s_latest_output.dt = telemetry->dt;
    s_latest_output.update_counter = telemetry->update_counter;

    /*
    * Output validity for current H743 IMU bring-up:
    * - attitude quaternion must be finite and near unit length
    * - gyro must be finite and valid
    * - mag is optional; EKF uses it when present
    * - sun/photodiodes are not part of this ADS phase
    */
    s_latest_output.valid =
        ADS_Output_QuaternionLooksValid(s_latest_output.attitude_q) &&
        ADS_Output_IsFiniteFloat(s_latest_output.gyro_rad_s.x) &&
        ADS_Output_IsFiniteFloat(s_latest_output.gyro_rad_s.y) &&
        ADS_Output_IsFiniteFloat(s_latest_output.gyro_rad_s.z) &&

        ADS_Output_IsFiniteFloat(s_latest_output.euler_rates_rad_s.x) &&
        ADS_Output_IsFiniteFloat(s_latest_output.euler_rates_rad_s.y) &&
        ADS_Output_IsFiniteFloat(s_latest_output.euler_rates_rad_s.z) &&

        ADS_Output_IsFiniteFloat(s_latest_output.accel_body_measured_m_s2.x) &&
        ADS_Output_IsFiniteFloat(s_latest_output.accel_body_measured_m_s2.y) &&
        ADS_Output_IsFiniteFloat(s_latest_output.accel_body_measured_m_s2.z) &&

        s_latest_output.sensor_status.gyro_valid != 0u &&
        s_latest_output.sensor_status.accel_valid != 0u;
}

ADS_Output ADS_Output_GetLatest(void)
{
    return s_latest_output;
}

bool ADS_Output_IsValid(void)
{
    return s_latest_output.valid;
}

EulerAngles ADS_Output_GetEulerRad(void)
{
    return s_latest_output.euler_rad;
}

Vector3 ADS_Output_GetEulerRatesRadS(void)
{
    return s_latest_output.euler_rates_rad_s;
}

adcs_state_t ADS_Output_GetAdcsState(void)
{
    adcs_state_t out;

    out.phi = s_latest_output.euler_rad.roll_rad;
    out.phi_dot = s_latest_output.euler_rates_rad_s.x;

    out.theta = s_latest_output.euler_rad.pitch_rad;
    out.theta_dot = s_latest_output.euler_rates_rad_s.y;

    out.psi = s_latest_output.euler_rad.yaw_rad;
    out.psi_dot = s_latest_output.euler_rates_rad_s.z;

    out.valid = s_latest_output.valid ? 1u : 0u;

    return out;
}