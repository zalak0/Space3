#include "ads_runtime_check.h"
#include "ads_config.h"

#include <math.h>
#include <string.h>

/*
 * Runtime check policy for current H743 ADS bring-up:
 *
 * Required:
 * - telemetry pointer valid
 * - quaternion finite and near unit length
 * - gyro telemetry finite
 * - update counter advancing
 * - gyro_valid asserted
 * - accel_valid asserted
 *
 * Optional:
 * - magnetometer validity
 * - sun/photodiode validity
 * - fake sensor update count
 *
 * This allows the real H743 backend to pass health with gyro-only propagation,
 * while still exposing mag/sun validity through the debug structure.
 */

static ADS_RuntimeCheckResult s_last_result;

static bool ADS_RuntimeCheck_IsFiniteFloat(float value)
{
    return isfinite(value);
}

void ADS_RuntimeCheck_Init(void)
{
    memset(&s_last_result, 0, sizeof(s_last_result));
}

ADS_RuntimeCheckResult ADS_RuntimeCheck_Evaluate(
    const ADCS_Telemetry *telemetry,
    uint32_t fake_sensor_update_count
)
{
    ADS_RuntimeCheckResult result = s_last_result;

    result.total_check_count++;

    if (telemetry == 0)
    {
        result.telemetry_finite = false;
        result.quaternion_norm_ok = false;
        result.fake_sensor_count_advancing = false;
        result.telemetry_update_counter_advancing = false;
        result.gyro_status_valid = false;
        result.accel_status_valid = false;
        result.sun_status_valid = false;
        result.mag_status_valid = false;
        result.sensor_status_valid = false;
        result.ads_ok = false;
        result.bad_check_count++;
        s_last_result = result;
        return s_last_result;
    }

    const float q0 = telemetry->attitude_q.w;
    const float q1 = telemetry->attitude_q.x;
    const float q2 = telemetry->attitude_q.y;
    const float q3 = telemetry->attitude_q.z;

    result.telemetry_finite =
        ADS_RuntimeCheck_IsFiniteFloat(q0) &&
        ADS_RuntimeCheck_IsFiniteFloat(q1) &&
        ADS_RuntimeCheck_IsFiniteFloat(q2) &&
        ADS_RuntimeCheck_IsFiniteFloat(q3) &&

        ADS_RuntimeCheck_IsFiniteFloat(telemetry->gyro_rad_s.x) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->gyro_rad_s.y) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->gyro_rad_s.z) &&

        ADS_RuntimeCheck_IsFiniteFloat(telemetry->euler_rates_rad_s.x) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->euler_rates_rad_s.y) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->euler_rates_rad_s.z) &&

        ADS_RuntimeCheck_IsFiniteFloat(telemetry->accel_body_measured_m_s2.x) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->accel_body_measured_m_s2.y) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->accel_body_measured_m_s2.z) &&

        ADS_RuntimeCheck_IsFiniteFloat(telemetry->sun_body_measured.x) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->sun_body_measured.y) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->sun_body_measured.z) &&

        ADS_RuntimeCheck_IsFiniteFloat(telemetry->mag_body_measured_uT.x) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->mag_body_measured_uT.y) &&
        ADS_RuntimeCheck_IsFiniteFloat(telemetry->mag_body_measured_uT.z) &&

        ADS_RuntimeCheck_IsFiniteFloat(telemetry->dt);

    result.quaternion_norm = sqrtf(
        (q0 * q0) +
        (q1 * q1) +
        (q2 * q2) +
        (q3 * q3)
    );

    result.quaternion_norm_ok =
        result.telemetry_finite &&
        result.quaternion_norm > ADS_QUATERNION_NORM_MIN &&
        result.quaternion_norm < ADS_QUATERNION_NORM_MAX;

    /*
     * Keep this diagnostic populated, but do not require it for H743 health.
     * On real hardware this is just source_update_count, not necessarily fake.
     */
    result.fake_sensor_count_advancing =
        fake_sensor_update_count > result.last_fake_sensor_update_count;

    result.telemetry_update_counter_advancing =
        telemetry->update_counter > result.last_telemetry_update_counter;

    result.gyro_status_valid =
        telemetry->sensor_status.gyro_valid != 0u;

    result.accel_status_valid =
        telemetry->sensor_status.accel_valid != 0u;

    result.sun_status_valid =
        telemetry->sensor_status.sun_valid != 0u;

    result.mag_status_valid =
        telemetry->sensor_status.mag_valid != 0u;

    /*
     * Current ADS sensor health:
     * - gyro required
     * - accel required
     * - mag optional
     * - sun optional
     */
    result.sensor_status_valid =
        result.gyro_status_valid &&
        result.accel_status_valid;

    result.last_fake_sensor_update_count = fake_sensor_update_count;
    result.last_telemetry_update_counter = telemetry->update_counter;

    result.ads_ok =
        result.telemetry_finite &&
        result.quaternion_norm_ok &&
        result.telemetry_update_counter_advancing &&
        result.sensor_status_valid;

    if (!result.ads_ok)
    {
        result.bad_check_count++;
    }

    s_last_result = result;
    return s_last_result;
}

ADS_RuntimeCheckResult ADS_RuntimeCheck_GetLastResult(void)
{
    return s_last_result;
}