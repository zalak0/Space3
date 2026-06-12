#include "ads_sensor_validate.h"

#include "ads_config.h"

#include <math.h>
#include <string.h>

/*
 * Sensor validation thresholds.
 *
 * Current ADS bring-up path:
 * - Gyro is required.
 * - Accelerometer gravity vector is required for bench attitude correction.
 * - Magnetometer is optional for yaw correction during bring-up.
 * - Sun vector / photodiode input is unused and NOT required for packet_ok.
 *
 * Photodiode inputs are intentionally not part of the ADS path unless explicitly
 * reintroduced later.
 *
 * Magnetometer:
 * - Earth field is usually around 25 to 65 microtesla.
 * - Use a wider bring-up sanity range here: 1 to 200 microtesla.
 * - Units entering ADCS are Tesla.
 */

static ADS_SensorValidationResult s_last_result;

static bool ADS_SensorValidate_IsFiniteFloat(float value)
{
    return isfinite(value);
}

static Vector3 ADS_SensorValidate_ZeroVector(void)
{
    Vector3 v;
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;
    return v;
}

static bool ADS_SensorValidate_VectorFinite(Vector3 v)
{
    return ADS_SensorValidate_IsFiniteFloat(v.x) &&
           ADS_SensorValidate_IsFiniteFloat(v.y) &&
           ADS_SensorValidate_IsFiniteFloat(v.z);
}

static float ADS_SensorValidate_VectorNorm(Vector3 v)
{
    return sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

static bool ADS_SensorValidate_GyroMagnitudeSensible(Vector3 gyro_rad_s)
{
    return fabsf(gyro_rad_s.x) <= ADS_SENSOR_GYRO_ABS_MAX_RAD_S &&
           fabsf(gyro_rad_s.y) <= ADS_SENSOR_GYRO_ABS_MAX_RAD_S &&
           fabsf(gyro_rad_s.z) <= ADS_SENSOR_GYRO_ABS_MAX_RAD_S;
}

void ADS_SensorValidate_Init(void)
{
    memset(&s_last_result, 0, sizeof(s_last_result));
}

ADS_SensorValidationResult ADS_SensorValidate_Apply(ADS_SensorPacket *packet)
{
    ADS_SensorValidationResult result = s_last_result;

    result.source_ok = false;
    result.gyro_ok = false;
    result.accel_ok = false;
    result.sun_ok = false;
    result.mag_ok = false;
    result.packet_ok = false;

    result.total_packets_checked++;

    if (packet == 0)
    {
        result.last_source = ADS_SENSOR_SOURCE_UNKNOWN;
        result.bad_packets_count++;
        s_last_result = result;
        return s_last_result;
    }

    result.last_source = packet->source;

    result.source_ok =
        packet->source == ADS_SENSOR_SOURCE_FAKE ||
        packet->source == ADS_SENSOR_SOURCE_H743_REAL ||
        packet->source == ADS_SENSOR_SOURCE_H743_STUB;

    /*
     * Gyro validation:
     * - Required for ADS propagation.
     * - If marked valid, vector must be finite.
     * - Each component must stay within a generous physical sanity limit.
     */
    if ((packet->status.gyro_valid != 0u) &&
        ADS_SensorValidate_VectorFinite(packet->gyro_rad_s) &&
        ADS_SensorValidate_GyroMagnitudeSensible(packet->gyro_rad_s))
    {
        result.gyro_ok = true;
    }
    else
    {
        packet->gyro_rad_s = ADS_SensorValidate_ZeroVector();
        packet->status.gyro_valid = 0u;
        result.gyro_ok = false;
    }

    /*
     * Accelerometer validation:
     * - Required for the no-sun bench ADS path.
     * - Treat it as a gravity-direction vector only when its magnitude is
     *   roughly near 1 g. Large translational acceleration will invalidate it.
     */
    if ((packet->status.accel_valid != 0u) &&
        ADS_SensorValidate_VectorFinite(packet->accel_body_m_s2))
    {
        const float accel_norm_m_s2 =
            ADS_SensorValidate_VectorNorm(packet->accel_body_m_s2);

        if ((accel_norm_m_s2 >= ADS_SENSOR_ACCEL_NORM_MIN_M_S2) &&
            (accel_norm_m_s2 <= ADS_SENSOR_ACCEL_NORM_MAX_M_S2))
        {
            result.accel_ok = true;
        }
        else
        {
            packet->accel_body_m_s2 = ADS_SensorValidate_ZeroVector();
            packet->status.accel_valid = 0u;
            result.accel_ok = false;
        }
    }
    else
    {
        packet->accel_body_m_s2 = ADS_SensorValidate_ZeroVector();
        packet->status.accel_valid = 0u;
        result.accel_ok = false;
    }

    /*
     * Sun validation:
     * - Optional for the current implementation.
     * - If present and valid, keep it.
     * - If absent or bad, clear it but DO NOT fail packet_ok.
     */
    if ((packet->status.sun_valid != 0u) &&
        ADS_SensorValidate_VectorFinite(packet->sun_body))
    {
        const float sun_norm = ADS_SensorValidate_VectorNorm(packet->sun_body);

        if ((sun_norm >= ADS_SENSOR_SUN_NORM_MIN) &&
            (sun_norm <= ADS_SENSOR_SUN_NORM_MAX))
        {
            result.sun_ok = true;
        }
        else
        {
            packet->sun_body = ADS_SensorValidate_ZeroVector();
            packet->status.sun_valid = 0u;
            result.sun_ok = false;
        }
    }
    else
    {
        packet->sun_body = ADS_SensorValidate_ZeroVector();
        packet->status.sun_valid = 0u;
        result.sun_ok = false;
    }

    /*
     * Magnetometer validation:
     * - Required for absolute ADS correction in the current gyro+mag EKF.
     * - Units must be Tesla by this point.
     */
    if ((packet->status.mag_valid != 0u) &&
        ADS_SensorValidate_VectorFinite(packet->mag_body_T))
    {
        const float mag_norm_T = ADS_SensorValidate_VectorNorm(packet->mag_body_T);

        if ((mag_norm_T >= ADS_SENSOR_MAG_NORM_MIN_T) &&
            (mag_norm_T <= ADS_SENSOR_MAG_NORM_MAX_T))
        {
            result.mag_ok = true;
        }
        else
        {
            packet->mag_body_T = ADS_SensorValidate_ZeroVector();
            packet->status.mag_valid = 0u;
            result.mag_ok = false;
        }
    }
    else
    {
        packet->mag_body_T = ADS_SensorValidate_ZeroVector();
        packet->status.mag_valid = 0u;
        result.mag_ok = false;
    }

    /*
    * Current ADS bring-up packet health:
    * - source required
    * - gyro required for attitude propagation
    * - accel required for bench gravity-vector correction
    * - magnetometer optional; used by EKF/QUEST when available
    * - sun/photodiode optional and intentionally unused in this ADS phase
    */
    result.packet_ok =
        result.source_ok &&
        result.gyro_ok &&
        result.accel_ok;

    if (!result.packet_ok)
    {
        result.bad_packets_count++;
    }

    s_last_result = result;
    return s_last_result;
}

ADS_SensorValidationResult ADS_SensorValidate_GetLastResult(void)
{
    return s_last_result;
}