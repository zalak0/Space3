#include "ads_sensor_validate.h"

#include "ads_config.h"

#include <math.h>
#include <string.h>

/*
 * Sensor validation thresholds.
 *
 * Sun vector:
 * - expected to be a unit vector.
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

static bool ADS_SensorValidate_GyroMagnitudeSensible(Vector3 gyro_rad_s)
{
    return fabsf(gyro_rad_s.x) <= ADS_SENSOR_GYRO_ABS_MAX_RAD_S &&
           fabsf(gyro_rad_s.y) <= ADS_SENSOR_GYRO_ABS_MAX_RAD_S &&
           fabsf(gyro_rad_s.z) <= ADS_SENSOR_GYRO_ABS_MAX_RAD_S;
}

static float ADS_SensorValidate_VectorNorm(Vector3 v)
{
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

void ADS_SensorValidate_Init(void)
{
    memset(&s_last_result, 0, sizeof(s_last_result));
}

ADS_SensorValidationResult ADS_SensorValidate_Apply(ADS_SensorPacket *packet)
{
    ADS_SensorValidationResult result = s_last_result;

    result.total_packets_checked++;

    if (packet == 0) {
        result.source_ok = false;
        result.gyro_ok = false;
        result.sun_ok = false;
        result.mag_ok = false;
        result.packet_ok = false;
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
    * - if marked valid, vector must be finite.
    * - each component must stay within a generous physical sanity limit.
    */
    if (packet->status.gyro_valid != 0u &&
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
     * Sun validation:
     * - if marked valid, vector must be finite and roughly unit length.
     */
    if (packet->status.sun_valid != 0u &&
        ADS_SensorValidate_VectorFinite(packet->sun_body))
    {
        const float sun_norm = ADS_SensorValidate_VectorNorm(packet->sun_body);

        if (sun_norm >= ADS_SENSOR_SUN_NORM_MIN &&
            sun_norm <= ADS_SENSOR_SUN_NORM_MAX)
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
     * - if marked valid, vector must be finite and have sensible Earth-field magnitude.
     */
    if (packet->status.mag_valid != 0u &&
        ADS_SensorValidate_VectorFinite(packet->mag_body_T))
    {
        const float mag_norm_T = ADS_SensorValidate_VectorNorm(packet->mag_body_T);

        if (mag_norm_T >= ADS_SENSOR_MAG_NORM_MIN_T &&
            mag_norm_T <= ADS_SENSOR_MAG_NORM_MAX_T)
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

    result.packet_ok =
        result.source_ok &&
        result.gyro_ok &&
        result.sun_ok &&
        result.mag_ok;

    if (!result.packet_ok) {
        result.bad_packets_count++;
    }

    s_last_result = result;
    return s_last_result;
}

ADS_SensorValidationResult ADS_SensorValidate_GetLastResult(void)
{
    return s_last_result;
}