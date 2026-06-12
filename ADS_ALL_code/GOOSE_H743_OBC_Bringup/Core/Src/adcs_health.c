#include "adcs_health.h"

static ADCS_HealthFlags health_flags;

void ADCS_Health_Init(void)
{
    health_flags.gyro_fault = 0;
    health_flags.accelerometer_fault = 0;
    health_flags.sun_sensor_fault = 0;
    health_flags.magnetometer_fault = 0;
    health_flags.attitude_fault = 0;
    health_flags.high_rate_fault = 0;
    health_flags.any_fault = 0;
}

void ADCS_Health_Update(
    const ADCS_Telemetry* adcs_telemetry,
    const ADCS_HealthModeTelemetry* mode_telemetry
)

{
    health_flags.gyro_fault = 0;
    health_flags.accelerometer_fault = 0;
    health_flags.sun_sensor_fault = 0;
    health_flags.magnetometer_fault = 0;
    health_flags.attitude_fault = 0;
    health_flags.high_rate_fault = 0;

    if (adcs_telemetry != 0)
    {
        if (adcs_telemetry->sensor_status.gyro_valid == 0)
        {
            health_flags.gyro_fault = 1;
        }

        if (adcs_telemetry->sensor_status.accel_valid == 0)
        {
            health_flags.accelerometer_fault = 1;
        }

        /* No sun sensors fitted: do not fault on sun_valid == 0. */
        health_flags.sun_sensor_fault = 0;

        if (adcs_telemetry->sensor_status.mag_valid == 0)
        {
            health_flags.magnetometer_fault = 1;
        }

        /*
         * Basic quaternion sanity check.
         * This should rarely fail because we normalise constantly,
         * but it is useful as a defensive check.
         */
        float q_norm_sq =
            (adcs_telemetry->attitude_q.w * adcs_telemetry->attitude_q.w) +
            (adcs_telemetry->attitude_q.x * adcs_telemetry->attitude_q.x) +
            (adcs_telemetry->attitude_q.y * adcs_telemetry->attitude_q.y) +
            (adcs_telemetry->attitude_q.z * adcs_telemetry->attitude_q.z);

        if ((q_norm_sq < 0.8f) || (q_norm_sq > 1.2f))
        {
            health_flags.attitude_fault = 1;
        }
    }

    if (mode_telemetry != 0)
    {
        if (mode_telemetry->angular_rate_rad_s >
            mode_telemetry->detumble_entry_threshold_rad_s)
        {
            health_flags.high_rate_fault = 1;
        }
    }

    health_flags.any_fault =
        health_flags.gyro_fault |
        health_flags.accelerometer_fault |
        health_flags.sun_sensor_fault |
        health_flags.magnetometer_fault |
        health_flags.attitude_fault |
        health_flags.high_rate_fault;
}

ADCS_HealthFlags ADCS_Health_GetFlags(void)
{
    return health_flags;
}