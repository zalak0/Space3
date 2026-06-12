#include "ads_sensor_backend.h"

#include "ads_h743_sensors.h"

#include <stdbool.h>

/*
 * H743 real sensor backend.
 *
 * Current implemented real path:
 * - ICM20948 accel/gyro over I2C
 *
 * Current not-yet-implemented path:
 * - magnetometer readout
 * - sun/photodiode input
 *
 * This backend deliberately emits:
 * - gyro_valid  = 1 when ICM20948 gyro read succeeds
 * - accel_valid = 1 when ICM20948 accel read succeeds
 * - mag_valid   = 1 when magnetometer read succeeds, otherwise optional
 * - sun_valid   = 0 because photodiodes are not part of this ADS phase
 */

static bool g_h743_backend_initialized = false;
static uint32_t g_h743_backend_update_count = 0u;
static float g_h743_backend_time_s = 0.0f;

bool ADS_SensorBackend_Init(void)
{
    ADS_H743_Sensors_Init();

    if (ADS_H743_Sensors_IsInitialized() == 0u)
    {
        g_h743_backend_initialized = false;
        return false;
    }

    /*
     * Initialisation means the backend module exists and can be called.
     * ADS_H743_Sensors_IsBackendReady() separately tells us whether the
     * physical ICM20948 was detected/configured.
     */
    g_h743_backend_initialized = true;
    g_h743_backend_update_count = 0u;
    g_h743_backend_time_s = 0.0f;

    return true;
}

ADS_SensorPacket ADS_SensorBackend_UpdateAndRead(float dt_s)
{
    ADS_SensorPacket packet;
    ADS_H743_SensorSample sample;

    if (dt_s < 0.0f)
    {
        dt_s = 0.0f;
    }

    g_h743_backend_update_count++;
    g_h743_backend_time_s += dt_s;

    if (g_h743_backend_initialized == false)
    {
        return ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_H743_REAL);
    }

    ADS_H743_Sensors_Update(dt_s);

    packet = ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_H743_REAL);

    packet.source_update_count = g_h743_backend_update_count;
    packet.source_time_s = g_h743_backend_time_s;

    if (ADS_H743_Sensors_IsBackendReady() == 0u)
    {
        return packet;
    }

    if (!ADS_H743_Sensors_ReadSample(&sample))
    {
        return packet;
    }

    if (sample.gyro_valid != 0u)
    {
        packet.gyro_rad_s = sample.gyro_rad_s;
        packet.status.gyro_valid = 1u;
    }

    if (sample.accel_valid != 0u)
    {
        packet.accel_body_m_s2 = sample.accel_m_s2;
        packet.status.accel_valid = 1u;
    }
    else
    {
        packet.accel_body_m_s2.x = 0.0f;
        packet.accel_body_m_s2.y = 0.0f;
        packet.accel_body_m_s2.z = 0.0f;
        packet.status.accel_valid = 0u;
    }

    if (sample.mag_valid != 0u)
    {
        packet.mag_body_T = sample.mag_T;
        packet.status.mag_valid = 1u;
    }
    else
    {
        packet.mag_body_T.x = 0.0f;
        packet.mag_body_T.y = 0.0f;
        packet.mag_body_T.z = 0.0f;
        packet.status.mag_valid = 0u;
    }

    /*
     * Photodiode/sun vector intentionally unused.
     */
    packet.sun_body.x = 0.0f;
    packet.sun_body.y = 0.0f;
    packet.sun_body.z = 0.0f;
    packet.status.sun_valid = 0u;

    return packet;
}