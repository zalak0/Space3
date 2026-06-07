#include "ads_sensor_backend.h"
#include "ads_sensor_interface.h"
#include "ads_h743_sensors.h"

#include <stdbool.h>

static bool g_h743_backend_initialized = false;

bool ADS_SensorBackend_Init(void)
{
    ADS_H743_Sensors_Init();

    if (ADS_H743_Sensors_IsInitialized() == 0u)
    {
        g_h743_backend_initialized = false;
        return false;
    }

    g_h743_backend_initialized = true;
    return true;
}

ADS_SensorPacket ADS_SensorBackend_UpdateAndRead(float dt_s)
{
    ADS_H743_SensorDiagnostics diag;

    if (g_h743_backend_initialized == false)
    {
        return ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_H743_REAL);
    }

    ADS_H743_Sensors_Update(dt_s);

    diag = ADS_H743_Sensors_GetDiagnostics();

    /*
     * Real H743 sensor backend is intentionally not ready yet.
     * Until IMU / magnetometer drivers are implemented and validated,
     * this backend must not emit valid ADS sensor packets.
     */
    if ((diag.initialized == 0u) || (diag.sensor_backend_ready == 0u))
    {
        return ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_H743_REAL);
    }

    /*
     * Future real sensor path:
     *
     * - read IMU gyro vector
     * - read magnetometer vector
     * - optionally read sun sensor vector if real hardware exists
     * - populate ADS_SensorPacket
     * - validate packet
     *
     * Do not implement this until hardware details are confirmed.
     */
    return ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_H743_REAL);
}