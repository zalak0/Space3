#include "ads_h743_sensors.h"

static ADS_H743_SensorDiagnostics g_h743_sensor_diag = {0};

void ADS_H743_Sensors_Init(void)
{
    g_h743_sensor_diag.init_count++;
    g_h743_sensor_diag.initialized = 1u;

    /*
     * Real H743 sensor drivers are not active yet.
     * This remains 0 until IMU / magnetometer hardware details are confirmed
     * and the sensor backend has been validated.
     */
    g_h743_sensor_diag.sensor_backend_ready = 0u;
}

void ADS_H743_Sensors_Update(float dt_s)
{
    (void)dt_s;

    if (g_h743_sensor_diag.initialized == 0u)
    {
        return;
    }

    g_h743_sensor_diag.update_count++;
}

uint8_t ADS_H743_Sensors_IsInitialized(void)
{
    return g_h743_sensor_diag.initialized;
}

uint8_t ADS_H743_Sensors_IsBackendReady(void)
{
    return g_h743_sensor_diag.sensor_backend_ready;
}

ADS_H743_SensorDiagnostics ADS_H743_Sensors_GetDiagnostics(void)
{
    g_h743_sensor_diag.read_count++;
    return g_h743_sensor_diag;
}