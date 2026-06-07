#include "ads_core.h"

#include "adcs.h"
#include "calibration.h"
#include "adcs_health.h"

/*
 * ads_core.c
 *
 * Hardware-free ADS estimator wrapper.
 *
 * This file owns the calls into the existing ADCS core:
 * - Calibration_Init()
 * - ADCS_Health_Init()
 * - ADCS_Init()
 * - ADCS_SetSensorInputs()
 * - ADCS_Update()
 * - ADCS_GetTelemetry()
 */

static void ADS_Core_TestStep(float dt_s)
{
    /*
     * F3Discovery ADS bring-up test hook.
     *
     * This replaces the old ADCS_TestStep() that originally lived in main.c.
     * Keep this function hardware-free.
     */

    (void)dt_s;
}

void ADS_Core_Init(void)
{
    Calibration_Init();
    ADCS_Health_Init();
    ADCS_Init();
}

ADCS_Telemetry ADS_Core_UpdateFromSensorPacket(
    const ADS_SensorPacket *sensor_packet,
    float dt_s
)
{
    if (sensor_packet == 0) {
        ADS_SensorPacket invalid_packet =
            ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_UNKNOWN);

        ADCS_SetSensorInputs(
            invalid_packet.gyro_rad_s,
            invalid_packet.sun_body,
            invalid_packet.mag_body_T,
            invalid_packet.status
        );
    }
    else {
        ADCS_SetSensorInputs(
            sensor_packet->gyro_rad_s,
            sensor_packet->sun_body,
            sensor_packet->mag_body_T,
            sensor_packet->status
        );
    }

    ADS_Core_TestStep(dt_s);

    ADCS_Update();

    return ADCS_GetTelemetry();
}