#include "ads_core.h"

#include "adcs.h"
#include "calibration.h"
#include "adcs_health.h"

#define ADS_CORE_TESLA_TO_MICROTESLA (1.0e6f)

static Vector3 ADS_Core_VectorScale(Vector3 v, float scale)
{
    Vector3 out;
    out.x = v.x * scale;
    out.y = v.y * scale;
    out.z = v.z * scale;
    return out;
}

static void ADS_Core_TestStep(float dt_s)
{
    /*
     * Hardware-free test hook retained for bring-up compatibility.
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
    ADCS_SetDt(dt_s);

    if (sensor_packet == 0)
    {
        ADS_SensorPacket invalid_packet =
            ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_UNKNOWN);

        ADCS_SetSensorInputs(
            invalid_packet.gyro_rad_s,
            invalid_packet.accel_body_m_s2,
            invalid_packet.sun_body,
            ADS_Core_VectorScale(
                invalid_packet.mag_body_T,
                ADS_CORE_TESLA_TO_MICROTESLA
            ),
            invalid_packet.status
        );
    }
    else
    {
        ADCS_SetSensorInputs(
            sensor_packet->gyro_rad_s,
            sensor_packet->accel_body_m_s2,
            sensor_packet->sun_body,
            ADS_Core_VectorScale(
                sensor_packet->mag_body_T,
                ADS_CORE_TESLA_TO_MICROTESLA
            ),
            sensor_packet->status
        );
    }

    ADS_Core_TestStep(dt_s);

    ADCS_Update();

    return ADCS_GetTelemetry();
}