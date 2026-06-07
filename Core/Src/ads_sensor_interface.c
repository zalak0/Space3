#include "ads_sensor_interface.h"

#include "ads_sensor_backend.h"

#include <string.h>

/*
 * ADS sensor interface.
 *
 * This is the stable interface used by ads_task.c.
 *
 * It does not know whether the data comes from:
 * - F3Discovery fake sensors
 * - H743 real sensors
 * - H743 safe stub
 *
 * That selection is handled by the backend files.
 */

static ADS_SensorInterfaceStatus s_sensor_interface_status;

static Vector3 ADS_SensorPacket_MakeZeroVector(void)
{
    Vector3 v;

    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;

    return v;
}

ADS_SensorPacket ADS_SensorPacket_MakeInvalid(ADS_SensorSource source)
{
    ADS_SensorPacket packet;

    packet.gyro_rad_s = ADS_SensorPacket_MakeZeroVector();
    packet.sun_body = ADS_SensorPacket_MakeZeroVector();
    packet.mag_body_T = ADS_SensorPacket_MakeZeroVector();

    packet.status.gyro_valid = 0u;
    packet.status.sun_valid = 0u;
    packet.status.mag_valid = 0u;

    packet.source_update_count = 0u;
    packet.source_time_s = 0.0f;

    packet.source = source;

    return packet;
}

bool ADS_SensorInterface_Init(void)
{
    memset(&s_sensor_interface_status, 0, sizeof(s_sensor_interface_status));

    bool backend_ok = ADS_SensorBackend_Init();

    s_sensor_interface_status.initialized = backend_ok;
    s_sensor_interface_status.last_source = ADS_SENSOR_SOURCE_UNKNOWN;
    s_sensor_interface_status.last_source_update_count = 0u;
    s_sensor_interface_status.last_source_time_s = 0.0f;

    s_sensor_interface_status.last_status.gyro_valid = 0u;
    s_sensor_interface_status.last_status.sun_valid = 0u;
    s_sensor_interface_status.last_status.mag_valid = 0u;

    s_sensor_interface_status.last_packet =
        ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_UNKNOWN);

    return backend_ok;
}

ADS_SensorPacket ADS_SensorInterface_UpdateAndRead(float dt_s)
{
    ADS_SensorPacket packet;

    if (!s_sensor_interface_status.initialized) {
        packet = ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_UNKNOWN);
    }
    else {
        packet = ADS_SensorBackend_UpdateAndRead(dt_s);
    }

    s_sensor_interface_status.last_source = packet.source;
    s_sensor_interface_status.last_source_update_count =
        packet.source_update_count;
    s_sensor_interface_status.last_source_time_s =
        packet.source_time_s;
    s_sensor_interface_status.last_status =
        packet.status;
    s_sensor_interface_status.last_packet =
        packet;

    return packet;
}

ADS_SensorInterfaceStatus ADS_SensorInterface_GetStatus(void){
    return s_sensor_interface_status;
}

const char *ADS_SensorSource_Name(ADS_SensorSource source)
{
    switch (source)
    {
        case ADS_SENSOR_SOURCE_UNKNOWN:
            return "UNKNOWN";

        case ADS_SENSOR_SOURCE_FAKE:
            return "FAKE";

        case ADS_SENSOR_SOURCE_H743_REAL:
            return "H743_REAL";

        case ADS_SENSOR_SOURCE_H743_STUB:
            return "H743_STUB";

        default:
            return "INVALID";
    }
}