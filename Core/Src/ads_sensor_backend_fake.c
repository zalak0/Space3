#include "ads_sensor_backend.h"
#include "ads_sensor_interface.h"
#include "ads_fake_sensors.h"
#include "ads_config.h"

#if (ADS_TARGET == ADS_TARGET_F3DISCOVERY) || \
    ((ADS_TARGET == ADS_TARGET_H743_OBC) && \
     (ADS_H743_SENSOR_BACKEND_READY == 0) && \
     (ADS_ALLOW_FAKE_BACKEND_ON_H743 == 1))

bool ADS_SensorBackend_Init(void)
{
    ADS_FakeSensors_Init();
    return true;
}

ADS_SensorPacket ADS_SensorBackend_UpdateAndRead(float dt_s)
{
#if defined(ADS_FORCE_FAKE_SENSOR_FAULT) && (ADS_FORCE_FAKE_SENSOR_FAULT == 1)
    (void)dt_s;
    return ADS_SensorPacket_MakeInvalid(ADS_SENSOR_SOURCE_FAKE);
#else
    ADS_FakeSensors_Update(dt_s);

    ADS_SensorPacket packet;

    packet.gyro_rad_s = ADS_FakeSensors_GetGyroRadS();
    packet.sun_body = ADS_FakeSensors_GetSunBody();
    packet.mag_body_T = ADS_FakeSensors_GetMagBody();
    packet.status = ADS_FakeSensors_GetStatus();

    packet.source_update_count = ADS_FakeSensors_GetUpdateCount();
    packet.source_time_s = ADS_FakeSensors_GetSimTime();

    packet.source = ADS_SENSOR_SOURCE_FAKE;

    return packet;
#endif
}

#endif