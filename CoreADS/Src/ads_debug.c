#include "ads_debug.h"

#include <string.h>

static ADS_DebugSnapshot s_ads_debug_snapshot;

void ADS_Debug_Init(void)
{
    memset(&s_ads_debug_snapshot, 0, sizeof(s_ads_debug_snapshot));
}

void ADS_Debug_UpdateFakeSensorInfo(
    uint32_t fake_sensor_update_count,
    float fake_sensor_sim_time_s,
    ADCS_SensorStatus sensor_status
)
{
    s_ads_debug_snapshot.fake_sensor_update_count = fake_sensor_update_count;
    s_ads_debug_snapshot.fake_sensor_sim_time_s = fake_sensor_sim_time_s;
    s_ads_debug_snapshot.last_sensor_status = sensor_status;
}

void ADS_Debug_UpdateTelemetry(ADCS_Telemetry telemetry)
{
    s_ads_debug_snapshot.ads_loop_count++;
    s_ads_debug_snapshot.last_telemetry = telemetry;
}

ADS_DebugSnapshot ADS_Debug_GetSnapshot(void)
{
    return s_ads_debug_snapshot;
}