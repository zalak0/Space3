#ifndef ADS_DEBUG_H
#define ADS_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "adcs.h"

/*
 * ADS debug snapshot module.
 *
 * Purpose:
 * - Store the latest ADCS telemetry in one global debug structure.
 * - Store simple counters for F3Discovery bring-up.
 * - Keep main.c from filling up with many scattered debug globals.
 *
 * This module is safe for F3Discovery.
 * It does not touch real hardware.
 */

typedef struct
{
    uint32_t ads_loop_count;
    uint32_t fake_sensor_update_count;
    float fake_sensor_sim_time_s;

    ADCS_SensorStatus last_sensor_status;
    ADCS_Telemetry last_telemetry;

} ADS_DebugSnapshot;

void ADS_Debug_Init(void);

void ADS_Debug_UpdateFakeSensorInfo(
    uint32_t fake_sensor_update_count,
    float fake_sensor_sim_time_s,
    ADCS_SensorStatus sensor_status
);

void ADS_Debug_UpdateTelemetry(ADCS_Telemetry telemetry);

ADS_DebugSnapshot ADS_Debug_GetSnapshot(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_DEBUG_H */