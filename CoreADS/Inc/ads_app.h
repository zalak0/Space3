#ifndef ADS_APP_H
#define ADS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ads_output.h"
#include "ads_runtime_check.h"
#include "ads_status.h"
#include "ads_timing.h"
#include "ads_sensor_validate.h"
#include "ads_config_check.h"
#include "ads_build_info.h"
#include "ads_sensor_interface.h"
#include "adcs.h"

/*
 * ads_app.h
 *
 * High-level ADS application interface.
 *
 * Purpose:
 * - Provide the clean API that main.c or the future flight state machine calls.
 * - Hide ADS task timing constants from main.c.
 * - Keep ADS task internals behind one subsystem-level interface.
 *
 * This module is hardware-free.
 */


typedef struct
{
    bool ads_initialized;
    bool ads_healthy;
    ADS_FaultReason fault_reason;

    bool sensor_backend_initialized;

    bool output_valid;
    bool sensor_packet_ok;
    bool runtime_ok;
    bool timing_ok;
    bool config_ok;

    uint32_t loop_count;
    uint32_t total_cycles;
    uint32_t healthy_cycles;
    uint32_t fault_cycles;

    ADS_SensorSource sensor_source;
    uint32_t sensor_source_update_count;
    float sensor_source_time_s;

    ADCS_SensorStatus sensor_status;

} ADS_AppHealthSummary;

bool ADS_App_Init(void);

bool ADS_App_Run(void);

bool ADS_App_IsHealthy(void);

bool ADS_App_IsInitialized(void);

bool ADS_App_IsSensorBackendInitialized(void);

uint32_t ADS_App_GetLoopCount(void);

ADS_Output ADS_App_GetOutput(void);

ADS_Status ADS_App_GetStatus(void);

ADS_RuntimeCheckResult ADS_App_GetRuntimeCheck(void);

ADS_TimingStatus ADS_App_GetTimingStatus(void);

ADS_SensorValidationResult ADS_App_GetSensorValidation(void);

ADS_ConfigCheckResult ADS_App_GetConfigCheck(void);

ADS_SensorInterfaceStatus ADS_App_GetSensorInterfaceStatus(void);

ADS_AppHealthSummary ADS_App_GetHealthSummary(void);

ADS_BuildInfo ADS_App_GetBuildInfo(void);

ADS_FaultReason ADS_App_GetLastFaultReason(void);

const char *ADS_App_GetLastFaultReasonName(void);

const char *ADS_App_GetSensorSourceName(void);

const char *ADS_App_GetBuildTargetName(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_APP_H */