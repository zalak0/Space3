#include "ads_task.h"
#include <stdbool.h>

#include "adcs.h"
#include "ads_core.h"
#include "ads_sensor_interface.h"
#include "ads_debug.h"
#include "ads_runtime_check.h"
#include "ads_output.h"
#include "ads_fault_injection.h"
#include "ads_status.h"
#include "ads_timing.h"
#include "ads_config.h"
#include "ads_sensor_validate.h"
#include "ads_config_check.h"

static uint32_t s_ads_task_loop_count = 0u;
static bool s_ads_task_healthy = false;
static bool s_ads_task_initialized = false;
static bool s_ads_sensor_backend_initialized = false;

static ADS_FaultReason ADS_Task_DetermineFaultReason(
    ADS_SensorValidationResult sensor_validation,
    ADS_RuntimeCheckResult runtime_check
)
{
    if (!s_ads_task_initialized) {
        return ADS_FAULT_REASON_NOT_INITIALIZED;
    }

    if (!s_ads_sensor_backend_initialized) {
        return ADS_FAULT_REASON_SENSOR_BACKEND_INIT;
    }

    if (!ADS_ConfigCheck_IsOk()) {
        return ADS_FAULT_REASON_CONFIG;
    }

    if (!ADS_Timing_IsOk()) {
        return ADS_FAULT_REASON_TIMING;
    }

    if (!sensor_validation.packet_ok) {
        return ADS_FAULT_REASON_SENSOR_PACKET;
    }

    if (!runtime_check.ads_ok) {
        return ADS_FAULT_REASON_RUNTIME_CHECK;
    }

    if (!ADS_Output_IsValid()) {
        return ADS_FAULT_REASON_OUTPUT_INVALID;
    }

    return ADS_FAULT_REASON_NONE;
}

bool ADS_Task_Init(void)
{
    s_ads_task_loop_count = 0u;
    s_ads_task_healthy = false;
    s_ads_task_initialized = false;
    s_ads_sensor_backend_initialized = false;

    ADS_Core_Init();
    s_ads_sensor_backend_initialized = ADS_SensorInterface_Init();    ADS_SensorInterface_Init();
    ADS_SensorValidate_Init();
    ADS_Debug_Init();
    ADS_RuntimeCheck_Init();
    ADS_Output_Init();
    ADS_FaultInjection_Init();
    ADS_Status_Init();
    ADS_Timing_Init(ADS_TASK_DT_S, ADS_TASK_DT_TOLERANCE_S);
    ADS_ConfigCheck_Init();

    s_ads_task_initialized =
        s_ads_sensor_backend_initialized &&
        ADS_ConfigCheck_IsOk();

    return s_ads_task_initialized;
}

bool ADS_Task_Run100Hz(float dt_s)
{
    if (!s_ads_task_initialized) {
        s_ads_task_healthy = false;
        ADS_Status_Update(false, ADS_FAULT_REASON_NOT_INITIALIZED);
        return false;
    }
    
    if (dt_s < 0.0f) {
        dt_s = 0.0f;
    }

    ADS_Timing_Update(dt_s);

    ADS_SensorPacket sensor_packet = ADS_SensorInterface_UpdateAndRead(dt_s);

    ADS_FaultInjection_Apply(&sensor_packet);

    ADS_SensorValidationResult sensor_validation =
        ADS_SensorValidate_Apply(&sensor_packet);

    ADS_Debug_UpdateFakeSensorInfo(
        sensor_packet.source_update_count,
        sensor_packet.source_time_s,
        sensor_packet.status
    );

    ADCS_Telemetry telemetry = ADS_Core_UpdateFromSensorPacket(
        &sensor_packet,
        dt_s
    );

    ADS_Debug_UpdateTelemetry(telemetry);
    ADS_Output_UpdateFromTelemetry(&telemetry);

    ADS_RuntimeCheckResult runtime_check = ADS_RuntimeCheck_Evaluate(
        &telemetry,
        sensor_packet.source_update_count
    );

    s_ads_task_healthy =
        sensor_validation.packet_ok &&
        runtime_check.ads_ok &&
        ADS_Output_IsValid() &&
        ADS_Timing_IsOk() &&
        ADS_ConfigCheck_IsOk();

    ADS_FaultReason fault_reason = ADS_Task_DetermineFaultReason(
        sensor_validation,
        runtime_check
    );

    ADS_Status_Update(s_ads_task_healthy, fault_reason);

    s_ads_task_loop_count++;

    return s_ads_task_healthy;
}

uint32_t ADS_Task_GetLoopCount(void)
{
    return s_ads_task_loop_count;
}

bool ADS_Task_IsHealthy(void)
{
    return s_ads_task_healthy;
}

bool ADS_Task_IsInitialized(void)
{
    return s_ads_task_initialized;
}

bool ADS_Task_IsSensorBackendInitialized(void)
{
    return s_ads_sensor_backend_initialized;
}

ADS_Output ADS_Task_GetOutput(void)
{
    return ADS_Output_GetLatest();
}

ADS_Status ADS_Task_GetStatus(void)
{
    return ADS_Status_Get();
}

ADS_RuntimeCheckResult ADS_Task_GetRuntimeCheck(void)
{
    return ADS_RuntimeCheck_GetLastResult();
}

ADS_TimingStatus ADS_Task_GetTimingStatus(void)
{
    return ADS_Timing_GetStatus();
}

ADS_SensorValidationResult ADS_Task_GetSensorValidation(void)
{
    return ADS_SensorValidate_GetLastResult();
}

ADS_ConfigCheckResult ADS_Task_GetConfigCheck(void)
{
    return ADS_ConfigCheck_GetLastResult();
}

ADS_SensorInterfaceStatus ADS_Task_GetSensorInterfaceStatus(void)
{
    return ADS_SensorInterface_GetStatus();
}