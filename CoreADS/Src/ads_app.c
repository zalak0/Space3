#include "ads_app.h"

#include "ads_config.h"
#include "ads_task.h"

/*
 * ads_app.c
 *
 * High-level ADS application wrapper.
 *
 * This file owns the configured ADS task timestep.
 * main.c should not need to know the numeric ADS dt.
 */

bool ADS_App_Init(void)
{
    return ADS_Task_Init();
}

bool ADS_App_Run(void)
{
    return ADS_Task_Run100Hz(ADS_TASK_DT_S);
}

bool ADS_App_IsHealthy(void)
{
    return ADS_Task_IsHealthy();
}

bool ADS_App_IsInitialized(void)
{
    return ADS_Task_IsInitialized();
}

bool ADS_App_IsSensorBackendInitialized(void)
{
    return ADS_Task_IsSensorBackendInitialized();
}

uint32_t ADS_App_GetLoopCount(void)
{
    return ADS_Task_GetLoopCount();
}

ADS_Output ADS_App_GetOutput(void)
{
    return ADS_Task_GetOutput();
}

ADS_Status ADS_App_GetStatus(void)
{
    return ADS_Task_GetStatus();
}

ADS_FaultReason ADS_App_GetLastFaultReason(void)
{
    return ADS_Status_GetLastFaultReason();
}

const char *ADS_App_GetLastFaultReasonName(void)
{
    return ADS_Status_FaultReasonName(ADS_App_GetLastFaultReason());
}

ADS_RuntimeCheckResult ADS_App_GetRuntimeCheck(void)
{
    return ADS_Task_GetRuntimeCheck();
}

ADS_TimingStatus ADS_App_GetTimingStatus(void)
{
    return ADS_Task_GetTimingStatus();
}

ADS_SensorValidationResult ADS_App_GetSensorValidation(void)
{
    return ADS_Task_GetSensorValidation();
}

ADS_ConfigCheckResult ADS_App_GetConfigCheck(void)
{
    return ADS_Task_GetConfigCheck();
}

ADS_SensorInterfaceStatus ADS_App_GetSensorInterfaceStatus(void)
{
    return ADS_Task_GetSensorInterfaceStatus();
}

ADS_AppHealthSummary ADS_App_GetHealthSummary(void)
{
    ADS_AppHealthSummary summary;

    ADS_Status status = ADS_App_GetStatus();
    ADS_RuntimeCheckResult runtime_check = ADS_App_GetRuntimeCheck();
    ADS_TimingStatus timing_status = ADS_App_GetTimingStatus();
    ADS_SensorValidationResult sensor_validation = ADS_App_GetSensorValidation();
    ADS_ConfigCheckResult config_check = ADS_App_GetConfigCheck();
    ADS_SensorInterfaceStatus sensor_interface_status =
        ADS_App_GetSensorInterfaceStatus();

    summary.ads_initialized = ADS_App_IsInitialized();
    summary.ads_healthy = ADS_App_IsHealthy();
    summary.fault_reason = ADS_App_GetLastFaultReason();

    summary.sensor_backend_initialized =
        ADS_App_IsSensorBackendInitialized();

    summary.output_valid = ADS_Output_IsValid();
    summary.sensor_packet_ok = sensor_validation.packet_ok;
    summary.runtime_ok = runtime_check.ads_ok;
    summary.timing_ok = timing_status.last_dt_ok;
    summary.config_ok = config_check.config_ok;

    summary.loop_count = ADS_App_GetLoopCount();
    summary.total_cycles = status.total_cycles;
    summary.healthy_cycles = status.healthy_cycles;
    summary.fault_cycles = status.fault_cycles;

    summary.sensor_source = sensor_interface_status.last_source;
    summary.sensor_source_update_count =
        sensor_interface_status.last_source_update_count;
    summary.sensor_source_time_s =
        sensor_interface_status.last_source_time_s;
    summary.sensor_status =
        sensor_interface_status.last_status;

    return summary;
}

ADS_BuildInfo ADS_App_GetBuildInfo(void)
{
    return ADS_BuildInfo_Get();
}

const char *ADS_App_GetBuildTargetName(void)
{
    return ADS_BuildInfo_GetTargetName();
}

const char *ADS_App_GetSensorSourceName(void)
{
    ADS_SensorInterfaceStatus sensor_interface_status =
        ADS_App_GetSensorInterfaceStatus();

    return ADS_SensorSource_Name(sensor_interface_status.last_source);
}