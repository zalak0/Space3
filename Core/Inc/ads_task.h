#ifndef ADS_TASK_H
#define ADS_TASK_H

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
#include "ads_sensor_interface.h"

/*
 * ads_task.h
 *
 * Public ADS task interface.
 *
 * Purpose:
 * - Keep main.c clean.
 * - Provide one clean entry point for running ADS at 100 Hz.
 * - Provide simple accessors for ADS output, health, status, and checks.
 *
 * This module is hardware-free except for whatever lower-level backend
 * is selected by ads_sensor_interface.c.
 */

bool ADS_Task_Init(void);

bool ADS_Task_Run100Hz(float dt_s);

uint32_t ADS_Task_GetLoopCount(void);

bool ADS_Task_IsHealthy(void);

bool ADS_Task_IsInitialized(void);

bool ADS_Task_IsSensorBackendInitialized(void);

ADS_Output ADS_Task_GetOutput(void);

ADS_Status ADS_Task_GetStatus(void);

ADS_RuntimeCheckResult ADS_Task_GetRuntimeCheck(void);

ADS_TimingStatus ADS_Task_GetTimingStatus(void);

ADS_SensorValidationResult ADS_Task_GetSensorValidation(void);

ADS_ConfigCheckResult ADS_Task_GetConfigCheck(void);

ADS_SensorInterfaceStatus ADS_Task_GetSensorInterfaceStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_TASK_H */