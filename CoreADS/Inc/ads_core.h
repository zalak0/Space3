#ifndef ADS_CORE_H
#define ADS_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs.h"
#include "ads_sensor_interface.h"

/*
 * ads_core.h
 *
 * ADS core wrapper around the existing ADCS estimator functions.
 *
 * Purpose:
 * - Keep ads_task.c clean.
 * - Keep ADCS_SetSensorInputs(), ADCS_Update(), and ADCS_GetTelemetry()
 *   grouped in one estimator-facing module.
 * - Make it easier later to swap/refactor the estimator without changing
 *   the scheduler/task layer.
 *
 * This module receives a validated ADS_SensorPacket and feeds it into the
 * existing ADCS estimator.
 */

void ADS_Core_Init(void);

ADCS_Telemetry ADS_Core_UpdateFromSensorPacket(
    const ADS_SensorPacket *sensor_packet,
    float dt_s
);

#ifdef __cplusplus
}
#endif

#endif /* ADS_CORE_H */