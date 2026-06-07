#ifndef ADS_FAULT_INJECTION_H
#define ADS_FAULT_INJECTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ads_sensor_interface.h"

/*
 * ads_fault_injection.h
 *
 * F3Discovery ADS bring-up fault injection.
 *
 * Purpose:
 * - Allow deliberate fake sensor/telemetry faults during bench testing.
 * - Prove runtime checks and LED fault indication work.
 * - Keep fault injection out of main.c and out of the ADCS estimator.
 *
 * Default state:
 * - all faults disabled
 *
 * This module is hardware-free.
 */

typedef enum
{
    ADS_FAULT_NONE = 0,
    ADS_FAULT_INVALID_GYRO,
    ADS_FAULT_INVALID_SUN,
    ADS_FAULT_INVALID_MAG,
    ADS_FAULT_NAN_GYRO,
    ADS_FAULT_ZERO_SUN_VECTOR,
    ADS_FAULT_ZERO_MAG_VECTOR
} ADS_FaultInjectionMode;

void ADS_FaultInjection_Init(void);

void ADS_FaultInjection_SetMode(ADS_FaultInjectionMode mode);

ADS_FaultInjectionMode ADS_FaultInjection_GetMode(void);

void ADS_FaultInjection_Apply(ADS_SensorPacket *packet);

bool ADS_FaultInjection_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_FAULT_INJECTION_H */