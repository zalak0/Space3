#ifndef ADS_STATUS_H
#define ADS_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * ads_status.h
 *
 * Portable ADS status counters and fault reason.
 *
 * Purpose:
 * - Track ADS task health independently of F3Discovery LEDs.
 * - Provide simple counters for later telemetry/diagnostics.
 * - Provide one compact reason code for the latest ADS unhealthy state.
 *
 * This module is hardware-free.
 */

typedef enum
{
    ADS_FAULT_REASON_NONE = 0,
    ADS_FAULT_REASON_NOT_INITIALIZED,
    ADS_FAULT_REASON_SENSOR_BACKEND_INIT,
    ADS_FAULT_REASON_SENSOR_PACKET,
    ADS_FAULT_REASON_RUNTIME_CHECK,
    ADS_FAULT_REASON_OUTPUT_INVALID,
    ADS_FAULT_REASON_TIMING,
    ADS_FAULT_REASON_CONFIG

} ADS_FaultReason;


typedef enum
{
    ADS_BACKEND_STATUS_UNKNOWN = 0,
    ADS_BACKEND_STATUS_FAKE_SELECTED = 1,
    ADS_BACKEND_STATUS_H743_REAL_SELECTED_NOT_READY = 2,
    ADS_BACKEND_STATUS_H743_REAL_SELECTED_READY = 3
} ADS_BackendStatus;

ADS_BackendStatus ADS_Status_GetBackendStatus(void);

typedef struct
{
    uint32_t total_cycles;
    uint32_t healthy_cycles;
    uint32_t fault_cycles;

    bool healthy;

    ADS_FaultReason last_fault_reason;

} ADS_Status;

void ADS_Status_Init(void);

void ADS_Status_Update(bool healthy, ADS_FaultReason fault_reason);

ADS_Status ADS_Status_Get(void);

ADS_FaultReason ADS_Status_GetLastFaultReason(void);

const char *ADS_Status_FaultReasonName(ADS_FaultReason reason);

#ifdef __cplusplus
}
#endif

#endif /* ADS_STATUS_H */