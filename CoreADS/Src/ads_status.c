#include "ads_status.h"

#include <string.h>

static ADS_Status s_ads_status;

void ADS_Status_Init(void)
{
    memset(&s_ads_status, 0, sizeof(s_ads_status));

    s_ads_status.healthy = false;
    s_ads_status.last_fault_reason = ADS_FAULT_REASON_NOT_INITIALIZED;
}

void ADS_Status_Update(bool healthy, ADS_FaultReason fault_reason)
{
    s_ads_status.total_cycles++;
    s_ads_status.healthy = healthy;

    if (healthy) {
        s_ads_status.healthy_cycles++;
        s_ads_status.last_fault_reason = ADS_FAULT_REASON_NONE;
    } else {
        s_ads_status.fault_cycles++;
        s_ads_status.last_fault_reason = fault_reason;
    }
}

ADS_Status ADS_Status_Get(void)
{
    return s_ads_status;
}

ADS_FaultReason ADS_Status_GetLastFaultReason(void)
{
    return s_ads_status.last_fault_reason;
}

const char *ADS_Status_FaultReasonName(ADS_FaultReason reason)
{
    switch (reason)
    {
        case ADS_FAULT_REASON_NONE:
            return "NONE";

        case ADS_FAULT_REASON_NOT_INITIALIZED:
            return "NOT_INITIALIZED";

        case ADS_FAULT_REASON_SENSOR_BACKEND_INIT:
            return "SENSOR_BACKEND_INIT";

        case ADS_FAULT_REASON_SENSOR_PACKET:
            return "SENSOR_PACKET";

        case ADS_FAULT_REASON_RUNTIME_CHECK:
            return "RUNTIME_CHECK";

        case ADS_FAULT_REASON_OUTPUT_INVALID:
            return "OUTPUT_INVALID";

        case ADS_FAULT_REASON_TIMING:
            return "TIMING";

        case ADS_FAULT_REASON_CONFIG:
            return "CONFIG";

        default:
            return "UNKNOWN";
    }
}

ADS_BackendStatus ADS_Status_GetBackendStatus(void)
{
#if defined(ADS_TARGET) && (ADS_TARGET == ADS_TARGET_H743_OBC)

#if defined(ADS_H743_SENSOR_BACKEND_READY) && (ADS_H743_SENSOR_BACKEND_READY == 1)
    return ADS_BACKEND_STATUS_H743_REAL_SELECTED_NOT_READY;
#else
    return ADS_BACKEND_STATUS_FAKE_SELECTED;
#endif

#else
    return ADS_BACKEND_STATUS_UNKNOWN;
#endif
}