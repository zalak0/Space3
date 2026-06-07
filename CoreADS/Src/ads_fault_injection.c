#include "ads_fault_injection.h"

#include "ads_config.h"

#include <math.h>

/*
 * ADS fault injection module.
 *
 * F3Discovery:
 * - enabled by default for bench testing.
 *
 * H743 OBC:
 * - disabled by default through ADS_FAULT_INJECTION_ENABLE.
 */

static ADS_FaultInjectionMode s_fault_mode = ADS_FAULT_NONE;

void ADS_FaultInjection_Init(void)
{
    s_fault_mode = ADS_FAULT_NONE;
}

void ADS_FaultInjection_SetMode(ADS_FaultInjectionMode mode)
{
#if ADS_FAULT_INJECTION_ENABLE != 0
    s_fault_mode = mode;
#else
    (void)mode;
    s_fault_mode = ADS_FAULT_NONE;
#endif
}

ADS_FaultInjectionMode ADS_FaultInjection_GetMode(void)
{
    return s_fault_mode;
}

bool ADS_FaultInjection_IsActive(void)
{
#if ADS_FAULT_INJECTION_ENABLE != 0
    return s_fault_mode != ADS_FAULT_NONE;
#else
    return false;
#endif
}

void ADS_FaultInjection_Apply(ADS_SensorPacket *packet)
{
    if (packet == 0) {
        return;
    }

#if ADS_FAULT_INJECTION_ENABLE != 0

    switch (s_fault_mode)
    {
        case ADS_FAULT_NONE:
            break;

        case ADS_FAULT_INVALID_GYRO:
            packet->status.gyro_valid = 0u;
            break;

        case ADS_FAULT_INVALID_SUN:
            packet->status.sun_valid = 0u;
            break;

        case ADS_FAULT_INVALID_MAG:
            packet->status.mag_valid = 0u;
            break;

        case ADS_FAULT_NAN_GYRO:
            packet->gyro_rad_s.x = NAN;
            break;

        case ADS_FAULT_ZERO_SUN_VECTOR:
            packet->sun_body.x = 0.0f;
            packet->sun_body.y = 0.0f;
            packet->sun_body.z = 0.0f;
            break;

        case ADS_FAULT_ZERO_MAG_VECTOR:
            packet->mag_body_T.x = 0.0f;
            packet->mag_body_T.y = 0.0f;
            packet->mag_body_T.z = 0.0f;
            break;

        default:
            s_fault_mode = ADS_FAULT_NONE;
            break;
    }

#else

    s_fault_mode = ADS_FAULT_NONE;

#endif
}