#include "ads_build_info.h"

#include "ads_config.h"

/*
 * ads_build_info.c
 *
 * Converts compile-time ADS configuration into a small runtime-readable struct.
 */

ADS_BuildInfo ADS_BuildInfo_Get(void)
{
    ADS_BuildInfo info;

#if ADS_TARGET == ADS_TARGET_F3DISCOVERY

    info.target = ADS_BUILD_TARGET_F3DISCOVERY;
    info.is_f3discovery = true;
    info.is_h743_obc = false;

#elif ADS_TARGET == ADS_TARGET_H743_OBC

    info.target = ADS_BUILD_TARGET_H743_OBC;
    info.is_f3discovery = false;
    info.is_h743_obc = true;

#else

    info.target = ADS_BUILD_TARGET_UNKNOWN;
    info.is_f3discovery = false;
    info.is_h743_obc = false;

#endif

#if ADS_H743_SENSOR_BACKEND_READY != 0
    info.h743_sensor_backend_ready = true;
#else
    info.h743_sensor_backend_ready = false;
#endif

#if ADS_FAULT_INJECTION_ENABLE != 0
    info.fault_injection_enabled = true;
#else
    info.fault_injection_enabled = false;
#endif

    return info;
}

const char *ADS_BuildInfo_GetTargetName(void)
{
#if ADS_TARGET == ADS_TARGET_F3DISCOVERY
    return "F3DISCOVERY";
#elif ADS_TARGET == ADS_TARGET_H743_OBC
    return "H743_OBC";
#else
    return "UNKNOWN";
#endif
}