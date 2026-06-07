#ifndef ADS_BUILD_INFO_H
#define ADS_BUILD_INFO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/*
 * ads_build_info.h
 *
 * ADS build/target information.
 *
 * Purpose:
 * - Expose the selected ADS target at runtime.
 * - Make F3/H743 backend selection visible to telemetry/debug/state-machine code.
 * - Avoid external modules needing to inspect preprocessor macros directly.
 *
 * This module is hardware-free.
 */

typedef enum
{
    ADS_BUILD_TARGET_UNKNOWN = 0,
    ADS_BUILD_TARGET_F3DISCOVERY = 1,
    ADS_BUILD_TARGET_H743_OBC = 2

} ADS_BuildTarget;

typedef struct
{
    ADS_BuildTarget target;

    bool is_f3discovery;
    bool is_h743_obc;

    bool h743_sensor_backend_ready;
    bool fault_injection_enabled;

} ADS_BuildInfo;

ADS_BuildInfo ADS_BuildInfo_Get(void);

const char *ADS_BuildInfo_GetTargetName(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_BUILD_INFO_H */