#ifndef ADS_TIMING_H
#define ADS_TIMING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * ads_timing.h
 *
 * Portable ADS task timing monitor.
 *
 * Purpose:
 * - Track the dt value passed into ADS_Task_Run100Hz().
 * - Detect obviously bad dt values during bring-up.
 * - Keep timing diagnostics separate from main.c.
 *
 * This module is hardware-free.
 */

typedef struct
{
    float expected_dt_s;
    float tolerance_s;

    float last_dt_s;
    float min_dt_s;
    float max_dt_s;

    uint32_t total_samples;
    uint32_t bad_dt_count;

    bool last_dt_ok;

} ADS_TimingStatus;

void ADS_Timing_Init(float expected_dt_s, float tolerance_s);

void ADS_Timing_Update(float dt_s);

ADS_TimingStatus ADS_Timing_GetStatus(void);

bool ADS_Timing_IsOk(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_TIMING_H */