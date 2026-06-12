#include "ads_timing.h"

#include <math.h>
#include <string.h>

static ADS_TimingStatus s_ads_timing_status;

void ADS_Timing_Init(float expected_dt_s, float tolerance_s)
{
    memset(&s_ads_timing_status, 0, sizeof(s_ads_timing_status));

    s_ads_timing_status.expected_dt_s = expected_dt_s;
    s_ads_timing_status.tolerance_s = tolerance_s;

    s_ads_timing_status.last_dt_s = expected_dt_s;
    s_ads_timing_status.min_dt_s = expected_dt_s;
    s_ads_timing_status.max_dt_s = expected_dt_s;

    s_ads_timing_status.last_dt_ok = true;
}

void ADS_Timing_Update(float dt_s)
{
    s_ads_timing_status.total_samples++;
    s_ads_timing_status.last_dt_s = dt_s;

    if (s_ads_timing_status.total_samples == 1u) {
        s_ads_timing_status.min_dt_s = dt_s;
        s_ads_timing_status.max_dt_s = dt_s;
    } else {
        if (dt_s < s_ads_timing_status.min_dt_s) {
            s_ads_timing_status.min_dt_s = dt_s;
        }

        if (dt_s > s_ads_timing_status.max_dt_s) {
            s_ads_timing_status.max_dt_s = dt_s;
        }
    }

    const float error_s = fabsf(dt_s - s_ads_timing_status.expected_dt_s);

    s_ads_timing_status.last_dt_ok =
        error_s <= s_ads_timing_status.tolerance_s;

    if (!s_ads_timing_status.last_dt_ok) {
        s_ads_timing_status.bad_dt_count++;
    }
}

ADS_TimingStatus ADS_Timing_GetStatus(void)
{
    return s_ads_timing_status;
}

bool ADS_Timing_IsOk(void)
{
    return s_ads_timing_status.last_dt_ok;
}