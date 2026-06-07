#ifndef ADS_RUNTIME_CHECK_H
#define ADS_RUNTIME_CHECK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "adcs.h"

/*
 * ads_runtime_check.h
 *
 * Lightweight ADS runtime checks for F3Discovery bring-up.
 *
 * Purpose:
 * - Detect obvious estimator/telemetry failures.
 * - Check that fake sensor validity flags propagate through ADCS telemetry.
 * - Check that ADCS_Update() is actually advancing telemetry.update_counter.
 * - Keep checks separate from main.c.
 * - Keep checks hardware-free and portable.
 *
 * This does not replace the real ADCS health module.
 * It is a bring-up sanity layer.
 */

typedef struct
{
    bool telemetry_finite;
    bool quaternion_norm_ok;
    bool fake_sensor_count_advancing;
    bool telemetry_update_counter_advancing;
    bool gyro_status_valid;
    bool sun_status_valid;
    bool mag_status_valid;
    bool sensor_status_valid;
    bool ads_ok;

    float quaternion_norm;

    uint32_t total_check_count;
    uint32_t bad_check_count;

    uint32_t last_fake_sensor_update_count;
    unsigned long last_telemetry_update_counter;

} ADS_RuntimeCheckResult;

void ADS_RuntimeCheck_Init(void);

ADS_RuntimeCheckResult ADS_RuntimeCheck_Evaluate(
    const ADCS_Telemetry *telemetry,
    uint32_t fake_sensor_update_count
);

ADS_RuntimeCheckResult ADS_RuntimeCheck_GetLastResult(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_RUNTIME_CHECK_H */