#ifndef ADS_SENSOR_VALIDATE_H
#define ADS_SENSOR_VALIDATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ads_sensor_interface.h"

/*
 * ads_sensor_validate.h
 *
 * ADS sensor packet validation/sanitisation.
 *
 * Purpose:
 * - Prevent bad real sensor data from reaching the ADCS estimator.
 * - Keep validation independent from F3 fake sensors and H743 drivers.
 * - Clear validity flags if vectors are NaN, Inf, zero, or physically unreasonable.
 *
 * This module is hardware-free.
 */

typedef struct
{
    bool source_ok;
    bool gyro_ok;
    bool accel_ok;
    bool sun_ok;
    bool mag_ok;
    bool packet_ok;

    ADS_SensorSource last_source;

    uint32_t total_packets_checked;
    uint32_t bad_packets_count;

} ADS_SensorValidationResult;

void ADS_SensorValidate_Init(void);

ADS_SensorValidationResult ADS_SensorValidate_Apply(ADS_SensorPacket *packet);

ADS_SensorValidationResult ADS_SensorValidate_GetLastResult(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_SENSOR_VALIDATE_H */