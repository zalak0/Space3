#ifndef ADS_FAKE_SENSORS_H
#define ADS_FAKE_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "adcs.h"
#include "vector3.h"

/*
 * F3Discovery fake ADS sensor source.
 *
 * Provides fake gyro, sun, and magnetometer measurements for ADCS bring-up.
 * This module is hardware-free and safe for the F3Discovery board.
 */

void ADS_FakeSensors_Init(void);

void ADS_FakeSensors_Update(float dt_s);

Vector3 ADS_FakeSensors_GetGyroRadS(void);

Vector3 ADS_FakeSensors_GetAccelBody(void);

Vector3 ADS_FakeSensors_GetSunBody(void);

Vector3 ADS_FakeSensors_GetMagBody(void);

ADCS_SensorStatus ADS_FakeSensors_GetStatus(void);

uint32_t ADS_FakeSensors_GetUpdateCount(void);

float ADS_FakeSensors_GetSimTime(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_FAKE_SENSORS_H */