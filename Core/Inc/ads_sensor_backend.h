#ifndef ADS_SENSOR_BACKEND_H
#define ADS_SENSOR_BACKEND_H

#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "ads_sensor_interface.h"

/*
 * ads_sensor_backend.h
 *
 * Backend implementation interface for ADS sensor acquisition.
 *
 * ads_sensor_interface.c calls these functions.
 *
 * F3Discovery backend:
 * - implemented in ads_sensor_backend_fake.c
 *
 * H743 OBC backend:
 * - implemented in ads_sensor_backend_h743.c
 *
 * Only one backend should provide symbols for a given ADS_TARGET.
 */

bool ADS_SensorBackend_Init(void);

ADS_SensorPacket ADS_SensorBackend_UpdateAndRead(float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* ADS_SENSOR_BACKEND_H */