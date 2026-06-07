#ifndef ADS_SENSOR_INTERFACE_H
#define ADS_SENSOR_INTERFACE_H

#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "adcs.h"
#include "vector3.h"

/*
 * ads_sensor_interface.h
 *
 * ADS sensor interface layer.
 *
 * Purpose:
 * - Give ads_task.c one clean source of ADS sensor measurements.
 * - Hide whether the data comes from fake F3Discovery sensors or real H743 hardware.
 * - Keep the ADCS/ADS core independent from board-specific drivers.
 *
  * Current bring-up backend:
 * - fake sensors, usable on F3Discovery and H743
 *
 * Future H743 real backend:
 * - IMU gyro
 * - magnetometer driver
 * - optional dedicated sun sensor only if added later
 *
 * Photodiode inputs are intentionally not part of the ADS path.
 */

typedef enum
{
    ADS_SENSOR_SOURCE_UNKNOWN = 0,
    ADS_SENSOR_SOURCE_FAKE = 1,
    ADS_SENSOR_SOURCE_H743_REAL = 2,
    ADS_SENSOR_SOURCE_H743_STUB = 3
} ADS_SensorSource;

typedef struct
{
    Vector3 gyro_rad_s;
    Vector3 sun_body;
    Vector3 mag_body_T;
    ADCS_SensorStatus status;

    uint32_t source_update_count;
    float source_time_s;

    ADS_SensorSource source;

} ADS_SensorPacket;

typedef struct
{
    bool initialized;

    ADS_SensorSource last_source;
    uint32_t last_source_update_count;
    float last_source_time_s;

    ADCS_SensorStatus last_status;

    ADS_SensorPacket last_packet;

} ADS_SensorInterfaceStatus;

bool ADS_SensorInterface_Init(void);

ADS_SensorPacket ADS_SensorInterface_UpdateAndRead(float dt_s);

ADS_SensorInterfaceStatus ADS_SensorInterface_GetStatus(void);

ADS_SensorPacket ADS_SensorPacket_MakeInvalid(ADS_SensorSource source);

const char *ADS_SensorSource_Name(ADS_SensorSource source);

#ifdef __cplusplus
}
#endif

#endif /* ADS_SENSOR_INTERFACE_H */