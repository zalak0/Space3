#ifndef ADCS_HEALTH_H
#define ADCS_HEALTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    unsigned char gyro_fault;
    unsigned char sun_sensor_fault;
    unsigned char magnetometer_fault;
    unsigned char attitude_fault;
    unsigned char high_rate_fault;

    unsigned char any_fault;

} ADCS_HealthFlags;

typedef struct
{
    float angular_rate_rad_s;
    float detumble_entry_threshold_rad_s;
} ADCS_HealthModeTelemetry;

void ADCS_Health_Init(void);
void ADCS_Health_Update(
    const ADCS_Telemetry* adcs_telemetry,
    const ADCS_HealthModeTelemetry* mode_telemetry
);

ADCS_HealthFlags ADCS_Health_GetFlags(void);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_HEALTH_H */