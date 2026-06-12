#ifndef ADS_OUTPUT_H
#define ADS_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "adcs.h"
#include "quaternion.h"
#include "vector3.h"
#include "common/adcs_type.h"

/*
 * ads_output.h
 *
 * Stores the latest ADS output estimate.
 *
 * Purpose:
 * - Provide a clean place for ADS products to live.
 * - Keep ads_task.c from becoming the owner of published attitude state.
 * - Make later integration with a flight software context easier.
 *
 * This module is hardware-free.
 */

typedef struct
{
    Quaternion attitude_q;
    EulerAngles euler_rad;
    Vector3 euler_rates_rad_s;    /* x=phi_dot, y=theta_dot, z=psi_dot */
    Vector3 gyro_rad_s;

    Vector3 accel_body_measured_m_s2;
    Vector3 accel_body_predicted_m_s2;
    Vector3 accel_error;

    Vector3 sun_body_measured;
    Vector3 sun_body_predicted;
    Vector3 sun_error;

    Vector3 mag_body_measured_uT;
    Vector3 mag_body_predicted_uT;
    Vector3 mag_error;

    ADCS_SensorStatus sensor_status;

    float dt;
    unsigned long update_counter;

    bool valid;

} ADS_Output;

void ADS_Output_Init(void);

void ADS_Output_UpdateFromTelemetry(const ADCS_Telemetry *telemetry);

ADS_Output ADS_Output_GetLatest(void);

bool ADS_Output_IsValid(void);

EulerAngles ADS_Output_GetEulerRad(void);
Vector3 ADS_Output_GetEulerRatesRadS(void);
adcs_state_t ADS_Output_GetAdcsState(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_OUTPUT_H */