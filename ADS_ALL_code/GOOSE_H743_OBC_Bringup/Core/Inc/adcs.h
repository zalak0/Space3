#ifndef ADCS_H
#define ADCS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vector3.h"
#include "quaternion.h"

typedef struct
{
    unsigned char gyro_valid;
    unsigned char accel_valid;
    unsigned char sun_valid;      /* retained for compatibility; unused in no-sun build */
    unsigned char mag_valid;
} ADCS_SensorStatus;

typedef struct
{
    Quaternion attitude_q;

    Vector3 gyro_rad_s;
    Vector3 gyro_bias_rad_s;

    Vector3 euler_rates_rad_s;    /* x=phi_dot, y=theta_dot, z=psi_dot */

    Vector3 accel_body_measured_m_s2;
    Vector3 accel_body_predicted_m_s2;
    Vector3 gravity_inertial_m_s2;
    Vector3 accel_error;

    Vector3 sun_body_measured;
    Vector3 sun_body_predicted;
    Vector3 sun_inertial;
    Vector3 sun_error;

    Vector3 mag_body_measured_uT;
    Vector3 mag_body_predicted_uT;
    Vector3 mag_inertial_uT;
    Vector3 mag_error;

    ADCS_SensorStatus sensor_status;

    float dt;
    float sun_correction_gain;
    float mag_correction_gain;

    unsigned long update_counter;

} ADCS_State;

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

} ADCS_Telemetry;


void ADCS_Init(void);
void ADCS_Update(void);

void ADCS_SetSensorInputs(
    Vector3 gyro_rad_s,
    Vector3 accel_body_measured_m_s2,
    Vector3 sun_body_measured,
    Vector3 mag_body_measured_uT,
    ADCS_SensorStatus status
);

void ADCS_SetInertialReferences(
    Vector3 gravity_inertial_m_s2,
    Vector3 mag_inertial_uT
);

void ADCS_SetDt(float dt);

const ADCS_State* ADCS_GetState(void);
ADCS_Telemetry ADCS_GetTelemetry(void);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_H */