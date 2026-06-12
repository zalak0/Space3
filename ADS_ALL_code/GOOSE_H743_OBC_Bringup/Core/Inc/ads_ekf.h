#ifndef ADS_EKF_H
#define ADS_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "quaternion.h"
#include "vector3.h"

typedef struct
{
    Quaternion q_body_to_inertial;
    Vector3 gyro_bias_rad_s;

    float covariance_trace;
    uint32_t update_count;

    bool initialized;
    bool healthy;

} ADS_EKF_State;

typedef struct
{
    Vector3 gyro_rad_s;

    Vector3 accel_body_m_s2;
    Vector3 gravity_inertial_m_s2;

    Vector3 mag_body_T;
    Vector3 mag_inertial_T;

    bool gyro_valid;
    bool accel_valid;
    bool mag_valid;

    float dt_s;

} ADS_EKF_Input;

void ADS_EKF_Init(void);

bool ADS_EKF_PredictUpdate(const ADS_EKF_Input* input);

ADS_EKF_State ADS_EKF_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_EKF_H */