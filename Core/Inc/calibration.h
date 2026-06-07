#ifndef CALIBRATION_H
#define CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vector3.h"

typedef struct
{
    Vector3 gyro_bias_rad_s;
    Vector3 mag_bias_uT;
    Vector3 mag_scale;

} IMU_Calibration;

typedef struct
{
    float pos_x_gain;
    float neg_x_gain;
    float pos_y_gain;
    float neg_y_gain;
    float pos_z_gain;
    float neg_z_gain;

    float pos_x_dark;
    float neg_x_dark;
    float pos_y_dark;
    float neg_y_dark;
    float pos_z_dark;
    float neg_z_dark;

    float min_valid_intensity;

} SunSensor_Calibration;

typedef struct
{
    IMU_Calibration imu;
    SunSensor_Calibration sun;

} ADCS_Calibration;

void Calibration_Init(void);

const ADCS_Calibration* Calibration_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_H */