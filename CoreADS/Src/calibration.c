#include "calibration.h"

static ADCS_Calibration calibration;

void Calibration_Init(void)
{
    calibration.imu.gyro_bias_rad_s.x = 0.0f;
    calibration.imu.gyro_bias_rad_s.y = 0.0f;
    calibration.imu.gyro_bias_rad_s.z = 0.0f;

    calibration.imu.mag_bias_uT.x = 0.0f;
    calibration.imu.mag_bias_uT.y = 0.0f;
    calibration.imu.mag_bias_uT.z = 0.0f;

    calibration.imu.mag_scale.x = 1.0f;
    calibration.imu.mag_scale.y = 1.0f;
    calibration.imu.mag_scale.z = 1.0f;

    calibration.sun.pos_x_gain = 1.0f;
    calibration.sun.neg_x_gain = 1.0f;
    calibration.sun.pos_y_gain = 1.0f;
    calibration.sun.neg_y_gain = 1.0f;
    calibration.sun.pos_z_gain = 1.0f;
    calibration.sun.neg_z_gain = 1.0f;

    calibration.sun.pos_x_dark = 0.0f;
    calibration.sun.neg_x_dark = 0.0f;
    calibration.sun.pos_y_dark = 0.0f;
    calibration.sun.neg_y_dark = 0.0f;
    calibration.sun.pos_z_dark = 0.0f;
    calibration.sun.neg_z_dark = 0.0f;

    calibration.sun.min_valid_intensity = 0.05f;
}

const ADCS_Calibration* Calibration_Get(void)
{
    return &calibration;
}