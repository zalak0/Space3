#ifndef ADS_H743_SENSORS_H
#define ADS_H743_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "vector3.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t init_count;
    uint32_t update_count;
    uint32_t read_count;

    uint8_t initialized;
    uint8_t sensor_backend_ready;

    uint8_t icm_detected;
    uint8_t icm_address_7bit;
    uint8_t icm_whoami;
    uint8_t icm_configured;

    uint32_t raw_read_ok_count;
    uint32_t raw_read_fail_count;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    uint8_t mag_configured;
    uint32_t mag_read_ok_count;
    uint32_t mag_read_fail_count;

    int16_t mag_x_raw;
    int16_t mag_y_raw;
    int16_t mag_z_raw;

} ADS_H743_SensorDiagnostics;

typedef struct
{
    Vector3 accel_m_s2;
    Vector3 gyro_rad_s;
    Vector3 mag_T;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    int16_t mag_x_raw;
    int16_t mag_y_raw;
    int16_t mag_z_raw;

    uint8_t accel_valid;
    uint8_t gyro_valid;
    uint8_t mag_valid;

} ADS_H743_SensorSample;

void ADS_H743_Sensors_AttachI2C(I2C_HandleTypeDef *hi2c);

void ADS_H743_Sensors_Init(void);
void ADS_H743_Sensors_Update(float dt_s);

uint8_t ADS_H743_Sensors_IsInitialized(void);
uint8_t ADS_H743_Sensors_IsBackendReady(void);

bool ADS_H743_Sensors_ReadSample(ADS_H743_SensorSample *sample_out);

ADS_H743_SensorDiagnostics ADS_H743_Sensors_GetDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_H743_SENSORS_H */