#ifndef ADS_ICM20948_PROBE_H
#define ADS_ICM20948_PROBE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef enum
{
    ADS_ICM20948_PROBE_NOT_RUN = 0,
    ADS_ICM20948_PROBE_FOUND_0X69 = 1,
    ADS_ICM20948_PROBE_FOUND_0X68 = 2,
    ADS_ICM20948_PROBE_NOT_FOUND = 3,
    ADS_ICM20948_PROBE_HAL_ERROR = 4
} ADS_ICM20948_ProbeResult;

typedef enum
{
    ADS_ICM20948_WHOAMI_NOT_RUN = 0,
    ADS_ICM20948_WHOAMI_OK = 1,
    ADS_ICM20948_WHOAMI_BAD_VALUE = 2,
    ADS_ICM20948_WHOAMI_HAL_ERROR = 3
} ADS_ICM20948_WhoAmIResult;

typedef enum
{
    ADS_ICM20948_WAKE_NOT_RUN = 0,
    ADS_ICM20948_WAKE_OK = 1,
    ADS_ICM20948_WAKE_HAL_ERROR = 2
} ADS_ICM20948_WakeResult;

typedef enum
{
    ADS_ICM20948_CONFIG_NOT_RUN = 0,
    ADS_ICM20948_CONFIG_OK = 1,
    ADS_ICM20948_CONFIG_HAL_ERROR = 2
} ADS_ICM20948_ConfigResult;


typedef enum
{
    ADS_ICM20948_RAW_NOT_RUN = 0,
    ADS_ICM20948_RAW_OK = 1,
    ADS_ICM20948_RAW_HAL_ERROR = 2,
    ADS_ICM20948_RAW_BAD_ARG = 3
} ADS_ICM20948_RawReadResult;

typedef struct
{
    ADS_ICM20948_ProbeResult result;
    uint8_t detected_address_7bit;
    HAL_StatusTypeDef hal_status_0x69;
    HAL_StatusTypeDef hal_status_0x68;
    uint32_t probe_count;
} ADS_ICM20948_ProbeDiagnostics;

typedef struct
{
    ADS_ICM20948_WhoAmIResult result;
    uint8_t address_7bit;
    uint8_t who_am_i_value;
    HAL_StatusTypeDef hal_status;
    uint32_t read_count;
} ADS_ICM20948_WhoAmIDiagnostics;

typedef struct
{
    ADS_ICM20948_WakeResult result;
    uint8_t address_7bit;
    HAL_StatusTypeDef hal_status;
    uint32_t wake_count;
} ADS_ICM20948_WakeDiagnostics;

typedef struct
{
    ADS_ICM20948_ConfigResult result;
    uint8_t address_7bit;
    HAL_StatusTypeDef hal_status;
    uint32_t config_count;
} ADS_ICM20948_ConfigDiagnostics;


typedef struct
{
    ADS_ICM20948_RawReadResult result;
    uint8_t address_7bit;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    HAL_StatusTypeDef hal_status;
    uint32_t read_count;
} ADS_ICM20948_RawDiagnostics;

ADS_ICM20948_ProbeDiagnostics ADS_ICM20948_Probe_I2C(I2C_HandleTypeDef *hi2c);

ADS_ICM20948_WhoAmIDiagnostics ADS_ICM20948_ReadWhoAmI(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
);

ADS_ICM20948_WakeDiagnostics ADS_ICM20948_Wake(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
);

ADS_ICM20948_RawDiagnostics ADS_ICM20948_ReadAccelGyroRaw(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
);

ADS_ICM20948_ConfigDiagnostics ADS_ICM20948_ConfigureAccelGyro(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
);

#ifdef __cplusplus
}
#endif

#endif /* ADS_ICM20948_PROBE_H */