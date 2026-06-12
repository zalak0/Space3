#ifndef ICM20948_H
#define ICM20948_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C addresses — AD0/SDO low = 0x68, high = 0x69 */
#define ICM20948_ADDR_LOW   (0x68 << 1)   /* HAL uses 8-bit address */
#define ICM20948_ADDR_HIGH  (0x69 << 1)

/* ── User Bank 0 registers ─────────────────────────────────────── */
#define ICM_WHO_AM_I        0x00
#define ICM_WHO_AM_I_VAL    0xEA

#define ICM_USER_CTRL       0x03
#define ICM_LP_CONFIG       0x05
#define ICM_PWR_MGMT_1      0x06
#define ICM_PWR_MGMT_2      0x07

#define ICM_ACCEL_XOUT_H    0x2D
#define ICM_GYRO_XOUT_H     0x33
#define ICM_TEMP_OUT_H      0x39

/* ── User Bank 2 registers ─────────────────────────────────────── */
#define ICM_GYRO_CONFIG_1   0x01
#define ICM_ACCEL_CONFIG    0x14

/* ── Magnetometer (AK09916) registers ─────────────────────────── */
#define AK09916_ADDR        (0x0C << 1)
#define AK09916_WIA2        0x01   /* Who Am I — expected 0x09   */
#define AK09916_ST1         0x10   /* Status 1                   */
#define AK09916_HXL         0x11   /* Mag X low byte             */
#define AK09916_CNTL2       0x31   /* Control 2 (mode)           */
#define AK09916_CNTL3       0x32   /* Control 3 (reset)          */

#define AK09916_CONT_MODE_100HZ  0x08

/* ── Scaled output structure ───────────────────────────────────── */
typedef struct {
    /* Accelerometer — m/s² */
    float accel_x, accel_y, accel_z;

    /* Gyroscope — rad/s */
    float gyro_x, gyro_y, gyro_z;

    /* Magnetometer — µT */
    float mag_x, mag_y, mag_z;

    /* Temperature — °C */
    float temp_c;

    /* Status */
    bool  valid;
} ICM20948_Data;

/* ── Public API ────────────────────────────────────────────────── */
bool ICM20948_Init(I2C_HandleTypeDef *hi2c);
bool ICM20948_Read(I2C_HandleTypeDef *hi2c, ICM20948_Data *out);

#endif /* ICM20948_H */
