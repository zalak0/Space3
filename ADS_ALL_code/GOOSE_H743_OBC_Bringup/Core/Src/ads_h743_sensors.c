#include "ads_h743_sensors.h"

#include "ads_icm20948_probe.h"
#include "calibration.h"

#include <math.h>
#include <string.h>

/*
 * ICM20948 accel/gyro conversion.
 *
 * Configuration in ads_icm20948_probe.c:
 * - gyro FS = +/- 500 deg/s
 * - accel FS = +/- 4 g
 *
 * Sensitivities:
 * - gyro: 65.5 LSB/(deg/s)
 * - accel: 8192 LSB/g
 */
#define ADS_H743_GYRO_LSB_PER_DPS       (65.5f)
#define ADS_H743_ACCEL_LSB_PER_G        (8192.0f)
#define ADS_H743_DEG_TO_RAD             (0.01745329251994329577f)
#define ADS_H743_GRAVITY_M_S2           (9.80665f)
#define ADS_H743_AK09916_UT_PER_LSB      (0.15f)
#define ADS_H743_MICROTESLA_TO_TESLA     (1.0e-6f)

static ADS_H743_SensorDiagnostics g_h743_sensor_diag;
static ADS_H743_SensorSample g_h743_last_sample;

static I2C_HandleTypeDef *g_h743_hi2c = 0;
static uint8_t g_h743_icm_address_7bit = 0u;

static Vector3 ADS_H743_ZeroVector(void)
{
    Vector3 v;
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;
    return v;
}

static void ADS_H743_ClearLastSample(void)
{
    memset(&g_h743_last_sample, 0, sizeof(g_h743_last_sample));

    g_h743_last_sample.accel_m_s2 = ADS_H743_ZeroVector();
    g_h743_last_sample.gyro_rad_s = ADS_H743_ZeroVector();

    g_h743_last_sample.accel_valid = 0u;
    g_h743_last_sample.gyro_valid = 0u;
}

static Vector3 ADS_H743_ConvertAccelRawToMps2(int16_t ax_raw,
                                              int16_t ay_raw,
                                              int16_t az_raw)
{
    Vector3 accel;

    accel.x = ((float)ax_raw / ADS_H743_ACCEL_LSB_PER_G) * ADS_H743_GRAVITY_M_S2;
    accel.y = ((float)ay_raw / ADS_H743_ACCEL_LSB_PER_G) * ADS_H743_GRAVITY_M_S2;
    accel.z = ((float)az_raw / ADS_H743_ACCEL_LSB_PER_G) * ADS_H743_GRAVITY_M_S2;

    return accel;
}

static Vector3 ADS_H743_ConvertGyroRawToRadS(int16_t gx_raw,
                                             int16_t gy_raw,
                                             int16_t gz_raw)
{
    const ADCS_Calibration *cal = Calibration_Get();

    Vector3 gyro;

    gyro.x = ((float)gx_raw / ADS_H743_GYRO_LSB_PER_DPS) * ADS_H743_DEG_TO_RAD;
    gyro.y = ((float)gy_raw / ADS_H743_GYRO_LSB_PER_DPS) * ADS_H743_DEG_TO_RAD;
    gyro.z = ((float)gz_raw / ADS_H743_GYRO_LSB_PER_DPS) * ADS_H743_DEG_TO_RAD;

    if (cal != 0)
    {
        gyro.x -= cal->imu.gyro_bias_rad_s.x;
        gyro.y -= cal->imu.gyro_bias_rad_s.y;
        gyro.z -= cal->imu.gyro_bias_rad_s.z;
    }

    return gyro;
}

static Vector3 ADS_H743_ConvertMagRawToTesla(int16_t mx_raw,
                                             int16_t my_raw,
                                             int16_t mz_raw)
{
    Vector3 mag;

    mag.x = ((float)mx_raw * ADS_H743_AK09916_UT_PER_LSB) *
            ADS_H743_MICROTESLA_TO_TESLA;

    mag.y = ((float)my_raw * ADS_H743_AK09916_UT_PER_LSB) *
            ADS_H743_MICROTESLA_TO_TESLA;

    mag.z = ((float)mz_raw * ADS_H743_AK09916_UT_PER_LSB) *
            ADS_H743_MICROTESLA_TO_TESLA;

    return mag;
}

void ADS_H743_Sensors_AttachI2C(I2C_HandleTypeDef *hi2c)
{
    g_h743_hi2c = hi2c;
}

void ADS_H743_Sensors_Init(void)
{
    ADS_ICM20948_ProbeDiagnostics probe_diag;
    ADS_ICM20948_WhoAmIDiagnostics whoami_diag;
    ADS_ICM20948_WakeDiagnostics wake_diag;
    ADS_ICM20948_ConfigDiagnostics config_diag;

    memset(&g_h743_sensor_diag, 0, sizeof(g_h743_sensor_diag));
    ADS_H743_ClearLastSample();

    g_h743_sensor_diag.init_count++;
    g_h743_sensor_diag.initialized = 1u;
    g_h743_sensor_diag.sensor_backend_ready = 0u;

    g_h743_last_sample.mag_T = ADS_H743_ZeroVector();
    g_h743_last_sample.mag_valid = 0u;

    g_h743_icm_address_7bit = 0u;

    if (g_h743_hi2c == 0)
    {
        /*
         * This is not a compile/build failure. It just means main.c has not
         * attached the H743 I2C handle yet.
         */
        return;
    }

    probe_diag = ADS_ICM20948_Probe_I2C(g_h743_hi2c);

    if ((probe_diag.result != ADS_ICM20948_PROBE_FOUND_0X69) &&
        (probe_diag.result != ADS_ICM20948_PROBE_FOUND_0X68))
    {
        return;
    }

    g_h743_icm_address_7bit = probe_diag.detected_address_7bit;

    g_h743_sensor_diag.icm_detected = 1u;
    g_h743_sensor_diag.icm_address_7bit = g_h743_icm_address_7bit;

    whoami_diag = ADS_ICM20948_ReadWhoAmI(
        g_h743_hi2c,
        g_h743_icm_address_7bit
    );

    g_h743_sensor_diag.icm_whoami = whoami_diag.who_am_i_value;

    if (whoami_diag.result != ADS_ICM20948_WHOAMI_OK)
    {
        return;
    }

    wake_diag = ADS_ICM20948_Wake(
        g_h743_hi2c,
        g_h743_icm_address_7bit
    );

    if (wake_diag.result != ADS_ICM20948_WAKE_OK)
    {
        return;
    }

    config_diag = ADS_ICM20948_ConfigureAccelGyro(
        g_h743_hi2c,
        g_h743_icm_address_7bit
    );

    if (config_diag.result != ADS_ICM20948_CONFIG_OK)
    {
        return;
    }

    g_h743_sensor_diag.icm_configured = 1u;
    g_h743_sensor_diag.sensor_backend_ready = 1u;

    ADS_ICM20948_MagDiagnostics mag_config_diag;

    mag_config_diag = ADS_ICM20948_ConfigureMag(
        g_h743_hi2c,
        g_h743_icm_address_7bit
    );

    if (mag_config_diag.result == ADS_ICM20948_MAG_OK)
    {
        g_h743_sensor_diag.mag_configured = 1u;
    }
    else
    {
        g_h743_sensor_diag.mag_configured = 0u;
    }
}

void ADS_H743_Sensors_Update(float dt_s)
{
    ADS_ICM20948_RawDiagnostics raw_diag;

    (void)dt_s;

    if (g_h743_sensor_diag.initialized == 0u)
    {
        return;
    }

    g_h743_sensor_diag.update_count++;

    if ((g_h743_hi2c == 0) ||
        (g_h743_icm_address_7bit == 0u) ||
        (g_h743_sensor_diag.sensor_backend_ready == 0u))
    {
        ADS_H743_ClearLastSample();
        return;
    }

    raw_diag = ADS_ICM20948_ReadAccelGyroRaw(
        g_h743_hi2c,
        g_h743_icm_address_7bit
    );

    if (raw_diag.result != ADS_ICM20948_RAW_OK)
    {
        g_h743_sensor_diag.raw_read_fail_count++;
        ADS_H743_ClearLastSample();
        return;
    }

    g_h743_sensor_diag.raw_read_ok_count++;

    g_h743_sensor_diag.accel_x_raw = raw_diag.accel_x_raw;
    g_h743_sensor_diag.accel_y_raw = raw_diag.accel_y_raw;
    g_h743_sensor_diag.accel_z_raw = raw_diag.accel_z_raw;

    g_h743_sensor_diag.gyro_x_raw = raw_diag.gyro_x_raw;
    g_h743_sensor_diag.gyro_y_raw = raw_diag.gyro_y_raw;
    g_h743_sensor_diag.gyro_z_raw = raw_diag.gyro_z_raw;

    g_h743_last_sample.accel_x_raw = raw_diag.accel_x_raw;
    g_h743_last_sample.accel_y_raw = raw_diag.accel_y_raw;
    g_h743_last_sample.accel_z_raw = raw_diag.accel_z_raw;

    g_h743_last_sample.gyro_x_raw = raw_diag.gyro_x_raw;
    g_h743_last_sample.gyro_y_raw = raw_diag.gyro_y_raw;
    g_h743_last_sample.gyro_z_raw = raw_diag.gyro_z_raw;

    g_h743_last_sample.accel_m_s2 = ADS_H743_ConvertAccelRawToMps2(
        raw_diag.accel_x_raw,
        raw_diag.accel_y_raw,
        raw_diag.accel_z_raw
    );

    g_h743_last_sample.gyro_rad_s = ADS_H743_ConvertGyroRawToRadS(
        raw_diag.gyro_x_raw,
        raw_diag.gyro_y_raw,
        raw_diag.gyro_z_raw
    );

    g_h743_last_sample.accel_valid = 1u;
    g_h743_last_sample.gyro_valid = 1u;


    if (g_h743_sensor_diag.mag_configured != 0u)
    {
        ADS_ICM20948_MagDiagnostics mag_diag;

        mag_diag = ADS_ICM20948_ReadMagRaw(
            g_h743_hi2c,
            g_h743_icm_address_7bit
        );

        if (mag_diag.result == ADS_ICM20948_MAG_OK)
        {
            g_h743_sensor_diag.mag_read_ok_count++;

            g_h743_sensor_diag.mag_x_raw = mag_diag.mag_x_raw;
            g_h743_sensor_diag.mag_y_raw = mag_diag.mag_y_raw;
            g_h743_sensor_diag.mag_z_raw = mag_diag.mag_z_raw;

            g_h743_last_sample.mag_x_raw = mag_diag.mag_x_raw;
            g_h743_last_sample.mag_y_raw = mag_diag.mag_y_raw;
            g_h743_last_sample.mag_z_raw = mag_diag.mag_z_raw;

            g_h743_last_sample.mag_T = ADS_H743_ConvertMagRawToTesla(
                mag_diag.mag_x_raw,
                mag_diag.mag_y_raw,
                mag_diag.mag_z_raw
            );

            g_h743_last_sample.mag_valid = 1u;
        }
        else
        {
            g_h743_sensor_diag.mag_read_fail_count++;
            g_h743_last_sample.mag_T = ADS_H743_ZeroVector();
            g_h743_last_sample.mag_valid = 0u;
        }
    }
    else
    {
        g_h743_last_sample.mag_T = ADS_H743_ZeroVector();
        g_h743_last_sample.mag_valid = 0u;
    }












}

uint8_t ADS_H743_Sensors_IsInitialized(void)
{
    return g_h743_sensor_diag.initialized;
}

uint8_t ADS_H743_Sensors_IsBackendReady(void)
{
    return g_h743_sensor_diag.sensor_backend_ready;
}

bool ADS_H743_Sensors_ReadSample(ADS_H743_SensorSample *sample_out)
{
    g_h743_sensor_diag.read_count++;

    if (sample_out == 0)
    {
        return false;
    }

    *sample_out = g_h743_last_sample;

    return (g_h743_last_sample.gyro_valid != 0u);
}

ADS_H743_SensorDiagnostics ADS_H743_Sensors_GetDiagnostics(void)
{
    return g_h743_sensor_diag;
}