#include "ads_fake_sensors.h"
#include "ads_config.h"

#include <math.h>
#include <string.h>

/*
 * F3Discovery fake ADS sensor source.
 *
 * Units:
 * - gyro: rad/s
 * - magnetometer: Tesla
 * - sun vector: unit vector in body frame
 *
 * This file does not touch real hardware.
 */

static Vector3 s_gyro_rad_s;
static Vector3 s_sun_body;
static Vector3 s_mag_body_T;
static ADCS_SensorStatus s_fake_status;

static uint32_t s_update_count = 0u;
static float s_sim_time_s = 0.0f;

static Vector3 make_vector3(float x, float y, float z)
{
    Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

static void ADS_FakeSensors_SetAllValid(void)
{
    s_fake_status.gyro_valid = 1u;
    s_fake_status.sun_valid = 1u;
    s_fake_status.mag_valid = 1u;
}

void ADS_FakeSensors_Init(void)
{
    memset(&s_fake_status, 0, sizeof(s_fake_status));

    s_update_count = 0u;
    s_sim_time_s = 0.0f;

    s_gyro_rad_s = make_vector3(0.0f, 0.0f, ADS_FAKE_ROTATION_RATE_RAD_S);

    s_sun_body = make_vector3(1.0f, 0.0f, 0.0f);

    /*
     * Rough Earth magnetic field magnitude order:
     * 25 to 65 microtesla.
     *
     * ADCS_SetSensorInputs() receives this as Tesla.
     * Your telemetry later stores measured magnetic field as microtesla.
     */
    s_mag_body_T = make_vector3(
        ADS_FAKE_MAG_FIELD_X_T,
        ADS_FAKE_MAG_FIELD_Y_T,
        ADS_FAKE_MAG_FIELD_Z_T
    );

    ADS_FakeSensors_SetAllValid();
}

void ADS_FakeSensors_Update(float dt_s)
{
    if (dt_s < 0.0f) {
        dt_s = 0.0f;
    }

    s_sim_time_s += dt_s;
    s_update_count++;

    /*
     * Very slow fake rotation in the body-frame measurements.
     * This gives the estimator changing but gentle inputs.
     */
    const float omega_fake = ADS_FAKE_ROTATION_RATE_RAD_S;
    const float angle = omega_fake * s_sim_time_s;

    const float c = cosf(angle);
    const float s = sinf(angle);

    s_gyro_rad_s = make_vector3(0.0f, 0.0f, omega_fake);

    s_sun_body = make_vector3(c, s, 0.0f);

    s_mag_body_T = make_vector3(
        ADS_FAKE_MAG_FIELD_X_T * c,
        ADS_FAKE_MAG_FIELD_X_T * s,
        ADS_FAKE_MAG_FIELD_Z_T
    );

    ADS_FakeSensors_SetAllValid();
}

Vector3 ADS_FakeSensors_GetGyroRadS(void)
{
    return s_gyro_rad_s;
}

Vector3 ADS_FakeSensors_GetSunBody(void)
{
    return s_sun_body;
}

Vector3 ADS_FakeSensors_GetMagBody(void)
{
    return s_mag_body_T;
}

ADCS_SensorStatus ADS_FakeSensors_GetStatus(void)
{
    return s_fake_status;
}

uint32_t ADS_FakeSensors_GetUpdateCount(void)
{
    return s_update_count;
}

float ADS_FakeSensors_GetSimTime(void)
{
    return s_sim_time_s;
}