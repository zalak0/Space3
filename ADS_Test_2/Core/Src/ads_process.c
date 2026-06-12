#include "ads_process.h"

#include <math.h>
#include <string.h>

#define ADS_PROCESS_EPSILON              (1.0e-6f)
#define ADS_PROCESS_MAG_MIN_T            (1.0e-7f)
#define ADS_PROCESS_ACCEL_MIN_M_S2       (1.0f)
#define ADS_PROCESS_ACCEL_MAX_M_S2       (25.0f)
#define ADS_PROCESS_RAD_TO_DEG           (57.29577951308232f)

static ADS_Process_Config s_ads_config;
static ADS_Process_Output s_ads_output;
static bool s_quest_seeded;

static bool ADS_Process_IsFiniteFloat(float x)
{
    return isfinite(x) ? true : false;
}

static bool ADS_Process_VectorIsFinite(Vector3 v)
{
    return ADS_Process_IsFiniteFloat(v.x) &&
           ADS_Process_IsFiniteFloat(v.y) &&
           ADS_Process_IsFiniteFloat(v.z);
}

static bool ADS_Process_VectorIsUsable(Vector3 v, float min_norm, float max_norm)
{
    if (!ADS_Process_VectorIsFinite(v))
    {
        return false;
    }

    const float n = Vector3_Norm(v);

    if (n < min_norm)
    {
        return false;
    }

    if ((max_norm > min_norm) && (n > max_norm))
    {
        return false;
    }

    return true;
}

static Vector3 ADS_Process_MagMicroTeslaToTesla(const ICM20948_Data* imu)
{
    Vector3 mag_T;

    mag_T.x = imu->mag_x * 1.0e-6f;
    mag_T.y = imu->mag_y * 1.0e-6f;
    mag_T.z = imu->mag_z * 1.0e-6f;

    return mag_T;
}

static bool ADS_Process_TryQuestSeed(Vector3 accel_body_m_s2, Vector3 mag_body_T)
{
    QUEST_Input quest_input;
    QUEST_InitInput(&quest_input);

    const unsigned char accel_added = QUEST_AddVectorPair(
        &quest_input,
        accel_body_m_s2,
        s_ads_config.gravity_reference_m_s2,
        1.0f
    );

    const unsigned char mag_added = QUEST_AddVectorPair(
        &quest_input,
        mag_body_T,
        s_ads_config.magnetic_reference_T,
        0.5f
    );

    if ((accel_added == 0U) || (mag_added == 0U))
    {
        return false;
    }

    const QUEST_Output quest_output = QUEST_Solve(&quest_input);

    s_ads_output.quest_q_body_to_inertial = quest_output.attitude_q;
    s_ads_output.quest_residual = quest_output.residual;
    s_ads_output.quest_valid = (quest_output.valid != 0U);

    if (quest_output.valid == 0U)
    {
        return false;
    }

    ADS_EKF_ResetWithAttitude(quest_output.attitude_q);
    s_quest_seeded = true;

    return true;
}

ADS_Process_Config ADS_Process_DefaultConfig(void)
{
    ADS_Process_Config config;

    /*
     * Bench-test defaults:
     * - gravity_reference_m_s2 is the accelerometer-observed 1 g direction
     *   when the estimator inertial/test frame is aligned with the body frame.
     * - magnetic_reference_T is only a placeholder. Replace it with an IGRF/orbit
     *   magnetic reference before using the estimate as flight truth.
     */
    config.gravity_reference_m_s2.x = 0.0f;
    config.gravity_reference_m_s2.y = 0.0f;
    config.gravity_reference_m_s2.z = 9.80665f;

    config.magnetic_reference_T.x = 25.0e-6f;
    config.magnetic_reference_T.y = 0.0f;
    config.magnetic_reference_T.z = -43.0e-6f;

    config.sample_period_s = 0.10f;

    return config;
}

void ADS_Process_Init(const ADS_Process_Config* config)
{
    memset(&s_ads_output, 0, sizeof(s_ads_output));

    if (config != 0)
    {
        s_ads_config = *config;
    }
    else
    {
        s_ads_config = ADS_Process_DefaultConfig();
    }

    if (!ADS_Process_VectorIsUsable(s_ads_config.gravity_reference_m_s2,
                                    ADS_PROCESS_EPSILON,
                                    0.0f))
    {
        s_ads_config.gravity_reference_m_s2.x = 0.0f;
        s_ads_config.gravity_reference_m_s2.y = 0.0f;
        s_ads_config.gravity_reference_m_s2.z = 9.80665f;
    }

    if (!ADS_Process_VectorIsUsable(s_ads_config.magnetic_reference_T,
                                    ADS_PROCESS_MAG_MIN_T,
                                    0.0f))
    {
        s_ads_config.magnetic_reference_T.x = 25.0e-6f;
        s_ads_config.magnetic_reference_T.y = 0.0f;
        s_ads_config.magnetic_reference_T.z = -43.0e-6f;
    }

    if ((!ADS_Process_IsFiniteFloat(s_ads_config.sample_period_s)) ||
        (s_ads_config.sample_period_s <= 0.0f))
    {
        s_ads_config.sample_period_s = 0.10f;
    }

    ADS_EKF_Init();
    s_quest_seeded = false;

    s_ads_output.initialized = true;
    s_ads_output.quest_q_body_to_inertial = Quaternion_Identity();
    s_ads_output.ekf_state = ADS_EKF_GetState();
    s_ads_output.euler_rad = Quaternion_ToEuler321(
        s_ads_output.ekf_state.q_body_to_inertial
    );
}

bool ADS_Process_Update(const ICM20948_Data* imu, ADS_Process_Output* out)
{
    if (imu == 0)
    {
        return false;
    }

    if (!s_ads_output.initialized)
    {
        ADS_Process_Init(0);
    }

    s_ads_output.sample_count++;
    s_ads_output.imu_valid = imu->valid;
    s_ads_output.accel_used = false;
    s_ads_output.mag_used = false;
    s_ads_output.ekf_valid = false;

    if (!imu->valid)
    {
        if (out != 0)
        {
            *out = s_ads_output;
        }

        return false;
    }

    Vector3 gyro_rad_s;
    gyro_rad_s.x = imu->gyro_x;
    gyro_rad_s.y = imu->gyro_y;
    gyro_rad_s.z = imu->gyro_z;

    Vector3 accel_body_m_s2;
    accel_body_m_s2.x = imu->accel_x;
    accel_body_m_s2.y = imu->accel_y;
    accel_body_m_s2.z = imu->accel_z;

    const Vector3 mag_body_T = ADS_Process_MagMicroTeslaToTesla(imu);

    const bool gyro_valid = ADS_Process_VectorIsFinite(gyro_rad_s);
    const bool accel_valid = ADS_Process_VectorIsUsable(accel_body_m_s2,
                                                        ADS_PROCESS_ACCEL_MIN_M_S2,
                                                        ADS_PROCESS_ACCEL_MAX_M_S2);
    const bool mag_valid = ADS_Process_VectorIsUsable(mag_body_T,
                                                      ADS_PROCESS_MAG_MIN_T,
                                                      0.0f);

    s_ads_output.accel_used = accel_valid;
    s_ads_output.mag_used = mag_valid;

    if ((!s_quest_seeded) && accel_valid && mag_valid)
    {
        (void)ADS_Process_TryQuestSeed(accel_body_m_s2, mag_body_T);
    }

    ADS_EKF_Input ekf_input;
    memset(&ekf_input, 0, sizeof(ekf_input));

    ekf_input.gyro_rad_s = gyro_rad_s;
    ekf_input.accel_body_m_s2 = accel_body_m_s2;
    ekf_input.gravity_inertial_m_s2 = s_ads_config.gravity_reference_m_s2;
    ekf_input.mag_body_T = mag_body_T;
    ekf_input.mag_inertial_T = s_ads_config.magnetic_reference_T;
    ekf_input.gyro_valid = gyro_valid;
    ekf_input.accel_valid = accel_valid;
    ekf_input.mag_valid = mag_valid;
    ekf_input.dt_s = s_ads_config.sample_period_s;

    const bool ekf_ok = ADS_EKF_PredictUpdate(&ekf_input);

    s_ads_output.ekf_state = ADS_EKF_GetState();
    s_ads_output.ekf_valid = ekf_ok && s_ads_output.ekf_state.healthy;

    s_ads_output.euler_rad = Quaternion_ToEuler321(
        s_ads_output.ekf_state.q_body_to_inertial
    );

    s_ads_output.roll_deg = s_ads_output.euler_rad.roll_rad * ADS_PROCESS_RAD_TO_DEG;
    s_ads_output.pitch_deg = s_ads_output.euler_rad.pitch_rad * ADS_PROCESS_RAD_TO_DEG;
    s_ads_output.yaw_deg = s_ads_output.euler_rad.yaw_rad * ADS_PROCESS_RAD_TO_DEG;

    if (out != 0)
    {
        *out = s_ads_output;
    }

    return s_ads_output.ekf_valid;
}

ADS_Process_Output ADS_Process_GetOutput(void)
{
    return s_ads_output;
}
