#include "adcs.h"

#include "quest.h"
#include "ads_ekf.h"

#include <math.h>
#include <stdbool.h>

#define ADCS_MICROTESLA_TO_TESLA (1.0e-6f)
#define ADCS_TESLA_TO_MICROTESLA (1.0e6f)
#define ADCS_STANDARD_GRAVITY_M_S2 (9.80665f)
#define ADCS_EULER_RATE_COS_THETA_MIN (1.0e-3f)

static ADCS_State adcs_state;
static bool s_adcs_quest_alignment_done = false;

static Vector3 ADCS_VectorZero(void)
{
    Vector3 v;
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;
    return v;
}

static Vector3 ADCS_VectorScaleLocal(Vector3 v, float scale)
{
    Vector3 out;
    out.x = v.x * scale;
    out.y = v.y * scale;
    out.z = v.z * scale;
    return out;
}

static bool ADCS_VectorLooksUsable(Vector3 v)
{
    return isfinite(v.x) &&
           isfinite(v.y) &&
           isfinite(v.z) &&
           Vector3_Norm(v) > 1.0e-6f;
}

static Vector3 ADCS_BodyFromInertial(Quaternion q_body_to_inertial,
                                     Vector3 inertial_vector)
{
    Quaternion q_conj = Quaternion_Conjugate(
        Quaternion_Normalize(q_body_to_inertial)
    );

    return Quaternion_RotateVector(q_conj, inertial_vector);
}

static void ADCS_UpdatePredictedAccelVector(void)
{
    if (!ADCS_VectorLooksUsable(adcs_state.gravity_inertial_m_s2))
    {
        adcs_state.accel_body_predicted_m_s2 = ADCS_VectorZero();
        return;
    }

    adcs_state.accel_body_predicted_m_s2 = ADCS_BodyFromInertial(
        adcs_state.attitude_q,
        adcs_state.gravity_inertial_m_s2
    );
}

static void ADCS_UpdatePredictedSunVector(void)
{
    if (!ADCS_VectorLooksUsable(adcs_state.sun_inertial))
    {
        adcs_state.sun_body_predicted = ADCS_VectorZero();
        return;
    }

    adcs_state.sun_body_predicted = ADCS_BodyFromInertial(
        adcs_state.attitude_q,
        adcs_state.sun_inertial
    );

    adcs_state.sun_body_predicted = Vector3_Normalize(
        adcs_state.sun_body_predicted
    );
}

static void ADCS_UpdatePredictedMagVector(void)
{
    if (!ADCS_VectorLooksUsable(adcs_state.mag_inertial_uT))
    {
        adcs_state.mag_body_predicted_uT = ADCS_VectorZero();
        return;
    }

    adcs_state.mag_body_predicted_uT = ADCS_BodyFromInertial(
        adcs_state.attitude_q,
        adcs_state.mag_inertial_uT
    );
}

static void ADCS_UpdateAccelResidual(void)
{
    if ((adcs_state.sensor_status.accel_valid == 0u) ||
        !ADCS_VectorLooksUsable(adcs_state.accel_body_measured_m_s2) ||
        !ADCS_VectorLooksUsable(adcs_state.accel_body_predicted_m_s2))
    {
        adcs_state.accel_error = ADCS_VectorZero();
        return;
    }

    Vector3 measured = Vector3_Normalize(adcs_state.accel_body_measured_m_s2);
    Vector3 predicted = Vector3_Normalize(adcs_state.accel_body_predicted_m_s2);

    adcs_state.accel_error = Vector3_Cross(predicted, measured);
}

static void ADCS_UpdateSunResidual(void)
{
    if ((adcs_state.sensor_status.sun_valid == 0u) ||
        !ADCS_VectorLooksUsable(adcs_state.sun_body_measured) ||
        !ADCS_VectorLooksUsable(adcs_state.sun_body_predicted))
    {
        adcs_state.sun_error = ADCS_VectorZero();
        return;
    }

    Vector3 measured = Vector3_Normalize(adcs_state.sun_body_measured);
    Vector3 predicted = Vector3_Normalize(adcs_state.sun_body_predicted);

    adcs_state.sun_error = Vector3_Cross(predicted, measured);
}

static void ADCS_UpdateMagResidual(void)
{
    if ((adcs_state.sensor_status.mag_valid == 0u) ||
        !ADCS_VectorLooksUsable(adcs_state.mag_body_measured_uT) ||
        !ADCS_VectorLooksUsable(adcs_state.mag_body_predicted_uT))
    {
        adcs_state.mag_error = ADCS_VectorZero();
        return;
    }

    Vector3 measured = Vector3_Normalize(adcs_state.mag_body_measured_uT);
    Vector3 predicted = Vector3_Normalize(adcs_state.mag_body_predicted_uT);

    adcs_state.mag_error = Vector3_Cross(predicted, measured);
}

static Vector3 ADCS_ComputeEulerRates321(EulerAngles euler_rad,
                                            Vector3 body_rates_rad_s)
{
    Vector3 rates;

    const float phi = euler_rad.roll_rad;
    const float theta = euler_rad.pitch_rad;

    const float sin_phi = sinf(phi);
    const float cos_phi = cosf(phi);
    const float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    if (fabsf(cos_theta) < ADCS_EULER_RATE_COS_THETA_MIN)
    {
        cos_theta = (cos_theta >= 0.0f) ?
            ADCS_EULER_RATE_COS_THETA_MIN :
            -ADCS_EULER_RATE_COS_THETA_MIN;
    }

    const float tan_theta = sin_theta / cos_theta;

    const float p = body_rates_rad_s.x;
    const float q = body_rates_rad_s.y;
    const float r = body_rates_rad_s.z;

    rates.x = p + (sin_phi * tan_theta * q) +
                  (cos_phi * tan_theta * r);

    rates.y = (cos_phi * q) - (sin_phi * r);

    rates.z = ((sin_phi / cos_theta) * q) +
              ((cos_phi / cos_theta) * r);

    if (!isfinite(rates.x) || !isfinite(rates.y) || !isfinite(rates.z))
    {
        rates = ADCS_VectorZero();
    }

    return rates;
}

static void ADCS_UpdateEulerRates(void)
{
    if (adcs_state.sensor_status.gyro_valid == 0u)
    {
        adcs_state.euler_rates_rad_s = ADCS_VectorZero();
        return;
    }

    const EulerAngles euler_rad = Quaternion_ToEuler321(adcs_state.attitude_q);
    const Vector3 body_rates_corrected = Vector3_Sub(
        adcs_state.gyro_rad_s,
        adcs_state.gyro_bias_rad_s
    );

    adcs_state.euler_rates_rad_s = ADCS_ComputeEulerRates321(
        euler_rad,
        body_rates_corrected
    );
}

static void ADCS_AttemptQuestInitialAlignment(void)
{
    /*
     * QUEST needs at least two non-collinear vector pairs.
     *
     * No-sun ADS path:
     *   1) accelerometer gravity direction
     *   2) magnetometer field direction
     *
     * This is valid for bench testing when accel magnitude is close to 1 g.
     */
    if (s_adcs_quest_alignment_done)
    {
        return;
    }

    if ((adcs_state.sensor_status.accel_valid == 0u) ||
        (adcs_state.sensor_status.mag_valid == 0u))
    {
        return;
    }

    if (!ADCS_VectorLooksUsable(adcs_state.accel_body_measured_m_s2) ||
        !ADCS_VectorLooksUsable(adcs_state.gravity_inertial_m_s2) ||
        !ADCS_VectorLooksUsable(adcs_state.mag_body_measured_uT) ||
        !ADCS_VectorLooksUsable(adcs_state.mag_inertial_uT))
    {
        return;
    }

    QUEST_Input quest_input;
    QUEST_InitInput(&quest_input);

    (void)QUEST_AddVectorPair(
        &quest_input,
        adcs_state.accel_body_measured_m_s2,
        adcs_state.gravity_inertial_m_s2,
        1.0f
    );

    (void)QUEST_AddVectorPair(
        &quest_input,
        adcs_state.mag_body_measured_uT,
        adcs_state.mag_inertial_uT,
        0.5f
    );

    QUEST_Output quest_output = QUEST_Solve(&quest_input);

    if (quest_output.valid != 0u)
    {
        adcs_state.attitude_q = Quaternion_Normalize(quest_output.attitude_q);
        s_adcs_quest_alignment_done = true;
    }
}

void ADCS_Init(void)
{
    adcs_state.attitude_q = Quaternion_Identity();

    adcs_state.gyro_rad_s = ADCS_VectorZero();
    adcs_state.gyro_bias_rad_s = ADCS_VectorZero();
    adcs_state.euler_rates_rad_s = ADCS_VectorZero();

    /*
     * Bench-test gravity reference. If the board reports the opposite sign
     * when level, flip this z sign during axis calibration.
     */
    adcs_state.gravity_inertial_m_s2.x = 0.0f;
    adcs_state.gravity_inertial_m_s2.y = 0.0f;
    adcs_state.gravity_inertial_m_s2.z = ADCS_STANDARD_GRAVITY_M_S2;

    adcs_state.sun_inertial.x = 1.0f;
    adcs_state.sun_inertial.y = 0.0f;
    adcs_state.sun_inertial.z = 0.0f;

    /*
     * Bring-up magnetic reference, units are microtesla inside ADCS_State.
     * This can later be replaced by proper orbit/IGRF reference generation.
     */
    adcs_state.mag_inertial_uT.x = 20.0f;
    adcs_state.mag_inertial_uT.y = 0.0f;
    adcs_state.mag_inertial_uT.z = -40.0f;

    adcs_state.accel_body_measured_m_s2 = ADCS_VectorZero();
    adcs_state.accel_body_predicted_m_s2 = ADCS_VectorZero();
    adcs_state.accel_error = ADCS_VectorZero();

    adcs_state.sun_body_measured = ADCS_VectorZero();
    adcs_state.sun_body_predicted = ADCS_VectorZero();
    adcs_state.sun_error = ADCS_VectorZero();

    adcs_state.mag_body_measured_uT = ADCS_VectorZero();
    adcs_state.mag_body_predicted_uT = ADCS_VectorZero();
    adcs_state.mag_error = ADCS_VectorZero();

    adcs_state.sensor_status.gyro_valid = 0u;
    adcs_state.sensor_status.accel_valid = 0u;
    adcs_state.sensor_status.sun_valid = 0u;
    adcs_state.sensor_status.mag_valid = 0u;

    adcs_state.dt = 0.01f;

    /*
     * These gains are retained for telemetry/backwards compatibility.
     * The actual correction now happens inside ADS_EKF_PredictUpdate().
     */
    adcs_state.sun_correction_gain = 0.0f;
    adcs_state.mag_correction_gain = 0.0f;

    adcs_state.update_counter = 0u;

    s_adcs_quest_alignment_done = false;

    ADS_EKF_Init();

    ADCS_UpdatePredictedAccelVector();
    ADCS_UpdatePredictedSunVector();
    ADCS_UpdatePredictedMagVector();
    ADCS_UpdateAccelResidual();
    ADCS_UpdateSunResidual();
    ADCS_UpdateMagResidual();
    ADCS_UpdateEulerRates();
}

void ADCS_Update(void)
{
    adcs_state.update_counter++;

    ADCS_AttemptQuestInitialAlignment();

    ADS_EKF_Input ekf_input;

    ekf_input.gyro_rad_s = adcs_state.gyro_rad_s;

    ekf_input.accel_body_m_s2 = adcs_state.accel_body_measured_m_s2;
    ekf_input.gravity_inertial_m_s2 = adcs_state.gravity_inertial_m_s2;

    ekf_input.mag_body_T = ADCS_VectorScaleLocal(
        adcs_state.mag_body_measured_uT,
        ADCS_MICROTESLA_TO_TESLA
    );

    ekf_input.mag_inertial_T = ADCS_VectorScaleLocal(
        adcs_state.mag_inertial_uT,
        ADCS_MICROTESLA_TO_TESLA
    );

    ekf_input.gyro_valid = adcs_state.sensor_status.gyro_valid != 0u;
    ekf_input.accel_valid = adcs_state.sensor_status.accel_valid != 0u;
    ekf_input.mag_valid = adcs_state.sensor_status.mag_valid != 0u;

    ekf_input.dt_s = adcs_state.dt;

    if (ADS_EKF_PredictUpdate(&ekf_input))
    {
        ADS_EKF_State ekf_state = ADS_EKF_GetState();

        adcs_state.attitude_q = Quaternion_Normalize(
            ekf_state.q_body_to_inertial
        );

        adcs_state.gyro_bias_rad_s = ekf_state.gyro_bias_rad_s;
    }
    else if (adcs_state.sensor_status.gyro_valid != 0u)
    {
        /*
         * Last-resort fallback: gyro-only propagation if EKF rejects input.
         */
        Vector3 omega_corrected = Vector3_Sub(
            adcs_state.gyro_rad_s,
            adcs_state.gyro_bias_rad_s
        );

        adcs_state.attitude_q = Quaternion_PropagateGyro(
            adcs_state.attitude_q,
            omega_corrected,
            adcs_state.dt
        );
    }

    ADCS_UpdatePredictedAccelVector();
    ADCS_UpdatePredictedSunVector();
    ADCS_UpdatePredictedMagVector();

    ADCS_UpdateAccelResidual();
    ADCS_UpdateSunResidual();
    ADCS_UpdateMagResidual();
    ADCS_UpdateEulerRates();
}

void ADCS_SetSensorInputs(
    Vector3 gyro_rad_s,
    Vector3 accel_body_measured_m_s2,
    Vector3 sun_body_measured,
    Vector3 mag_body_measured_uT,
    ADCS_SensorStatus status
)
{
    adcs_state.sensor_status = status;

    if (status.gyro_valid != 0u)
    {
        adcs_state.gyro_rad_s = gyro_rad_s;
    }
    else
    {
        adcs_state.gyro_rad_s = ADCS_VectorZero();
    }

    if ((status.accel_valid != 0u) &&
        ADCS_VectorLooksUsable(accel_body_measured_m_s2))
    {
        adcs_state.accel_body_measured_m_s2 = accel_body_measured_m_s2;
    }
    else
    {
        adcs_state.accel_body_measured_m_s2 = ADCS_VectorZero();
        adcs_state.sensor_status.accel_valid = 0u;
    }

    /* Sun vector intentionally unused in this no-sun build. */
    (void)sun_body_measured;
    adcs_state.sun_body_measured = ADCS_VectorZero();
    adcs_state.sensor_status.sun_valid = 0u;

    if ((status.mag_valid != 0u) && ADCS_VectorLooksUsable(mag_body_measured_uT))
    {
        adcs_state.mag_body_measured_uT = mag_body_measured_uT;
    }
    else
    {
        adcs_state.mag_body_measured_uT = ADCS_VectorZero();
        adcs_state.sensor_status.mag_valid = 0u;
    }
}

void ADCS_SetInertialReferences(
    Vector3 gravity_inertial_m_s2,
    Vector3 mag_inertial_uT
)
{
    if (ADCS_VectorLooksUsable(gravity_inertial_m_s2))
    {
        adcs_state.gravity_inertial_m_s2 = gravity_inertial_m_s2;
    }

    if (ADCS_VectorLooksUsable(mag_inertial_uT))
    {
        adcs_state.mag_inertial_uT = mag_inertial_uT;
    }
}

void ADCS_SetDt(float dt)
{
    if ((dt > 0.0f) && (dt < 1.0f) && isfinite(dt))
    {
        adcs_state.dt = dt;
    }
}

const ADCS_State* ADCS_GetState(void)
{
    return &adcs_state;
}

ADCS_Telemetry ADCS_GetTelemetry(void)
{
    ADCS_Telemetry telemetry;

    telemetry.attitude_q = adcs_state.attitude_q;
    telemetry.euler_rad = Quaternion_ToEuler321(adcs_state.attitude_q);
    telemetry.euler_rates_rad_s = adcs_state.euler_rates_rad_s;

    telemetry.gyro_rad_s = adcs_state.gyro_rad_s;

    telemetry.accel_body_measured_m_s2 = adcs_state.accel_body_measured_m_s2;
    telemetry.accel_body_predicted_m_s2 = adcs_state.accel_body_predicted_m_s2;
    telemetry.accel_error = adcs_state.accel_error;

    telemetry.sun_body_measured = adcs_state.sun_body_measured;
    telemetry.sun_body_predicted = adcs_state.sun_body_predicted;
    telemetry.sun_error = adcs_state.sun_error;

    telemetry.mag_body_measured_uT = adcs_state.mag_body_measured_uT;
    telemetry.mag_body_predicted_uT = adcs_state.mag_body_predicted_uT;
    telemetry.mag_error = adcs_state.mag_error;

    telemetry.sensor_status = adcs_state.sensor_status;

    telemetry.dt = adcs_state.dt;
    telemetry.update_counter = adcs_state.update_counter;

    return telemetry;
}