#include "ads_ekf.h"

#include <math.h>
#include <string.h>

/*
 * ads_ekf.c
 *
 * Embedded multiplicative EKF-style attitude estimator.
 *
 * State:
 *   nominal quaternion q_body_to_inertial
 *   gyro bias b_g
 *
 * Error-state covariance:
 *   x_err = [dtheta_x dtheta_y dtheta_z dbx dby dbz]^T
 *   P is stored internally as 6x6.
 *
 * Current measurement updates:
 *   accelerometer gravity-direction vector compared against inertial gravity
 *   magnetometer body vector compared against inertial magnetic reference.
 *
 * Notes:
 * - No dynamic allocation.
 * - Float-only.
 * - Safe gyro-only fallback if mag update is unavailable.
 * - Photodiodes/sun sensors intentionally not used.
 */

#define ADS_EKF_STATE_DIM               (6U)

#define ADS_EKF_EPSILON                 (1.0e-6f)
#define ADS_EKF_DT_MIN_S                (1.0e-5f)
#define ADS_EKF_DT_MAX_S                (1.0f)

#define ADS_EKF_INITIAL_ATT_VAR         (0.25f)
#define ADS_EKF_INITIAL_BIAS_VAR        (1.0e-4f)

#define ADS_EKF_GYRO_NOISE_VAR          (2.5e-5f)
#define ADS_EKF_BIAS_RANDOM_WALK_VAR    (1.0e-8f)

#define ADS_EKF_ACCEL_MEAS_VAR          (1.0e-2f)
#define ADS_EKF_MAG_MEAS_VAR            (2.5e-3f)

#define ADS_EKF_MAX_CORRECTION_RAD      (0.25f)
#define ADS_EKF_MAX_BIAS_RAD_S          (0.20f)

static ADS_EKF_State g_ads_ekf_state;
static float g_ads_ekf_P[ADS_EKF_STATE_DIM][ADS_EKF_STATE_DIM];

static float ADS_EKF_Abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float ADS_EKF_Clamp(float x, float min_value, float max_value)
{
    if (x < min_value)
    {
        return min_value;
    }

    if (x > max_value)
    {
        return max_value;
    }

    return x;
}

static bool ADS_EKF_IsFiniteFloat(float x)
{
    return isfinite(x) ? true : false;
}

static bool ADS_EKF_VectorIsFinite(Vector3 v)
{
    return ADS_EKF_IsFiniteFloat(v.x) &&
           ADS_EKF_IsFiniteFloat(v.y) &&
           ADS_EKF_IsFiniteFloat(v.z);
}

static bool ADS_EKF_VectorIsUsable(Vector3 v)
{
    if (!ADS_EKF_VectorIsFinite(v))
    {
        return false;
    }

    if (Vector3_Norm(v) <= ADS_EKF_EPSILON)
    {
        return false;
    }

    return true;
}

static Vector3 ADS_EKF_VectorClampNorm(Vector3 v, float max_norm)
{
    const float n = Vector3_Norm(v);

    if (n <= max_norm)
    {
        return v;
    }

    if (n <= ADS_EKF_EPSILON)
    {
        Vector3 zero = {0.0f, 0.0f, 0.0f};
        return zero;
    }

    return Vector3_Scale(v, max_norm / n);
}

static Vector3 ADS_EKF_VectorSubBias(Vector3 gyro, Vector3 bias)
{
    return Vector3_Sub(gyro, bias);
}

static Vector3 ADS_EKF_InverseRotateVector(Quaternion q_body_to_inertial,
                                            Vector3 inertial_vector)
{
    /*
     * Quaternion_RotateVector(q, v) applies:
     *   v_inertial = q * v_body * q_conj
     *
     * To predict a body-frame sensor vector from an inertial reference:
     *   v_body = q_conj * v_inertial * q
     */
    Quaternion q_conj = Quaternion_Conjugate(Quaternion_Normalize(q_body_to_inertial));
    return Quaternion_RotateVector(q_conj, inertial_vector);
}

static void ADS_EKF_ResetCovariance(void)
{
    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c = 0U; c < ADS_EKF_STATE_DIM; c++)
        {
            g_ads_ekf_P[r][c] = 0.0f;
        }
    }

    g_ads_ekf_P[0][0] = ADS_EKF_INITIAL_ATT_VAR;
    g_ads_ekf_P[1][1] = ADS_EKF_INITIAL_ATT_VAR;
    g_ads_ekf_P[2][2] = ADS_EKF_INITIAL_ATT_VAR;

    g_ads_ekf_P[3][3] = ADS_EKF_INITIAL_BIAS_VAR;
    g_ads_ekf_P[4][4] = ADS_EKF_INITIAL_BIAS_VAR;
    g_ads_ekf_P[5][5] = ADS_EKF_INITIAL_BIAS_VAR;
}

static float ADS_EKF_CovarianceTrace(void)
{
    float trace = 0.0f;

    for (unsigned int i = 0U; i < ADS_EKF_STATE_DIM; i++)
    {
        trace += g_ads_ekf_P[i][i];
    }

    return trace;
}

static void ADS_EKF_SymmetriseCovariance(void)
{
    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c = r + 1U; c < ADS_EKF_STATE_DIM; c++)
        {
            const float average = 0.5f * (g_ads_ekf_P[r][c] + g_ads_ekf_P[c][r]);
            g_ads_ekf_P[r][c] = average;
            g_ads_ekf_P[c][r] = average;
        }
    }

    for (unsigned int i = 0U; i < ADS_EKF_STATE_DIM; i++)
    {
        if ((!ADS_EKF_IsFiniteFloat(g_ads_ekf_P[i][i])) ||
            (g_ads_ekf_P[i][i] < 1.0e-9f))
        {
            g_ads_ekf_P[i][i] = 1.0e-9f;
        }
    }
}

static void ADS_EKF_PropagateCovariance(float dt_s)
{
    /*
     * Simple 6-state covariance propagation.
     *
     * Error dynamics approximation:
     *   dtheta_dot = -dbias
     *   dbias_dot  = noise
     *
     * Discrete F:
     *   F = [I  -dt*I
     *        0    I   ]
     */
    float F[ADS_EKF_STATE_DIM][ADS_EKF_STATE_DIM];
    float FP[ADS_EKF_STATE_DIM][ADS_EKF_STATE_DIM];
    float FPFt[ADS_EKF_STATE_DIM][ADS_EKF_STATE_DIM];

    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c = 0U; c < ADS_EKF_STATE_DIM; c++)
        {
            F[r][c] = 0.0f;
            FP[r][c] = 0.0f;
            FPFt[r][c] = 0.0f;
        }

        F[r][r] = 1.0f;
    }

    F[0][3] = -dt_s;
    F[1][4] = -dt_s;
    F[2][5] = -dt_s;

    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c = 0U; c < ADS_EKF_STATE_DIM; c++)
        {
            for (unsigned int k = 0U; k < ADS_EKF_STATE_DIM; k++)
            {
                FP[r][c] += F[r][k] * g_ads_ekf_P[k][c];
            }
        }
    }

    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c = 0U; c < ADS_EKF_STATE_DIM; c++)
        {
            for (unsigned int k = 0U; k < ADS_EKF_STATE_DIM; k++)
            {
                FPFt[r][c] += FP[r][k] * F[c][k];
            }
        }
    }

    memcpy(g_ads_ekf_P, FPFt, sizeof(g_ads_ekf_P));

    g_ads_ekf_P[0][0] += ADS_EKF_GYRO_NOISE_VAR * dt_s;
    g_ads_ekf_P[1][1] += ADS_EKF_GYRO_NOISE_VAR * dt_s;
    g_ads_ekf_P[2][2] += ADS_EKF_GYRO_NOISE_VAR * dt_s;

    g_ads_ekf_P[3][3] += ADS_EKF_BIAS_RANDOM_WALK_VAR * dt_s;
    g_ads_ekf_P[4][4] += ADS_EKF_BIAS_RANDOM_WALK_VAR * dt_s;
    g_ads_ekf_P[5][5] += ADS_EKF_BIAS_RANDOM_WALK_VAR * dt_s;

    ADS_EKF_SymmetriseCovariance();
}

static void ADS_EKF_ApplyCorrection(Vector3 delta_theta, Vector3 delta_bias)
{
    delta_theta = ADS_EKF_VectorClampNorm(delta_theta, ADS_EKF_MAX_CORRECTION_RAD);

    Quaternion dq = Quaternion_FromSmallAngle(delta_theta);

    /*
     * Right-multiplicative correction, matching Quaternion_PropagateGyro():
     *   q_new = q_old ⊗ dq
     */
    g_ads_ekf_state.q_body_to_inertial =
        Quaternion_Normalize(
            Quaternion_Multiply(g_ads_ekf_state.q_body_to_inertial, dq)
        );

    g_ads_ekf_state.gyro_bias_rad_s =
        Vector3_Add(g_ads_ekf_state.gyro_bias_rad_s, delta_bias);

    g_ads_ekf_state.gyro_bias_rad_s.x =
        ADS_EKF_Clamp(g_ads_ekf_state.gyro_bias_rad_s.x,
                      -ADS_EKF_MAX_BIAS_RAD_S,
                       ADS_EKF_MAX_BIAS_RAD_S);

    g_ads_ekf_state.gyro_bias_rad_s.y =
        ADS_EKF_Clamp(g_ads_ekf_state.gyro_bias_rad_s.y,
                      -ADS_EKF_MAX_BIAS_RAD_S,
                       ADS_EKF_MAX_BIAS_RAD_S);

    g_ads_ekf_state.gyro_bias_rad_s.z =
        ADS_EKF_Clamp(g_ads_ekf_state.gyro_bias_rad_s.z,
                      -ADS_EKF_MAX_BIAS_RAD_S,
                       ADS_EKF_MAX_BIAS_RAD_S);
}

static bool ADS_EKF_UpdateVector(Vector3 measured_body,
                                 Vector3 reference_inertial,
                                 float measurement_variance)
{
    if (!ADS_EKF_VectorIsUsable(measured_body))
    {
        return false;
    }

    if (!ADS_EKF_VectorIsUsable(reference_inertial))
    {
        return false;
    }

    measured_body = Vector3_Normalize(measured_body);
    reference_inertial = Vector3_Normalize(reference_inertial);

    const Vector3 predicted_body =
        Vector3_Normalize(
            ADS_EKF_InverseRotateVector(
                g_ads_ekf_state.q_body_to_inertial,
                reference_inertial
            )
        );

    if (!ADS_EKF_VectorIsUsable(predicted_body))
    {
        return false;
    }

    /*
     * Innovation as small rotation that moves predicted_body toward measured_body.
     */
    const Vector3 innovation = Vector3_Cross(predicted_body, measured_body);

    /*
     * Measurement model:
     *   innovation ≈ H * x_err
     *
     * For a unit vector observation, the attitude block is approximated
     * by the skew-symmetric form of predicted_body.
     *
     * H = [ -skew(predicted_body)   0_3x3 ]
     */
    float H[3][ADS_EKF_STATE_DIM];

    for (unsigned int r = 0U; r < 3U; r++)
    {
        for (unsigned int c = 0U; c < ADS_EKF_STATE_DIM; c++)
        {
            H[r][c] = 0.0f;
        }
    }

    H[0][1] =  predicted_body.z;
    H[0][2] = -predicted_body.y;

    H[1][0] = -predicted_body.z;
    H[1][2] =  predicted_body.x;

    H[2][0] =  predicted_body.y;
    H[2][1] = -predicted_body.x;

    float HP[3][ADS_EKF_STATE_DIM];
    float S[3][3];

    for (unsigned int r = 0U; r < 3U; r++)
    {
        for (unsigned int c = 0U; c < ADS_EKF_STATE_DIM; c++)
        {
            HP[r][c] = 0.0f;

            for (unsigned int k = 0U; k < ADS_EKF_STATE_DIM; k++)
            {
                HP[r][c] += H[r][k] * g_ads_ekf_P[k][c];
            }
        }
    }

    for (unsigned int r = 0U; r < 3U; r++)
    {
        for (unsigned int c = 0U; c < 3U; c++)
        {
            S[r][c] = 0.0f;

            for (unsigned int k = 0U; k < ADS_EKF_STATE_DIM; k++)
            {
                S[r][c] += HP[r][k] * H[c][k];
            }

            if (r == c)
            {
                S[r][c] += measurement_variance;
            }
        }
    }

    /*
     * Invert 3x3 S.
     */
    const float a = S[0][0];
    const float b = S[0][1];
    const float c = S[0][2];

    const float d = S[1][0];
    const float e = S[1][1];
    const float f = S[1][2];

    const float g = S[2][0];
    const float h = S[2][1];
    const float i = S[2][2];

    const float det =
        (a * ((e * i) - (f * h))) -
        (b * ((d * i) - (f * g))) +
        (c * ((d * h) - (e * g)));

    if ((!ADS_EKF_IsFiniteFloat(det)) || (ADS_EKF_Abs(det) < 1.0e-12f))
    {
        return false;
    }

    const float inv_det = 1.0f / det;

    float S_inv[3][3];

    S_inv[0][0] =  ((e * i) - (f * h)) * inv_det;
    S_inv[0][1] = -((b * i) - (c * h)) * inv_det;
    S_inv[0][2] =  ((b * f) - (c * e)) * inv_det;

    S_inv[1][0] = -((d * i) - (f * g)) * inv_det;
    S_inv[1][1] =  ((a * i) - (c * g)) * inv_det;
    S_inv[1][2] = -((a * f) - (c * d)) * inv_det;

    S_inv[2][0] =  ((d * h) - (e * g)) * inv_det;
    S_inv[2][1] = -((a * h) - (b * g)) * inv_det;
    S_inv[2][2] =  ((a * e) - (b * d)) * inv_det;

    /*
     * K_gain = P H^T S^-1
     */
    float PHt[ADS_EKF_STATE_DIM][3];
    float K_gain[ADS_EKF_STATE_DIM][3];

    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c2 = 0U; c2 < 3U; c2++)
        {
            PHt[r][c2] = 0.0f;
            K_gain[r][c2] = 0.0f;

            for (unsigned int k = 0U; k < ADS_EKF_STATE_DIM; k++)
            {
                PHt[r][c2] += g_ads_ekf_P[r][k] * H[c2][k];
            }
        }
    }

    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c2 = 0U; c2 < 3U; c2++)
        {
            for (unsigned int k = 0U; k < 3U; k++)
            {
                K_gain[r][c2] += PHt[r][k] * S_inv[k][c2];
            }
        }
    }

    const float innov_vec[3] =
    {
        innovation.x,
        innovation.y,
        innovation.z
    };

    float dx[ADS_EKF_STATE_DIM];

    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        dx[r] = 0.0f;

        for (unsigned int k = 0U; k < 3U; k++)
        {
            dx[r] += K_gain[r][k] * innov_vec[k];
        }

        if (!ADS_EKF_IsFiniteFloat(dx[r]))
        {
            return false;
        }
    }

    Vector3 delta_theta =
    {
        dx[0],
        dx[1],
        dx[2]
    };

    Vector3 delta_bias =
    {
        dx[3],
        dx[4],
        dx[5]
    };

    ADS_EKF_ApplyCorrection(delta_theta, delta_bias);

    /*
     * Covariance update:
     *   P = (I - K H) P
     */
    float KH[ADS_EKF_STATE_DIM][ADS_EKF_STATE_DIM];
    float I_minus_KH[ADS_EKF_STATE_DIM][ADS_EKF_STATE_DIM];
    float P_new[ADS_EKF_STATE_DIM][ADS_EKF_STATE_DIM];

    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c2 = 0U; c2 < ADS_EKF_STATE_DIM; c2++)
        {
            KH[r][c2] = 0.0f;
            I_minus_KH[r][c2] = 0.0f;
            P_new[r][c2] = 0.0f;

            for (unsigned int k = 0U; k < 3U; k++)
            {
                KH[r][c2] += K_gain[r][k] * H[k][c2];
            }

            I_minus_KH[r][c2] = ((r == c2) ? 1.0f : 0.0f) - KH[r][c2];
        }
    }

    for (unsigned int r = 0U; r < ADS_EKF_STATE_DIM; r++)
    {
        for (unsigned int c2 = 0U; c2 < ADS_EKF_STATE_DIM; c2++)
        {
            for (unsigned int k = 0U; k < ADS_EKF_STATE_DIM; k++)
            {
                P_new[r][c2] += I_minus_KH[r][k] * g_ads_ekf_P[k][c2];
            }
        }
    }

    memcpy(g_ads_ekf_P, P_new, sizeof(g_ads_ekf_P));
    ADS_EKF_SymmetriseCovariance();

    return true;
}

void ADS_EKF_Init(void)
{
    g_ads_ekf_state.q_body_to_inertial = Quaternion_Identity();

    g_ads_ekf_state.gyro_bias_rad_s.x = 0.0f;
    g_ads_ekf_state.gyro_bias_rad_s.y = 0.0f;
    g_ads_ekf_state.gyro_bias_rad_s.z = 0.0f;

    g_ads_ekf_state.covariance_trace = 0.0f;
    g_ads_ekf_state.update_count = 0U;

    g_ads_ekf_state.initialized = true;
    g_ads_ekf_state.healthy = true;

    ADS_EKF_ResetCovariance();
    g_ads_ekf_state.covariance_trace = ADS_EKF_CovarianceTrace();
}

bool ADS_EKF_PredictUpdate(const ADS_EKF_Input* input)
{
    if (input == 0)
    {
        g_ads_ekf_state.healthy = false;
        return false;
    }

    if (!g_ads_ekf_state.initialized)
    {
        ADS_EKF_Init();
    }

    if ((!ADS_EKF_IsFiniteFloat(input->dt_s)) ||
        (input->dt_s < ADS_EKF_DT_MIN_S) ||
        (input->dt_s > ADS_EKF_DT_MAX_S))
    {
        g_ads_ekf_state.healthy = false;
        return false;
    }

    if ((!input->gyro_valid) || (!ADS_EKF_VectorIsFinite(input->gyro_rad_s)))
    {
        g_ads_ekf_state.healthy = false;
        return false;
    }

    const Vector3 corrected_gyro =
        ADS_EKF_VectorSubBias(input->gyro_rad_s,
                              g_ads_ekf_state.gyro_bias_rad_s);

    g_ads_ekf_state.q_body_to_inertial =
        Quaternion_PropagateGyro(g_ads_ekf_state.q_body_to_inertial,
                                 corrected_gyro,
                                 input->dt_s);

    ADS_EKF_PropagateCovariance(input->dt_s);

    bool measurement_update_used = false;

    if (input->accel_valid)
    {
        measurement_update_used =
            ADS_EKF_UpdateVector(input->accel_body_m_s2,
                                 input->gravity_inertial_m_s2,
                                 ADS_EKF_ACCEL_MEAS_VAR) ||
            measurement_update_used;
    }

    if (input->mag_valid)
    {
        measurement_update_used =
            ADS_EKF_UpdateVector(input->mag_body_T,
                                 input->mag_inertial_T,
                                 ADS_EKF_MAG_MEAS_VAR) ||
            measurement_update_used;
    }

    /*
     * Gyro-only propagation is still a valid fallback path.
     * So health remains true if propagation succeeded, even when mag update is skipped.
     */
    g_ads_ekf_state.q_body_to_inertial =
        Quaternion_Normalize(g_ads_ekf_state.q_body_to_inertial);

    g_ads_ekf_state.update_count++;
    g_ads_ekf_state.covariance_trace = ADS_EKF_CovarianceTrace();
    g_ads_ekf_state.healthy = true;

    (void)measurement_update_used;

    return true;
}

ADS_EKF_State ADS_EKF_GetState(void)
{
    return g_ads_ekf_state;
}