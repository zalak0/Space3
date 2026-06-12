#include "quest.h"

#include <math.h>

#define QUEST_EPSILON              (1.0e-6f)
#define QUEST_JACOBI_MAX_ITERS     (50U)
#define QUEST_MAX_RESIDUAL_WARN    (0.50f)

static unsigned char QUEST_IsFiniteFloat(float x)
{
    return isfinite(x) ? 1U : 0U;
}

static unsigned char QUEST_VectorIsFinite(Vector3 v)
{
    return (QUEST_IsFiniteFloat(v.x) &&
            QUEST_IsFiniteFloat(v.y) &&
            QUEST_IsFiniteFloat(v.z));
}

static unsigned char QUEST_VectorIsUsable(Vector3 v)
{
    if (!QUEST_VectorIsFinite(v))
    {
        return 0U;
    }

    if (Vector3_Norm(v) <= QUEST_EPSILON)
    {
        return 0U;
    }

    return 1U;
}

static void QUEST_SetIdentity4(float A[4][4])
{
    for (unsigned int r = 0U; r < 4U; r++)
    {
        for (unsigned int c = 0U; c < 4U; c++)
        {
            A[r][c] = 0.0f;
        }

        A[r][r] = 1.0f;
    }
}

static void QUEST_Normalize4(float q[4])
{
    const float n = sqrtf(
        (q[0] * q[0]) +
        (q[1] * q[1]) +
        (q[2] * q[2]) +
        (q[3] * q[3])
    );

    if (n <= QUEST_EPSILON)
    {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;

    /*
     * q and -q describe the same attitude.
     * Keep scalar part positive for cleaner telemetry.
     */
    if (q[0] < 0.0f)
    {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
}

static void QUEST_LargestEigenvectorSymmetric4(const float input_K[4][4], float q_out[4])
{
    /*
     * Small fixed-size Jacobi eigensolver for symmetric 4x4 matrix.
     * No dynamic allocation.
     *
     * Eigenvectors are stored as columns of V.
     */
    float K[4][4];
    float V[4][4];

    for (unsigned int r = 0U; r < 4U; r++)
    {
        for (unsigned int c = 0U; c < 4U; c++)
        {
            K[r][c] = input_K[r][c];
        }
    }

    QUEST_SetIdentity4(V);

    for (unsigned int iter = 0U; iter < QUEST_JACOBI_MAX_ITERS; iter++)
    {
        unsigned int p = 0U;
        unsigned int q = 1U;
        float max_offdiag = fabsf(K[p][q]);

        for (unsigned int r = 0U; r < 4U; r++)
        {
            for (unsigned int c = r + 1U; c < 4U; c++)
            {
                const float value = fabsf(K[r][c]);

                if (value > max_offdiag)
                {
                    max_offdiag = value;
                    p = r;
                    q = c;
                }
            }
        }

        if (max_offdiag < 1.0e-7f)
        {
            break;
        }

        const float app = K[p][p];
        const float aqq = K[q][q];
        const float apq = K[p][q];

        if (fabsf(apq) < 1.0e-12f)
        {
            continue;
        }

        const float tau = (aqq - app) / (2.0f * apq);
        const float t_sign = (tau >= 0.0f) ? 1.0f : -1.0f;
        const float t = t_sign / (fabsf(tau) + sqrtf(1.0f + (tau * tau)));
        const float c_rot = 1.0f / sqrtf(1.0f + (t * t));
        const float s_rot = t * c_rot;

        for (unsigned int i = 0U; i < 4U; i++)
        {
            if ((i != p) && (i != q))
            {
                const float kip = K[i][p];
                const float kiq = K[i][q];

                K[i][p] = (c_rot * kip) - (s_rot * kiq);
                K[p][i] = K[i][p];

                K[i][q] = (s_rot * kip) + (c_rot * kiq);
                K[q][i] = K[i][q];
            }
        }

        K[p][p] = (c_rot * c_rot * app) -
                  (2.0f * s_rot * c_rot * apq) +
                  (s_rot * s_rot * aqq);

        K[q][q] = (s_rot * s_rot * app) +
                  (2.0f * s_rot * c_rot * apq) +
                  (c_rot * c_rot * aqq);

        K[p][q] = 0.0f;
        K[q][p] = 0.0f;

        for (unsigned int i = 0U; i < 4U; i++)
        {
            const float vip = V[i][p];
            const float viq = V[i][q];

            V[i][p] = (c_rot * vip) - (s_rot * viq);
            V[i][q] = (s_rot * vip) + (c_rot * viq);
        }
    }

    unsigned int max_index = 0U;
    float max_eigenvalue = K[0][0];

    for (unsigned int i = 1U; i < 4U; i++)
    {
        if (K[i][i] > max_eigenvalue)
        {
            max_eigenvalue = K[i][i];
            max_index = i;
        }
    }

    q_out[0] = V[0][max_index];
    q_out[1] = V[1][max_index];
    q_out[2] = V[2][max_index];
    q_out[3] = V[3][max_index];

    QUEST_Normalize4(q_out);
}

static float QUEST_ComputeResidual(const QUEST_Input* input, Quaternion q_body_to_inertial)
{
    float weighted_error_sum = 0.0f;
    float weight_sum = 0.0f;

    for (unsigned int i = 0U; i < input->vector_count; i++)
    {
        const Vector3 body = input->body_vectors[i];
        const Vector3 inertial = input->inertial_vectors[i];
        const float weight = input->weights[i];

        const Vector3 predicted_inertial = Quaternion_RotateVector(q_body_to_inertial, body);
        const Vector3 error = Vector3_Sub(inertial, predicted_inertial);

        weighted_error_sum += weight * Vector3_Dot(error, error);
        weight_sum += weight;
    }

    if (weight_sum <= QUEST_EPSILON)
    {
        return 999.0f;
    }

    return sqrtf(weighted_error_sum / weight_sum);
}

void QUEST_InitInput(QUEST_Input* input)
{
    if (input == 0)
    {
        return;
    }

    input->vector_count = 0U;

    for (unsigned int i = 0U; i < QUEST_MAX_VECTOR_PAIRS; i++)
    {
        input->body_vectors[i].x = 0.0f;
        input->body_vectors[i].y = 0.0f;
        input->body_vectors[i].z = 0.0f;

        input->inertial_vectors[i].x = 0.0f;
        input->inertial_vectors[i].y = 0.0f;
        input->inertial_vectors[i].z = 0.0f;

        input->weights[i] = 0.0f;
    }
}

unsigned char QUEST_AddVectorPair(
    QUEST_Input* input,
    Vector3 body_vector,
    Vector3 inertial_vector,
    float weight
)
{
    if (input == 0)
    {
        return 0U;
    }

    if (input->vector_count >= QUEST_MAX_VECTOR_PAIRS)
    {
        return 0U;
    }

    if ((!QUEST_IsFiniteFloat(weight)) || (weight <= 0.0f))
    {
        return 0U;
    }

    if (!QUEST_VectorIsUsable(body_vector))
    {
        return 0U;
    }

    if (!QUEST_VectorIsUsable(inertial_vector))
    {
        return 0U;
    }

    const unsigned int i = input->vector_count;

    input->body_vectors[i] = Vector3_Normalize(body_vector);
    input->inertial_vectors[i] = Vector3_Normalize(inertial_vector);
    input->weights[i] = weight;

    input->vector_count++;

    return 1U;
}

QUEST_Output QUEST_Solve(const QUEST_Input* input)
{
    QUEST_Output output;

    output.attitude_q = Quaternion_Identity();
    output.residual = 999.0f;
    output.valid = 0U;

    if (input == 0)
    {
        return output;
    }

    /*
     * QUEST/Davenport needs at least two non-collinear vector observations
     * for a full 3-axis attitude estimate.
     */
    if (input->vector_count < 2U)
    {
        return output;
    }

    float weight_sum = 0.0f;

    /*
     * Attitude profile matrix.
     *
     * Convention used here:
     * - Quaternion output maps body frame to inertial frame.
     * - Quaternion_RotateVector(q, body_vector) should predict inertial_vector.
     * - Therefore B = sum(weight * body_vector * inertial_vector^T).
     */
    float B[3][3] =
    {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f}
    };

    for (unsigned int i = 0U; i < input->vector_count; i++)
    {
        const Vector3 b = input->body_vectors[i];
        const Vector3 r = input->inertial_vectors[i];
        const float w = input->weights[i];

        if ((!QUEST_VectorIsUsable(b)) ||
            (!QUEST_VectorIsUsable(r)) ||
            (!QUEST_IsFiniteFloat(w)) ||
            (w <= 0.0f))
        {
            return output;
        }

        B[0][0] += w * b.x * r.x;
        B[0][1] += w * b.x * r.y;
        B[0][2] += w * b.x * r.z;

        B[1][0] += w * b.y * r.x;
        B[1][1] += w * b.y * r.y;
        B[1][2] += w * b.y * r.z;

        B[2][0] += w * b.z * r.x;
        B[2][1] += w * b.z * r.y;
        B[2][2] += w * b.z * r.z;

        weight_sum += w;
    }

    if (weight_sum <= QUEST_EPSILON)
    {
        return output;
    }

    const float sigma = B[0][0] + B[1][1] + B[2][2];

    const float S00 = B[0][0] + B[0][0];
    const float S01 = B[0][1] + B[1][0];
    const float S02 = B[0][2] + B[2][0];

    const float S10 = S01;
    const float S11 = B[1][1] + B[1][1];
    const float S12 = B[1][2] + B[2][1];

    const float S20 = S02;
    const float S21 = S12;
    const float S22 = B[2][2] + B[2][2];

    const float Z0 = B[1][2] - B[2][1];
    const float Z1 = B[2][0] - B[0][2];
    const float Z2 = B[0][1] - B[1][0];

    /*
     * Scalar-first Davenport K matrix.
     *
     * Eigenvector layout:
     * q = [qw, qx, qy, qz]
     */
    float K[4][4];

    K[0][0] = sigma;
    K[0][1] = Z0;
    K[0][2] = Z1;
    K[0][3] = Z2;

    K[1][0] = Z0;
    K[1][1] = S00 - sigma;
    K[1][2] = S01;
    K[1][3] = S02;

    K[2][0] = Z1;
    K[2][1] = S10;
    K[2][2] = S11 - sigma;
    K[2][3] = S12;

    K[3][0] = Z2;
    K[3][1] = S20;
    K[3][2] = S21;
    K[3][3] = S22 - sigma;

    float q_vec[4];
    QUEST_LargestEigenvectorSymmetric4(K, q_vec);

    Quaternion q_solution;
    q_solution.w = q_vec[0];
    q_solution.x = q_vec[1];
    q_solution.y = q_vec[2];
    q_solution.z = q_vec[3];
    q_solution = Quaternion_Normalize(q_solution);

    output.attitude_q = q_solution;
    output.residual = QUEST_ComputeResidual(input, q_solution);

    if (QUEST_IsFiniteFloat(output.residual) &&
        (output.residual < QUEST_MAX_RESIDUAL_WARN))
    {
        output.valid = 1U;
    }
    else
    {
        /*
         * Keep the attitude estimate available for telemetry/debug,
         * but mark it invalid if the vector fit is awful.
         */
        output.valid = 0U;
    }

    return output;
}