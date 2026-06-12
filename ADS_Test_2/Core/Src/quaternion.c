#include "quaternion.h"
#include <math.h>

Quaternion Quaternion_Identity(void)
{
    Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}

Quaternion Quaternion_Normalize(Quaternion q)
{
    float norm = sqrtf(
        (q.w * q.w) +
        (q.x * q.x) +
        (q.y * q.y) +
        (q.z * q.z)
    );

    if (norm <= 1.0e-6f)
    {
        return Quaternion_Identity();
    }

    Quaternion result;

    result.w = q.w / norm;
    result.x = q.x / norm;
    result.y = q.y / norm;
    result.z = q.z / norm;

    return result;
}

Quaternion Quaternion_Conjugate(Quaternion q)
{
    Quaternion result;

    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;

    return result;
}

Quaternion Quaternion_Multiply(Quaternion q1, Quaternion q2)
{
    Quaternion result;

    result.w = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z);
    result.x = (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y);
    result.y = (q1.w * q2.y) - (q1.x * q2.z) + (q1.y * q2.w) + (q1.z * q2.x);
    result.z = (q1.w * q2.z) + (q1.x * q2.y) - (q1.y * q2.x) + (q1.z * q2.w);

    return result;
}

Vector3 Quaternion_RotateVector(Quaternion q, Vector3 v)
{
    Quaternion q_normalized = Quaternion_Normalize(q);

    Quaternion v_quat;
    v_quat.w = 0.0f;
    v_quat.x = v.x;
    v_quat.y = v.y;
    v_quat.z = v.z;

    Quaternion q_conj = Quaternion_Conjugate(q_normalized);

    Quaternion rotated_quat = Quaternion_Multiply(
        Quaternion_Multiply(q_normalized, v_quat),
        q_conj
    );

    Vector3 result;
    result.x = rotated_quat.x;
    result.y = rotated_quat.y;
    result.z = rotated_quat.z;

    return result;
}

Quaternion Quaternion_FromSmallAngle(Vector3 delta_theta)
{
    Quaternion q;

    q.w = 1.0f;
    q.x = 0.5f * delta_theta.x;
    q.y = 0.5f * delta_theta.y;
    q.z = 0.5f * delta_theta.z;

    return Quaternion_Normalize(q);
}

Quaternion Quaternion_PropagateGyro(Quaternion q, Vector3 omega_rad_s, float dt)
{
    /*
     * Gyro propagation using small-angle quaternion update.
     *
     * omega_rad_s: angular velocity in rad/s
     * dt: timestep in seconds
     *
     * delta_theta = omega * dt
     * dq ≈ [1, 0.5*delta_theta]
     * q_new = q ⊗ dq
     */

    Vector3 delta_theta;

    delta_theta.x = omega_rad_s.x * dt;
    delta_theta.y = omega_rad_s.y * dt;
    delta_theta.z = omega_rad_s.z * dt;

    Quaternion dq = Quaternion_FromSmallAngle(delta_theta);

    Quaternion q_new = Quaternion_Multiply(q, dq);

    return Quaternion_Normalize(q_new);
}

EulerAngles Quaternion_ToEuler321(Quaternion q)
{
    /*
     * Converts quaternion to 3-2-1 Euler angles:
     * roll  about x-axis
     * pitch about y-axis
     * yaw   about z-axis
     *
     * Assumes quaternion is scalar-first: q = [w, x, y, z].
     */

    q = Quaternion_Normalize(q);

    EulerAngles euler;

    float sin_roll_cos_pitch = 2.0f * ((q.w * q.x) + (q.y * q.z));
    float cos_roll_cos_pitch = 1.0f - 2.0f * ((q.x * q.x) + (q.y * q.y));

    euler.roll_rad = atan2f(sin_roll_cos_pitch, cos_roll_cos_pitch);

    float sin_pitch = 2.0f * ((q.w * q.y) - (q.z * q.x));

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    euler.pitch_rad = asinf(sin_pitch);

    float sin_yaw_cos_pitch = 2.0f * ((q.w * q.z) + (q.x * q.y));
    float cos_yaw_cos_pitch = 1.0f - 2.0f * ((q.y * q.y) + (q.z * q.z));

    euler.yaw_rad = atan2f(sin_yaw_cos_pitch, cos_yaw_cos_pitch);

    return euler;
}