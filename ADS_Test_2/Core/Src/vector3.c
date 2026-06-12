#include "vector3.h"

#include <math.h>

#define VECTOR3_EPSILON (1.0e-6f)

Vector3 Vector3_Zero(void)
{
    Vector3 v = {0.0f, 0.0f, 0.0f};
    return v;
}

Vector3 Vector3_Add(Vector3 a, Vector3 b)
{
    Vector3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

Vector3 Vector3_Sub(Vector3 a, Vector3 b)
{
    Vector3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

Vector3 Vector3_Scale(Vector3 v, float scalar)
{
    Vector3 result = {v.x * scalar, v.y * scalar, v.z * scalar};
    return result;
}

float Vector3_Dot(Vector3 a, Vector3 b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

Vector3 Vector3_Cross(Vector3 a, Vector3 b)
{
    Vector3 result;

    result.x = (a.y * b.z) - (a.z * b.y);
    result.y = (a.z * b.x) - (a.x * b.z);
    result.z = (a.x * b.y) - (a.y * b.x);

    return result;
}

float Vector3_Norm(Vector3 v)
{
    return sqrtf(Vector3_Dot(v, v));
}

Vector3 Vector3_Normalize(Vector3 v)
{
    const float n = Vector3_Norm(v);

    if (n <= VECTOR3_EPSILON)
    {
        return Vector3_Zero();
    }

    return Vector3_Scale(v, 1.0f / n);
}
