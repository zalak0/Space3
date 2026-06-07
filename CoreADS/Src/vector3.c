#include "vector3.h"
#include <math.h>

Vector3 Vector3_Add(Vector3 a, Vector3 b)
{
    Vector3 result;

    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;

    return result;
}

Vector3 Vector3_Sub(Vector3 a, Vector3 b)
{
    Vector3 result;

    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;

    return result;
}

Vector3 Vector3_Scale(Vector3 v, float scalar)
{
    Vector3 result;

    result.x = scalar * v.x;
    result.y = scalar * v.y;
    result.z = scalar * v.z;

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
    float norm = Vector3_Norm(v);

    if (norm <= 1.0e-6f)
    {
        Vector3 zero = {0.0f, 0.0f, 0.0f};
        return zero;
    }

    return Vector3_Scale(v, 1.0f / norm);
}