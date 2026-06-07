#include "matrix_utils.h"

Matrix3 Matrix3_Zero(void)
{
    Matrix3 A;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            A.m[i][j] = 0.0f;
        }
    }

    return A;
}

Matrix3 Matrix3_Identity(void)
{
    Matrix3 A = Matrix3_Zero();

    A.m[0][0] = 1.0f;
    A.m[1][1] = 1.0f;
    A.m[2][2] = 1.0f;

    return A;
}

Matrix3 Matrix3_Add(Matrix3 A, Matrix3 B)
{
    Matrix3 C;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C.m[i][j] = A.m[i][j] + B.m[i][j];
        }
    }

    return C;
}

Matrix3 Matrix3_Sub(Matrix3 A, Matrix3 B)
{
    Matrix3 C;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C.m[i][j] = A.m[i][j] - B.m[i][j];
        }
    }

    return C;
}

Matrix3 Matrix3_Scale(Matrix3 A, float scalar)
{
    Matrix3 C;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C.m[i][j] = scalar * A.m[i][j];
        }
    }

    return C;
}

Vector3 Matrix3_MultiplyVector(Matrix3 A, Vector3 v)
{
    Vector3 result;

    result.x = A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z;
    result.y = A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z;
    result.z = A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z;

    return result;
}

Matrix3 Matrix3_Multiply(Matrix3 A, Matrix3 B)
{
    Matrix3 C = Matrix3_Zero();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                C.m[i][j] += A.m[i][k] * B.m[k][j];
            }
        }
    }

    return C;
}

Matrix3 Matrix3_Transpose(Matrix3 A)
{
    Matrix3 T;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            T.m[i][j] = A.m[j][i];
        }
    }

    return T;
}

Matrix3 Matrix3_Skew(Vector3 v)
{
    Matrix3 S = Matrix3_Zero();

    S.m[0][0] = 0.0f;
    S.m[0][1] = -v.z;
    S.m[0][2] = v.y;

    S.m[1][0] = v.z;
    S.m[1][1] = 0.0f;
    S.m[1][2] = -v.x;

    S.m[2][0] = -v.y;
    S.m[2][1] = v.x;
    S.m[2][2] = 0.0f;

    return S;
}