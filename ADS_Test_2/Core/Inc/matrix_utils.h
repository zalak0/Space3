#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vector3.h"

typedef struct
{
    float m[3][3];
} Matrix3;

Matrix3 Matrix3_Zero(void);
Matrix3 Matrix3_Identity(void);

Matrix3 Matrix3_Add(Matrix3 A, Matrix3 B);
Matrix3 Matrix3_Sub(Matrix3 A, Matrix3 B);
Matrix3 Matrix3_Scale(Matrix3 A, float scalar);

Vector3 Matrix3_MultiplyVector(Matrix3 A, Vector3 v);
Matrix3 Matrix3_Multiply(Matrix3 A, Matrix3 B);
Matrix3 Matrix3_Transpose(Matrix3 A);

Matrix3 Matrix3_Skew(Vector3 v);

#ifdef __cplusplus
}
#endif

#endif /* MATRIX_UTILS_H */