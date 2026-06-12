#ifndef VECTOR3_H
#define VECTOR3_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x;
    float y;
    float z;
} Vector3;

Vector3 Vector3_Zero(void);
Vector3 Vector3_Add(Vector3 a, Vector3 b);
Vector3 Vector3_Sub(Vector3 a, Vector3 b);
Vector3 Vector3_Scale(Vector3 v, float scalar);
float   Vector3_Dot(Vector3 a, Vector3 b);
Vector3 Vector3_Cross(Vector3 a, Vector3 b);
float   Vector3_Norm(Vector3 v);
Vector3 Vector3_Normalize(Vector3 v);

#ifdef __cplusplus
}
#endif

#endif /* VECTOR3_H */
