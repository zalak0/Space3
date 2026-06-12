#ifndef QUATERNION_H
#define QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vector3.h"

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} EulerAngles;

/* Constructors / basic operations */
Quaternion Quaternion_Identity(void);
Quaternion Quaternion_Normalize(Quaternion q);
Quaternion Quaternion_Conjugate(Quaternion q);
Quaternion Quaternion_Multiply(Quaternion q1, Quaternion q2);

/* Rotate vector by quaternion */
Vector3 Quaternion_RotateVector(Quaternion q, Vector3 v);

/* Create small-angle correction quaternion */
Quaternion Quaternion_FromSmallAngle(Vector3 delta_theta);

/* Propagate attitude using gyro angular velocity */
Quaternion Quaternion_PropagateGyro(Quaternion q, Vector3 omega_rad_s, float dt);

EulerAngles Quaternion_ToEuler321(Quaternion q);




#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_H */