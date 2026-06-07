#ifndef QUEST_H
#define QUEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vector3.h"
#include "quaternion.h"

#define QUEST_MAX_VECTOR_PAIRS 4

typedef struct
{
    Vector3 body_vectors[QUEST_MAX_VECTOR_PAIRS];
    Vector3 inertial_vectors[QUEST_MAX_VECTOR_PAIRS];
    float weights[QUEST_MAX_VECTOR_PAIRS];

    unsigned int vector_count;

} QUEST_Input;

typedef struct
{
    Quaternion attitude_q;
    float residual;
    unsigned char valid;

} QUEST_Output;

void QUEST_InitInput(QUEST_Input* input);

unsigned char QUEST_AddVectorPair(
    QUEST_Input* input,
    Vector3 body_vector,
    Vector3 inertial_vector,
    float weight
);

QUEST_Output QUEST_Solve(const QUEST_Input* input);

#ifdef __cplusplus
}
#endif

#endif /* QUEST_H */