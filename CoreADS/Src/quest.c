#include "quest.h"

void QUEST_InitInput(QUEST_Input* input)
{
    if (input == 0)
    {
        return;
    }

    input->vector_count = 0;

    for (unsigned int i = 0; i < QUEST_MAX_VECTOR_PAIRS; i++)
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
        return 0;
    }

    if (input->vector_count >= QUEST_MAX_VECTOR_PAIRS)
    {
        return 0;
    }

    if (weight <= 0.0f)
    {
        return 0;
    }

    unsigned int i = input->vector_count;

    input->body_vectors[i] = Vector3_Normalize(body_vector);
    input->inertial_vectors[i] = Vector3_Normalize(inertial_vector);
    input->weights[i] = weight;

    input->vector_count++;

    return 1;
}

QUEST_Output QUEST_Solve(const QUEST_Input* input)
{
    QUEST_Output output;

    output.attitude_q = Quaternion_Identity();
    output.residual = 0.0f;
    output.valid = 0;

    /*
     * Placeholder implementation.
     *
     * Later this will implement QUEST/Davenport q-method using:
     *   - weighted vector pairs
     *   - attitude profile matrix B
     *   - K matrix construction
     *   - largest eigenvalue/eigenvector solution
     *
     * For now, mark valid only when we have at least two vector pairs.
     */

    if (input == 0)
    {
        return output;
    }

    if (input->vector_count >= 2)
    {
        output.valid = 1;
        output.attitude_q = Quaternion_Identity();
        output.residual = 0.0f;
    }

    return output;
}