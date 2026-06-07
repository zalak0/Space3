#include "adcs.h"
#include "quest.h"

static ADCS_State adcs_state;

static void ADCS_UpdatePredictedSunVector(void)
{
    Quaternion q_conj = Quaternion_Conjugate(adcs_state.attitude_q);

    adcs_state.sun_body_predicted = Quaternion_RotateVector(
        q_conj,
        adcs_state.sun_inertial
    );

    adcs_state.sun_body_predicted = Vector3_Normalize(
        adcs_state.sun_body_predicted
    );
}

static void ADCS_UpdatePredictedMagVector(void)
{
    Quaternion q_conj = Quaternion_Conjugate(adcs_state.attitude_q);

    adcs_state.mag_body_predicted_uT = Quaternion_RotateVector(
        q_conj,
        adcs_state.mag_inertial_uT
    );
}

static void ADCS_UpdateSunResidual(void)
{
    if (adcs_state.sensor_status.sun_valid == 0)
    {
        adcs_state.sun_error.x = 0.0f;
        adcs_state.sun_error.y = 0.0f;
        adcs_state.sun_error.z = 0.0f;
        return;
    }

    Vector3 measured = Vector3_Normalize(adcs_state.sun_body_measured);
    Vector3 predicted = Vector3_Normalize(adcs_state.sun_body_predicted);

    adcs_state.sun_error = Vector3_Cross(measured, predicted);
}

static void ADCS_UpdateMagResidual(void)
{
    if (adcs_state.sensor_status.mag_valid == 0)
    {
        adcs_state.mag_error.x = 0.0f;
        adcs_state.mag_error.y = 0.0f;
        adcs_state.mag_error.z = 0.0f;
        return;
    }

    Vector3 measured = Vector3_Normalize(adcs_state.mag_body_measured_uT);
    Vector3 predicted = Vector3_Normalize(adcs_state.mag_body_predicted_uT);

    adcs_state.mag_error = Vector3_Cross(measured, predicted);
}

static void ADCS_ApplyVectorCorrection(void)
{
    Vector3 sun_delta = Vector3_Scale(
        adcs_state.sun_error,
        adcs_state.sun_correction_gain
    );

    Vector3 mag_delta = Vector3_Scale(
        adcs_state.mag_error,
        adcs_state.mag_correction_gain
    );

    Vector3 delta_theta = Vector3_Add(sun_delta, mag_delta);

    Quaternion dq = Quaternion_FromSmallAngle(delta_theta);

    adcs_state.attitude_q = Quaternion_Multiply(
        adcs_state.attitude_q,
        dq
    );

    adcs_state.attitude_q = Quaternion_Normalize(adcs_state.attitude_q);
}

void ADCS_Init(void)
{
    adcs_state.attitude_q = Quaternion_Identity();

    adcs_state.gyro_rad_s.x = 0.0f;
    adcs_state.gyro_rad_s.y = 0.0f;
    adcs_state.gyro_rad_s.z = 0.0f;

    adcs_state.gyro_bias_rad_s.x = 0.0f;
    adcs_state.gyro_bias_rad_s.y = 0.0f;
    adcs_state.gyro_bias_rad_s.z = 0.0f;

    adcs_state.sun_inertial.x = 1.0f;
    adcs_state.sun_inertial.y = 0.0f;
    adcs_state.sun_inertial.z = 0.0f;

    adcs_state.mag_inertial_uT.x = 20.0f;
    adcs_state.mag_inertial_uT.y = 0.0f;
    adcs_state.mag_inertial_uT.z = -40.0f;

    adcs_state.sun_body_measured.x = 1.0f;
    adcs_state.sun_body_measured.y = 0.0f;
    adcs_state.sun_body_measured.z = 0.0f;

    adcs_state.mag_body_measured_uT.x = 20.0f;
    adcs_state.mag_body_measured_uT.y = 5.0f;
    adcs_state.mag_body_measured_uT.z = -40.0f;

    adcs_state.sensor_status.gyro_valid = 1;
    adcs_state.sensor_status.sun_valid = 1;
    adcs_state.sensor_status.mag_valid = 1;

    adcs_state.dt = 0.01f;

    adcs_state.sun_correction_gain = 0.01f;
    adcs_state.mag_correction_gain = 0.005f;

    adcs_state.update_counter = 0;

    QUEST_Input quest_input;
    QUEST_InitInput(&quest_input);

    QUEST_AddVectorPair(
        &quest_input,
        adcs_state.sun_body_measured,
        adcs_state.sun_inertial,
        1.0f
    );

    QUEST_AddVectorPair(
        &quest_input,
        adcs_state.mag_body_measured_uT,
        adcs_state.mag_inertial_uT,
        0.5f
    );

    QUEST_Output quest_output = QUEST_Solve(&quest_input);

    if (quest_output.valid)
    {
        /*
        * Placeholder only.
        * Do not overwrite attitude_q yet because QUEST_Solve currently
        * returns identity until fully implemented.
        */
    }

    ADCS_UpdatePredictedSunVector();
    ADCS_UpdatePredictedMagVector();
    ADCS_UpdateSunResidual();
    ADCS_UpdateMagResidual();
}

void ADCS_SetSensorInputs(
    Vector3 gyro_rad_s,
    Vector3 sun_body_measured,
    Vector3 mag_body_measured_uT,
    ADCS_SensorStatus status
)
{
    adcs_state.sensor_status = status;

    if (status.gyro_valid)
    {
        adcs_state.gyro_rad_s = gyro_rad_s;
    }

    if (status.sun_valid)
    {
        adcs_state.sun_body_measured = Vector3_Normalize(
            sun_body_measured
        );
    }

    if (status.mag_valid)
    {
        adcs_state.mag_body_measured_uT = mag_body_measured_uT;
    }
}

void ADCS_SetInertialReferences(
    Vector3 sun_inertial,
    Vector3 mag_inertial_uT
)
{
    adcs_state.sun_inertial = Vector3_Normalize(sun_inertial);
    adcs_state.mag_inertial_uT = mag_inertial_uT;
}

void ADCS_SetDt(float dt)
{
    if ((dt > 0.0f) && (dt < 1.0f))
    {
        adcs_state.dt = dt;
    }
}

void ADCS_Update(void)
{
    adcs_state.update_counter++;

    Vector3 omega_corrected;

    if (adcs_state.sensor_status.gyro_valid)
    {
        omega_corrected.x = adcs_state.gyro_rad_s.x - adcs_state.gyro_bias_rad_s.x;
        omega_corrected.y = adcs_state.gyro_rad_s.y - adcs_state.gyro_bias_rad_s.y;
        omega_corrected.z = adcs_state.gyro_rad_s.z - adcs_state.gyro_bias_rad_s.z;

        adcs_state.attitude_q = Quaternion_PropagateGyro(
            adcs_state.attitude_q,
            omega_corrected,
            adcs_state.dt
        );
    }

    ADCS_UpdatePredictedSunVector();
    ADCS_UpdatePredictedMagVector();

    ADCS_UpdateSunResidual();
    ADCS_UpdateMagResidual();

    ADCS_ApplyVectorCorrection();

    ADCS_UpdatePredictedSunVector();
    ADCS_UpdatePredictedMagVector();
}

const ADCS_State* ADCS_GetState(void)
{
    return &adcs_state;
}


ADCS_Telemetry ADCS_GetTelemetry(void)
{
    ADCS_Telemetry telemetry;

    telemetry.attitude_q = adcs_state.attitude_q;
    telemetry.euler_rad = Quaternion_ToEuler321(adcs_state.attitude_q);

    telemetry.gyro_rad_s = adcs_state.gyro_rad_s;

    telemetry.sun_body_measured = adcs_state.sun_body_measured;
    telemetry.sun_body_predicted = adcs_state.sun_body_predicted;
    telemetry.sun_error = adcs_state.sun_error;

    telemetry.mag_body_measured_uT = adcs_state.mag_body_measured_uT;
    telemetry.mag_body_predicted_uT = adcs_state.mag_body_predicted_uT;
    telemetry.mag_error = adcs_state.mag_error;

    telemetry.sensor_status = adcs_state.sensor_status;

    telemetry.dt = adcs_state.dt;
    telemetry.update_counter = adcs_state.update_counter;

    return telemetry;
}