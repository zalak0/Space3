#ifndef ADS_PROCESS_H
#define ADS_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ads_ekf.h"
#include "icm20948.h"
#include "quest.h"
#include "quaternion.h"
#include "vector3.h"

typedef struct
{
    /*
     * Reference vectors expressed in the estimator inertial/test frame.
     * Replace these with LVLH/orbit-propagated references when available.
     */
    Vector3 gravity_reference_m_s2;
    Vector3 magnetic_reference_T;

    /* Nominal time step used by the current polling loop. */
    float sample_period_s;

} ADS_Process_Config;

typedef struct
{
    bool initialized;
    bool imu_valid;
    bool accel_used;
    bool mag_used;
    bool quest_valid;
    bool ekf_valid;

    uint32_t sample_count;

    Quaternion quest_q_body_to_inertial;
    float quest_residual;

    ADS_EKF_State ekf_state;
    EulerAngles euler_rad;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

} ADS_Process_Output;

ADS_Process_Config ADS_Process_DefaultConfig(void);
void ADS_Process_Init(const ADS_Process_Config* config);
bool ADS_Process_Update(const ICM20948_Data* imu, ADS_Process_Output* out);
ADS_Process_Output ADS_Process_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_PROCESS_H */
