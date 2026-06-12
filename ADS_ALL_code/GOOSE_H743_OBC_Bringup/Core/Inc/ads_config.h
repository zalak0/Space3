#ifndef ADS_CONFIG_H
#define ADS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ads_config.h
 *
 * Central ADS bring-up configuration.
 *
 * Current target:
 * - STM32H743 OBC
 * - ICM20948 accel/gyro backend over I2C
 * - no photodiode/sun vector dependency
 * - magnetometer optional until AK09916/readout path is implemented
 */

/* ADS target/backend selection */
#define ADS_TARGET_F3DISCOVERY                 (1)
#define ADS_TARGET_H743_OBC                    (2)

/*
 * Default to H743 for this final OBC bring-up branch.
 * This can still be overridden from CMake if needed.
 */
#ifndef ADS_TARGET
#define ADS_TARGET                             ADS_TARGET_H743_OBC
#endif

/*
 * H743 backend readiness gate.
 *
 * Set to 1 because the H743 backend now has a real ICM20948 accel/gyro path.
 * Magnetometer may still be invalid at runtime; that is allowed during this
 * bring-up phase.
 */
#ifndef ADS_H743_SENSOR_BACKEND_READY
#define ADS_H743_SENSOR_BACKEND_READY          (1)
#endif

/*
 * Fake backend override.
 *
 * Must stay 0 for real H743 OBC firmware.
 */
#ifndef ADS_ALLOW_FAKE_BACKEND_ON_H743
#define ADS_ALLOW_FAKE_BACKEND_ON_H743         (0)
#endif

/*
 * Fault injection.
 *
 * Disabled by default on H743 so deliberate test faults cannot accidentally
 * run in OBC firmware.
 */
#ifndef ADS_FAULT_INJECTION_ENABLE
#if ADS_TARGET == ADS_TARGET_F3DISCOVERY
#define ADS_FAULT_INJECTION_ENABLE             (1)
#else
#define ADS_FAULT_INJECTION_ENABLE             (0)
#endif
#endif

/* ADS task timing */
#define ADS_TASK_PERIOD_MS                     (10u)
#define ADS_TASK_DT_S                          (0.01f)
#define ADS_TASK_DT_TOLERANCE_S                (0.001f)

/* Fake gyro/sensor motion, retained only for non-H743 testing */
#define ADS_FAKE_ROTATION_RATE_RAD_S           (0.05f)

/* Fake magnetic field, Tesla, retained only for non-H743 testing */
#define ADS_FAKE_MAG_FIELD_X_T                 (35.0e-6f)
#define ADS_FAKE_MAG_FIELD_Y_T                 (0.0f)
#define ADS_FAKE_MAG_FIELD_Z_T                 (5.0e-6f)

/* Runtime/output quaternion sanity bounds */
#define ADS_QUATERNION_NORM_MIN                (0.90f)
#define ADS_QUATERNION_NORM_MAX                (1.10f)

/* Sensor packet validation thresholds */
#define ADS_SENSOR_GYRO_ABS_MAX_RAD_S          (20.0f)

/* Accelerometer gravity-vector sanity bounds for bench testing, m/s^2. */
#define ADS_SENSOR_ACCEL_NORM_MIN_M_S2         (5.0f)
#define ADS_SENSOR_ACCEL_NORM_MAX_M_S2         (15.0f)

/*
 * Sun vector bounds are retained for optional future vector use.
 * Sun/photodiode validity is not required for current ADS health.
 */
#define ADS_SENSOR_SUN_NORM_MIN                (0.80f)
#define ADS_SENSOR_SUN_NORM_MAX                (1.20f)

/*
 * Magnetometer bounds.
 * Mag validity is optional during current H743 gyro-only bring-up.
 */
#define ADS_SENSOR_MAG_NORM_MIN_T              (1.0e-6f)
#define ADS_SENSOR_MAG_NORM_MAX_T              (200.0e-6f)

/*
 * Photodiode-to-sun-vector threshold.
 * Retained only for optional future sun-vector path.
 */
#define ADS_SUN_VECTOR_MIN_TOTAL_SIGNAL        (1.0e-6f)

/* Status LED timing, in ADS task ticks */
#define ADS_STATUS_LED_HEALTHY_TOGGLE_TICKS    (100u)
#define ADS_STATUS_LED_FAULT_TOGGLE_TICKS      (10u)


/* --------------------------------------------------------------------------
 * Compile-time configuration sanity checks
 * -------------------------------------------------------------------------- */

#if (ADS_TARGET != ADS_TARGET_F3DISCOVERY) && (ADS_TARGET != ADS_TARGET_H743_OBC)
#error "Invalid ADS_TARGET. Use ADS_TARGET_F3DISCOVERY or ADS_TARGET_H743_OBC."
#endif

#if (ADS_H743_SENSOR_BACKEND_READY != 0) && (ADS_H743_SENSOR_BACKEND_READY != 1)
#error "ADS_H743_SENSOR_BACKEND_READY must be 0 or 1."
#endif

#if (ADS_ALLOW_FAKE_BACKEND_ON_H743 != 0) && (ADS_ALLOW_FAKE_BACKEND_ON_H743 != 1)
#error "ADS_ALLOW_FAKE_BACKEND_ON_H743 must be 0 or 1."
#endif

#if (ADS_FAULT_INJECTION_ENABLE != 0) && (ADS_FAULT_INJECTION_ENABLE != 1)
#error "ADS_FAULT_INJECTION_ENABLE must be 0 or 1."
#endif

#if (ADS_TARGET == ADS_TARGET_F3DISCOVERY) && (ADS_H743_SENSOR_BACKEND_READY != 0)
#error "F3Discovery build must not enable ADS_H743_SENSOR_BACKEND_READY."
#endif

#if (ADS_TARGET == ADS_TARGET_H743_OBC) && (ADS_H743_SENSOR_BACKEND_READY == 0)
  #if (ADS_ALLOW_FAKE_BACKEND_ON_H743 == 0)
    #error "H743 OBC build selected but ADS_H743_SENSOR_BACKEND_READY is 0."
  #endif
#endif

#if (ADS_TARGET == ADS_TARGET_H743_OBC) && (ADS_H743_SENSOR_BACKEND_READY == 1)
  #if (ADS_ALLOW_FAKE_BACKEND_ON_H743 == 1)
    #error "Real H743 backend selected, but ADS_ALLOW_FAKE_BACKEND_ON_H743 is still enabled."
  #endif
#endif

#if ADS_TASK_PERIOD_MS == 0u
#error "ADS_TASK_PERIOD_MS must be non-zero."
#endif

#if ADS_STATUS_LED_HEALTHY_TOGGLE_TICKS == 0u
#error "ADS_STATUS_LED_HEALTHY_TOGGLE_TICKS must be non-zero."
#endif

#if ADS_STATUS_LED_FAULT_TOGGLE_TICKS == 0u
#error "ADS_STATUS_LED_FAULT_TOGGLE_TICKS must be non-zero."
#endif

#ifdef __cplusplus
}
#endif

#endif /* ADS_CONFIG_H */