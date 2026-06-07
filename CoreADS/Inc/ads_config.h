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
 * These constants are used by the F3Discovery ADS test harness.
 * Later, the H743 OBC target can keep this file, replace values here,
 * or move mission-level constants into a higher-level flight config file.
 */

/* ADS target/backend selection */
#define ADS_TARGET_F3DISCOVERY                 (1)
#define ADS_TARGET_H743_OBC                    (2)

#ifndef ADS_TARGET
#warning "ADS_TARGET not defined by build system. Defaulting to ADS_TARGET_F3DISCOVERY."
#define ADS_TARGET                             ADS_TARGET_F3DISCOVERY
#endif

/*
 * H743 backend readiness gate.
 *
 * Keep this at 0 until ads_sensor_backend_h743.c has real sensor drivers wired in.
 *
 * If ADS_TARGET is changed to ADS_TARGET_H743_OBC while this is still 0,
 * the build will intentionally fail. This prevents accidentally building
 * real OBC firmware with the safe invalid stub/backend.
 *
 * This is intentionally overridable from CMake:
 *
 *   ADS_H743_SENSOR_BACKEND_READY=1
 *
 * but only do that once real H743 sensor drivers are implemented.
 */
#ifndef ADS_H743_SENSOR_BACKEND_READY
#define ADS_H743_SENSOR_BACKEND_READY          (0)
#endif

/*
 * Fault injection enable.
 *
 * F3Discovery:
 * - enabled by default for bench testing.
 *
 * H743 OBC:
 * - disabled by default so test faults cannot be accidentally enabled
 *   in real OBC builds.
 *
 * This can still be overridden from CMake if deliberately required.
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

/* Fake gyro/sensor motion */
#define ADS_FAKE_ROTATION_RATE_RAD_S           (0.05f)

/* Fake magnetic field, Tesla */
#define ADS_FAKE_MAG_FIELD_X_T                 (35.0e-6f)
#define ADS_FAKE_MAG_FIELD_Y_T                 (0.0f)
#define ADS_FAKE_MAG_FIELD_Z_T                 (5.0e-6f)

/* Runtime/output quaternion sanity bounds */
#define ADS_QUATERNION_NORM_MIN                (0.90f)
#define ADS_QUATERNION_NORM_MAX                (1.10f)

/* Sensor packet validation thresholds */
#define ADS_SENSOR_GYRO_ABS_MAX_RAD_S          (20.0f)

#define ADS_SENSOR_SUN_NORM_MIN                (0.80f)
#define ADS_SENSOR_SUN_NORM_MAX                (1.20f)

#define ADS_SENSOR_MAG_NORM_MIN_T              (1.0e-6f)
#define ADS_SENSOR_MAG_NORM_MAX_T              (200.0e-6f)

/*
 * Photodiode-to-sun-vector threshold.
 *
 * Unitless for now because it may be raw ADC counts or calibrated relative
 * face illumination depending on the H743 implementation.
 */
#define ADS_SUN_VECTOR_MIN_TOTAL_SIGNAL        (1.0e-6f)

/* F3Discovery ADS status LED timing, in ADS task ticks */
#define ADS_STATUS_LED_HEALTHY_TOGGLE_TICKS    (100u)
#define ADS_STATUS_LED_FAULT_TOGGLE_TICKS      (10u)


/*
 * Compile-time configuration sanity checks.
 *
 * Only integer-valued macros are checked here.
 * Float-valued macros are checked at runtime where needed, because the C
 * preprocessor cannot evaluate floating-point constants in #if expressions.
 */

#if (ADS_TARGET != ADS_TARGET_F3DISCOVERY) && (ADS_TARGET != ADS_TARGET_H743_OBC)
#error "Invalid ADS_TARGET. Use ADS_TARGET_F3DISCOVERY or ADS_TARGET_H743_OBC."
#endif

#if (ADS_H743_SENSOR_BACKEND_READY != 0) && (ADS_H743_SENSOR_BACKEND_READY != 1)
#error "ADS_H743_SENSOR_BACKEND_READY must be 0 or 1."
#endif

#if (ADS_FAULT_INJECTION_ENABLE != 0) && (ADS_FAULT_INJECTION_ENABLE != 1)
#error "ADS_FAULT_INJECTION_ENABLE must be 0 or 1."
#endif

#if (ADS_TARGET == ADS_TARGET_F3DISCOVERY) && (ADS_H743_SENSOR_BACKEND_READY != 0)
#error "F3Discovery build must not enable ADS_H743_SENSOR_BACKEND_READY."
#endif

#if (ADS_TARGET == ADS_TARGET_H743_OBC)
  #if (ADS_H743_SENSOR_BACKEND_READY == 0)
    #if !defined(ADS_ALLOW_FAKE_BACKEND_ON_H743) || (ADS_ALLOW_FAKE_BACKEND_ON_H743 == 0)
      #error "H743 OBC build selected but ADS_H743_SENSOR_BACKEND_READY is 0."
    #endif
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


/* --------------------------------------------------------------------------
 * ADS backend selection sanity checks
 *
 * Valid current bring-up configuration:
 *   ADS_TARGET=ADS_TARGET_H743_OBC
 *   ADS_H743_SENSOR_BACKEND_READY=0
 *   ADS_ALLOW_FAKE_BACKEND_ON_H743=1
 *
 * This allows the real H743 MCU to run the fake ADS backend while hardware
 * sensor drivers are still being developed.
 *
 * Photodiode inputs are intentionally not part of the ADS path.
 * -------------------------------------------------------------------------- */

#ifndef ADS_TARGET
#error "ADS_TARGET is not defined."
#endif

#ifndef ADS_H743_SENSOR_BACKEND_READY
#error "ADS_H743_SENSOR_BACKEND_READY is not defined."
#endif

#if (ADS_TARGET != ADS_TARGET_F3DISCOVERY) && \
    (ADS_TARGET != ADS_TARGET_H743_OBC)
#error "Invalid ADS_TARGET selection."
#endif

#if (ADS_TARGET == ADS_TARGET_F3DISCOVERY) && \
    (ADS_H743_SENSOR_BACKEND_READY != 0)
#error "F3Discovery build must not enable the H743 sensor backend."
#endif

#if (ADS_TARGET == ADS_TARGET_H743_OBC) && \
    (ADS_H743_SENSOR_BACKEND_READY == 0)

    #ifndef ADS_ALLOW_FAKE_BACKEND_ON_H743
    #error "H743 fake-backend bring-up requires ADS_ALLOW_FAKE_BACKEND_ON_H743=1."
    #endif

    #if (ADS_ALLOW_FAKE_BACKEND_ON_H743 != 1)
    #error "H743 fake-backend bring-up requires ADS_ALLOW_FAKE_BACKEND_ON_H743=1."
    #endif

#endif

#if (ADS_TARGET == ADS_TARGET_H743_OBC) && \
    (ADS_H743_SENSOR_BACKEND_READY == 1)

    #ifdef ADS_ALLOW_FAKE_BACKEND_ON_H743
        #if (ADS_ALLOW_FAKE_BACKEND_ON_H743 == 1)
        #error "Real H743 backend selected, but fake backend override is still enabled."
        #endif
    #endif

#endif

#endif /* ADS_CONFIG_H */