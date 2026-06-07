#ifndef ADS_MODULE_MAP_H
#define ADS_MODULE_MAP_H

/*
 * ads_module_map.h
 *
 * ADS module dependency map.
 *
 * This file is documentation only.
 * Do not include this header in normal source files.
 *
 * Purpose:
 * - Record which ADS files are shared flight logic.
 * - Record which files are F3Discovery-only test support.
 * - Record which files are H743 OBC implementation support.
 * - Reduce mistakes when copying ADS code into the real H743 OBC project.
 */

/*
 * ============================================================
 * SHARED ADS APPLICATION LAYER
 * ============================================================
 *
 * These files define the public ADS subsystem API.
 * They should move to the H743 OBC project.
 *
 *   ads_app.c/.h
 *   ads_task.c/.h
 *
 * Normal external calls should go through:
 *
 *   ADS_App_Init()
 *   ADS_App_Run()
 *   ADS_App_IsHealthy()
 *   ADS_App_GetOutput()
 *   ADS_App_GetHealthSummary()
 */

/*
 * ============================================================
 * SHARED ADS CORE / ESTIMATOR WRAPPER
 * ============================================================
 *
 * These files wrap the existing estimator and expose cleaned ADS outputs.
 * They should move to the H743 OBC project.
 *
 *   ads_core.c/.h
 *   ads_output.c/.h
 *
 * Existing estimator/math modules also move:
 *
 *   adcs.c/.h
 *   adcs_health.c/.h
 *   calibration.c/.h
 *   quest.c/.h
 *   vector3.c/.h
 *   quaternion.c/.h
 *   matrix_utils.c/.h
 */

/*
 * ============================================================
 * SHARED ADS SENSOR INTERFACE
 * ============================================================
 *
 * These files define the backend-independent ADS sensor input path.
 * They should move to the H743 OBC project.
 *
 *   ads_sensor_interface.c/.h
 *   ads_sensor_backend.h
 *   ads_sensor_validate.c/.h
 *
 * ads_task.c should only see ADS_SensorPacket and should not care whether
 * data comes from fake F3 sensors or real H743 sensors.
 */

/*
 * ============================================================
 * F3DISCOVERY-ONLY SENSOR BACKEND
 * ============================================================
 *
 * These files are for F3Discovery fake-sensor testing only.
 * Do not use these as the real H743 backend.
 *
 *   ads_fake_sensors.c/.h
 *   ads_sensor_backend_fake.c
 *
 * These are selected when:
 *
 *   ADS_TARGET=ADS_TARGET_F3DISCOVERY
 */

/*
 * ============================================================
 * H743 OBC SENSOR BACKEND
 * ============================================================
 *
 * These files define the real OBC sensor integration point.
 * These should move to the H743 OBC project.
 *
 *   ads_sensor_backend_h743.c
 *   ads_h743_sensors.c/.h
 *
 * These are selected when:
 *
 *   ADS_TARGET=ADS_TARGET_H743_OBC
 *
 * The H743 backend must not be enabled until real sensor drivers are wired in:
 *
 *   ADS_H743_SENSOR_BACKEND_READY=1
 */

/*
 * ============================================================
 * SHARED ADS HEALTH / DIAGNOSTIC MODULES
 * ============================================================
 *
 * These files are hardware-free and useful for H743 telemetry/state-machine
 * diagnostics. They should move to the H743 OBC project.
 *
 *   ads_status.c/.h
 *   ads_runtime_check.c/.h
 *   ads_timing.c/.h
 *   ads_config_check.c/.h
 *   ads_build_info.c/.h
 */

/*
 * ============================================================
 * SHARED SUN VECTOR PROCESSING
 * ============================================================
 *
 * These files convert photodiode readings into a body-frame sun vector.
 * They should move to the H743 OBC project.
 *
 *   ads_sun_vector.c/.h
 */

/*
 * ============================================================
 * F3DISCOVERY-ONLY DEBUG LED
 * ============================================================
 *
 * This file is board-debug support for the F3Discovery.
 * It should not be required for the H743 OBC ADS logic.
 *
 *   ads_status_led.c/.h
 *
 * Current F3 mapping:
 *
 *   LD3 red LED = PE9
 *
 * The H743 OBC may either:
 *
 *   - omit this module,
 *   - leave it as no-op,
 *   - or replace it with H743 board-specific status indication.
 */

/*
 * ============================================================
 * FAULT INJECTION
 * ============================================================
 *
 * Fault injection is useful for F3 bench testing.
 *
 *   ads_fault_injection.c/.h
 *
 * Default configuration:
 *
 *   F3Discovery: enabled
 *   H743 OBC: disabled
 *
 * Controlled by:
 *
 *   ADS_FAULT_INJECTION_ENABLE
 */

/*
 * ============================================================
 * CONFIGURATION
 * ============================================================
 *
 * Central ADS configuration lives in:
 *
 *   ads_config.h
 *
 * F3 CMake should define:
 *
 *   ADS_TARGET=ADS_TARGET_F3DISCOVERY
 *   ADS_H743_SENSOR_BACKEND_READY=0
 *
 * H743 CMake should eventually define:
 *
 *   ADS_TARGET=ADS_TARGET_H743_OBC
 *   ADS_H743_SENSOR_BACKEND_READY=1
 */

/*
 * ============================================================
 * CURRENT F3 MAIN.C ROLE
 * ============================================================
 *
 * The F3Discovery main.c should remain a test harness only:
 *
 *   - CubeMX init
 *   - non-blocking 100 Hz scheduler
 *   - ADS_App_Init()
 *   - ADS_App_Run()
 *   - LED heartbeat/status indication
 *
 * It should not contain estimator logic, sensor simulation logic, or H743
 * hardware-specific ADS code.
 */

#endif /* ADS_MODULE_MAP_H */