#ifndef ADS_H743_PORTING_NOTES_H
#define ADS_H743_PORTING_NOTES_H

/*
 * ads_h743_porting_notes.h
 *
 * H743 ADS implementation checklist.
 *
 * This file is documentation only.
 * Do not include this header in normal source files.
 *
 * Purpose:
 * - Record exactly what must be implemented when moving the ADS subsystem
 *   from the F3Discovery fake-sensor backend to the real STM32H743 OBC.
 * - Keep H743 implementation requirements close to the ADS source code.
 */

/*
 * ============================================================
 * 1. Build configuration
 * ============================================================
 *
 * H743 CMake compile definitions must eventually include:
 *
 *   ADS_TARGET=ADS_TARGET_H743_OBC
 *   ADS_H743_SENSOR_BACKEND_READY=1
 *
 * Keep this disabled until real sensor drivers are implemented and tested:
 *
 *   ADS_H743_SENSOR_BACKEND_READY=0
 *
 * F3Discovery bring-up build should remain:
 *
 *   ADS_TARGET=ADS_TARGET_F3DISCOVERY
 *   ADS_H743_SENSOR_BACKEND_READY=0
 */

/*
 * ============================================================
 * 2. H743 CubeMX/peripheral requirements
 * ============================================================
 *
 * The real H743 OBC project must configure peripherals for:
 *
 *   - gyro / IMU interface
 *       likely SPI or I2C, depending on selected IMU
 *
 *   - magnetometer interface
 *       likely SPI or I2C, depending on selected magnetometer
 *
 *   - photodiode / sun sensor inputs
 *       ADC channels for CubeSat face photodiodes
 *
 *   - optional debug output
 *       UART, USB CDC, SWO, or telemetry pathway
 *
 * ADS shared logic should not directly depend on H743 HAL peripheral handles.
 * Hardware-specific access should stay inside:
 *
 *   ads_h743_sensors.c
 *
 * or lower-level board support drivers called by ads_h743_sensors.c.
 */

/*
 * ============================================================
 * 3. Required H743 sensor functions
 * ============================================================
 *
 * The real H743 implementation must provide working versions of:
 *
 *   bool ADS_H743_Sensors_Init(void);
 *
 *   bool ADS_H743_ReadGyroRadS(Vector3 *gyro_rad_s);
 *
 *   bool ADS_H743_ReadMagBodyT(Vector3 *mag_body_T);
 *
 *   bool ADS_H743_ReadSunBodyUnit(Vector3 *sun_body);
 *
 * These are declared in:
 *
 *   ads_h743_sensors.h
 *
 * and stubbed in:
 *
 *   ads_h743_sensors.c
 */

/*
 * ============================================================
 * 4. Required units and conventions
 * ============================================================
 *
 * ADS_H743_ReadGyroRadS:
 *   - output units: rad/s
 *   - body-frame axes
 *   - finite values only
 *   - return false on stale/failed/invalid read
 *
 * ADS_H743_ReadMagBodyT:
 *   - output units: Tesla
 *   - body-frame axes
 *   - apply magnetometer axis mapping before returning
 *   - return false on stale/failed/invalid read
 *
 * ADS_H743_ReadSunBodyUnit:
 *   - output: unit vector in body frame
 *   - should use ADS_SunVector_FromPhotodiodes()
 *   - return false when illumination geometry is unusable
 *
 * Quaternion convention currently used by ADCS_Telemetry:
 *   - attitude_q.w
 *   - attitude_q.x
 *   - attitude_q.y
 *   - attitude_q.z
 */

/*
 * ============================================================
 * 5. Photodiode processing path
 * ============================================================
 *
 * H743 photodiode/ADC readings should be converted into:
 *
 *   ADS_PhotodiodeReadings readings;
 *
 * with fields:
 *
 *   pos_x, neg_x,
 *   pos_y, neg_y,
 *   pos_z, neg_z
 *
 * Then call:
 *
 *   ADS_SunVectorResult result =
 *       ADS_SunVector_FromPhotodiodes(readings);
 *
 * Use result.sun_body only if:
 *
 *   result.valid == true
 */

/*
 * ============================================================
 * 6. Sensor validation path
 * ============================================================
 *
 * The H743 backend should output an ADS_SensorPacket.
 *
 * Before entering the estimator, the packet is passed through:
 *
 *   ADS_SensorValidate_Apply(&sensor_packet);
 *
 * Validation checks include:
 *
 *   - known packet source
 *   - gyro finite and within ADS_SENSOR_GYRO_ABS_MAX_RAD_S
 *   - sun vector finite and roughly unit length
 *   - magnetometer finite and within Earth-field sanity limits
 *
 * Thresholds live in:
 *
 *   ads_config.h
 */

/*
 * ============================================================
 * 7. H743 backend activation
 * ============================================================
 *
 * The current H743 path is intentionally gated.
 *
 * Do not set:
 *
 *   ADS_H743_SENSOR_BACKEND_READY=1
 *
 * until:
 *
 *   - ADS_H743_Sensors_Init() returns true with real hardware
 *   - gyro read returns valid rad/s data
 *   - magnetometer read returns valid Tesla data
 *   - photodiode-to-sun-vector path returns valid unit vectors
 *   - ADS runtime health stays healthy with real sensor data
 */

/*
 * ============================================================
 * 8. Files expected to move into the H743 OBC project
 * ============================================================
 *
 * Shared ADS files:
 *
 *   ads_app.c/.h
 *   ads_task.c/.h
 *   ads_core.c/.h
 *   ads_output.c/.h
 *   ads_status.c/.h
 *   ads_runtime_check.c/.h
 *   ads_sensor_interface.c/.h
 *   ads_sensor_backend.h
 *   ads_sensor_backend_h743.c
 *   ads_sensor_validate.c/.h
 *   ads_config_check.c/.h
 *   ads_config.h
 *   ads_sun_vector.c/.h
 *   ads_h743_sensors.c/.h
 *   ads_build_info.c/.h
 *
 * F3-only files:
 *
 *   ads_fake_sensors.c/.h
 *   ads_sensor_backend_fake.c
 *   ads_status_led.c/.h
 *
 * Existing estimator/math files:
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
 * 9. Expected H743 integration flow
 * ============================================================
 *
 * 1. Copy shared ADS files into H743 project.
 * 2. Configure H743 CubeMX peripherals.
 * 3. Add lower-level sensor drivers or BSP wrappers.
 * 4. Implement ads_h743_sensors.c using those drivers.
 * 5. Build first with:
 *
 *      ADS_TARGET=ADS_TARGET_H743_OBC
 *      ADS_H743_SENSOR_BACKEND_READY=0
 *
 *    This should intentionally fail because the backend is not ready.
 *
 * 6. Once real sensor functions are implemented, set:
 *
 *      ADS_H743_SENSOR_BACKEND_READY=1
 *
 * 7. Verify:
 *
 *      ADS_App_Init() returns true
 *      ADS_App_Run() returns true during valid sensor operation
 *      ADS_App_GetHealthSummary().ads_healthy == true
 */

#endif /* ADS_H743_PORTING_NOTES_H */