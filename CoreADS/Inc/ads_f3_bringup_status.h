#ifndef ADS_F3_BRINGUP_STATUS_H
#define ADS_F3_BRINGUP_STATUS_H

/*
 * ads_f3_bringup_status.h
 *
 * F3Discovery ADS bring-up status record.
 *
 * This file is documentation only.
 * Do not include this header in normal source files.
 *
 * Purpose:
 * - Record what has been proven on the STM32F3Discovery test board.
 * - Make clear what is still fake/stubbed before moving to H743.
 */

/*
 * ============================================================
 * Confirmed F3Discovery bring-up status
 * ============================================================
 *
 * Board:
 *   STM32F3Discovery
 *   STM32F303VCT6
 *
 * Current role:
 *   Safe ADS software test harness before real STM32H743 OBC upload.
 *
 * Confirmed working:
 *   - CubeMX/CMake/VS Code build
 *   - ELF flashing through STM32CubeProgrammer
 *   - non-blocking HAL_GetTick() scheduler
 *   - 100 Hz ADS task period
 *   - LD6 green heartbeat
 *   - LD3 red ADS health/status indication
 *   - ADS app/task/core modular structure
 *   - fake sensor backend
 *   - sensor packet validation
 *   - ADCS estimator wrapper
 *   - ADS output publication
 *   - runtime health checks
 *   - config checks
 *   - timing checks
 *   - fault injection test path
 *   - healthy path gives slow LD3 blink
 *   - intentional fault path gives fast LD3 blink
 */

/*
 * ============================================================
 * Current F3 build definitions
 * ============================================================
 *
 * F3 CMake should define:
 *
 *   ADS_TARGET=ADS_TARGET_F3DISCOVERY
 *   ADS_H743_SENSOR_BACKEND_READY=0
 *
 * Current expected build info:
 *
 *   ADS_App_GetBuildTargetName()      -> "F3DISCOVERY"
 *   ADS_App_GetSensorSourceName()     -> "F3_FAKE"
 *   ADS_App_GetLastFaultReasonName()  -> "NONE" during healthy operation
 *
 * Fault injection:
 *
 *   ADS_FAULT_INJECTION_ENABLE defaults to 1 for F3Discovery.
 */

/*
 * ============================================================
 * Current fake sensor behaviour
 * ============================================================
 *
 * Fake sensor source:
 *
 *   ads_fake_sensors.c
 *   ads_sensor_backend_fake.c
 *
 * Fake packet content:
 *
 *   gyro_rad_s:
 *     slowly rotating/fixed test rate
 *
 *   sun_body:
 *     unit vector rotating slowly in body frame
 *
 *   mag_body_T:
 *     Earth-field-order magnetic vector in Tesla
 *
 *   status:
 *     gyro_valid = 1
 *     sun_valid  = 1
 *     mag_valid  = 1
 */

/*
 * ============================================================
 * Current H743 status
 * ============================================================
 *
 * H743 real sensor backend is not implemented yet.
 *
 * Existing H743 implementation points:
 *
 *   ads_sensor_backend_h743.c
 *   ads_h743_sensors.c/.h
 *
 * Required real H743 functions:
 *
 *   ADS_H743_Sensors_Init()
 *   ADS_H743_ReadGyroRadS()
 *   ADS_H743_ReadMagBodyT()
 *   ADS_H743_ReadSunBodyUnit()
 *
 * H743 backend must remain gated until these are real:
 *
 *   ADS_H743_SENSOR_BACKEND_READY=0
 */

/*
 * ============================================================
 * Before moving to H743
 * ============================================================
 *
 * Final F3-side checks:
 *
 *   1. Build has no warnings worth fixing.
 *   2. main.c only calls ADS_App_Init(), ADS_App_Run(), and status LED helper.
 *   3. mode_manager.c is not required by ADS build.
 *   4. ACS/magnetorquer code is not required by ADS build.
 *   5. F3-specific HAL usage is isolated to F3 test/debug files.
 *   6. H743 hardware integration points are isolated to ads_h743_sensors.c.
 */

#endif /* ADS_F3_BRINGUP_STATUS_H */