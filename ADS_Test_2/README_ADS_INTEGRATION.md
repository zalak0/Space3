# ADS QUEST + EKF integration notes

This project has been updated so the existing ICM20948 I2C polling path feeds the uploaded QUEST and EKF attitude-determination code.

## Added source modules

- `Core/Inc/vector3.h`, `Core/Src/vector3.c`  
  Missing vector math support required by QUEST, quaternion, matrix, and EKF code.

- `Core/Inc/matrix_utils.h`, `Core/Src/matrix_utils.c`  
  Uploaded 3x3 matrix utilities.

- `Core/Inc/quaternion.h`, `Core/Src/quaternion.c`  
  Uploaded quaternion utilities and Euler 3-2-1 conversion.

- `Core/Inc/quest.h`, `Core/Src/quest.c`  
  Uploaded QUEST / Davenport q-method solver.

- `Core/Inc/ads_ekf.h`, `Core/Src/ads_ekf.c`  
  Uploaded EKF estimator. One API function was added: `ADS_EKF_ResetWithAttitude()`, used to seed the EKF from QUEST.

- `Core/Inc/ads_process.h`, `Core/Src/ads_process.c`  
  New wrapper layer connecting `ICM20948_Data` to QUEST/EKF input/output packaging.

## Main loop integration

`main.c` now:

1. Initializes the ICM20948 exactly as before.
2. Creates default ADS reference vectors using `ADS_Process_DefaultConfig()`.
3. Calls `ADS_Process_Init()` once.
4. On every successful `ICM20948_Read()`, calls `ADS_Process_Update()`.
5. Exposes debugger-friendly watch variables:
   - `imu_data`
   - `ads_output`
   - `ads_ekf_state`
   - `ads_q`
   - `ads_euler`

## Important reference-vector note

The default ADS config is suitable for bench bring-up only:

- gravity/reference accelerometer vector: `{0, 0, 9.80665}` m/s²
- magnetic field reference placeholder: `{25e-6, 0, -43e-6}` T
- sample period: `0.10` s

For flight/mission use, replace these defaults with the real LVLH/orbit propagated gravity and magnetic reference vectors before treating the attitude estimate as flight truth.

## Build note

The old generated `Debug/` build directory was intentionally removed from the package so CubeIDE regenerates fresh makefiles including the new ADS source files. Import/open the project in STM32CubeIDE, refresh the project, then build Debug normally.
