/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads_app.h"
#include <stdint.h>
#include "ads_status.h"
#include "ads_icm20948_probe.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ICM20948_ACCEL_4G_LSB_PER_G        (8192.0f)
#define ICM20948_STANDARD_GRAVITY_MPS2     (9.80665f)

#define ICM20948_GYRO_500DPS_LSB_PER_DPS   (65.5f)
#define DEG_TO_RAD                         (0.01745329251994329577f)

#define ICM20948_REG_BANK_SEL             0x7Fu
#define ICM20948_BANK0                    0x00u
#define ICM20948_USER_CTRL                0x03u
#define ICM20948_INT_PIN_CFG              0x0Fu

#define ICM20948_I2C_BYPASS_EN            0x02u

#define AK09916_I2C_ADDR_7BIT             0x0Cu
#define AK09916_I2C_ADDR_HAL              (AK09916_I2C_ADDR_7BIT << 1)

#define AK09916_REG_WIA2                  0x01u
#define AK09916_REG_ST1                   0x10u
#define AK09916_REG_HXL                   0x11u
#define AK09916_REG_ST2                   0x18u
#define AK09916_REG_CNTL2                 0x31u
#define AK09916_REG_CNTL3                 0x32u

#define AK09916_WIA2_EXPECTED             0x09u

#define AK09916_CNTL2_POWER_DOWN          0x00u
#define AK09916_CNTL2_CONTINUOUS_10HZ     0x02u
#define AK09916_CNTL2_CONTINUOUS_20HZ     0x04u
#define AK09916_CNTL2_CONTINUOUS_50HZ     0x06u
#define AK09916_CNTL2_CONTINUOUS_100HZ    0x08u

#define AK09916_ST1_DRDY                  0x01u
#define AK09916_ST2_HOFL                  0x08u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile uint32_t g_uart4_test_status_debug = 999u;

static uint32_t g_ads_last_run_ms = 0u;
static uint32_t g_ads_status_last_toggle_ms = 0u;

static uint32_t g_imu_csv_start_ms = 0u;
static uint32_t g_imu_csv_last_ms = 0u;
static uint32_t g_imu_csv_sample = 0u;

#if 0
static uint32_t g_gyro50_start_ms = 0u;
static uint32_t g_gyro50_last_ms = 0u;
static uint32_t g_gyro50_sample = 0u;
static uint32_t g_uart_telemetry_last_ms = 0u;

static uint32_t g_uart_telemetry_sample_count = 0u;
static uint32_t g_uart_telemetry_last_sample_ms = 0u;
#endif


/*
 * Prepared for later 50 Hz gyro telemetry.
 * Leave disabled for now while we finish clean accel/gyro telemetry.
 */
#define ADS_UART_TELEMETRY_50HZ_ENABLE 0

/* Debug-visible counters */
volatile uint32_t g_ads_run_count = 0u;
volatile uint32_t g_ads_init_done = 0u;
volatile uint32_t g_ads_healthy_debug = 0u;


volatile uint32_t g_icm20948_probe_result_debug = 0u;
volatile uint32_t g_icm20948_detected_addr_debug = 0u;
volatile uint32_t g_icm20948_probe_count_debug = 0u;
volatile uint32_t g_icm20948_hal_status_0x69_debug = 0u;
volatile uint32_t g_icm20948_hal_status_0x68_debug = 0u;

volatile uint32_t g_icm20948_whoami_result_debug = 0u;
volatile uint32_t g_icm20948_whoami_value_debug = 0u;
volatile uint32_t g_icm20948_whoami_hal_status_debug = 0u;
volatile uint32_t g_icm20948_whoami_read_count_debug = 0u;

static uint8_t g_icm20948_probe_done = 0u;
static uint8_t g_icm20948_probe_found = 0u;
static uint8_t g_icm20948_whoami_ok = 0u;

volatile uint32_t g_icm20948_wake_result_debug = 0u;
volatile uint32_t g_icm20948_wake_hal_status_debug = 0u;
volatile uint32_t g_icm20948_wake_count_debug = 0u;

volatile int32_t g_icm20948_accel_x_raw_debug = 0;
volatile int32_t g_icm20948_accel_y_raw_debug = 0;
volatile int32_t g_icm20948_accel_z_raw_debug = 0;

volatile int32_t g_icm20948_gyro_x_raw_debug = 0;
volatile int32_t g_icm20948_gyro_y_raw_debug = 0;
volatile int32_t g_icm20948_gyro_z_raw_debug = 0;

volatile uint32_t g_icm20948_raw_result_debug = 0u;
volatile uint32_t g_icm20948_raw_hal_status_debug = 0u;
volatile uint32_t g_icm20948_raw_read_count_debug = 0u;

static uint8_t g_icm20948_raw_ok = 0u;
static uint32_t g_icm20948_last_raw_read_ms = 0u;

volatile uint32_t g_icm20948_config_result_debug = 0u;
volatile uint32_t g_icm20948_config_hal_status_debug = 0u;
volatile uint32_t g_icm20948_config_count_debug = 0u;

static uint8_t g_icm20948_config_ok = 0u;

volatile float g_icm20948_accel_x_mps2_debug = 0.0f;
volatile float g_icm20948_accel_y_mps2_debug = 0.0f;
volatile float g_icm20948_accel_z_mps2_debug = 0.0f;

volatile float g_icm20948_gyro_x_radps_debug = 0.0f;
volatile float g_icm20948_gyro_y_radps_debug = 0.0f;
volatile float g_icm20948_gyro_z_radps_debug = 0.0f;

static uint8_t g_ak09916_bypass_ok = 0u;
static uint8_t g_ak09916_whoami_ok = 0u;
static uint8_t g_ak09916_config_ok = 0u;
static uint8_t g_ak09916_raw_ok = 0u;

static uint32_t g_ak09916_last_read_ms = 0u;
static uint32_t g_mag_csv_start_ms = 0u;
static uint32_t g_mag_csv_last_ms = 0u;
static uint32_t g_mag_csv_sample = 0u;

static uint32_t g_ak09916_hal_status_debug = 0u;
static uint32_t g_ak09916_wia2_debug = 0u;
static uint32_t g_ak09916_st1_debug = 0u;
static uint32_t g_ak09916_st2_debug = 0u;

static int32_t g_ak09916_mag_x_raw_debug = 0;
static int32_t g_ak09916_mag_y_raw_debug = 0;
static int32_t g_ak09916_mag_z_raw_debug = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
#if 0
static void MX_UART4_Init(void);
#endif
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef GOOSE_ICM20948_WriteReg(uint8_t icm_addr_7bit,
                                                 uint8_t reg,
                                                 uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c2,
                           (uint16_t)(icm_addr_7bit << 1),
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1u,
                           100u);
}

static HAL_StatusTypeDef GOOSE_AK09916_WriteReg(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c2,
                           AK09916_I2C_ADDR_HAL,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1u,
                           100u);
}

static HAL_StatusTypeDef GOOSE_AK09916_ReadReg(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c2,
                          AK09916_I2C_ADDR_HAL,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          value,
                          1u,
                          100u);
}

static HAL_StatusTypeDef GOOSE_AK09916_ReadRegs(uint8_t reg,
                                                uint8_t *data,
                                                uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c2,
                          AK09916_I2C_ADDR_HAL,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          len,
                          100u);
}

static void GOOSE_AK09916_InitForTelemetry(void)
{
  HAL_StatusTypeDef status;
  uint8_t wia2 = 0u;

  g_ak09916_bypass_ok = 0u;
  g_ak09916_whoami_ok = 0u;
  g_ak09916_config_ok = 0u;
  g_ak09916_raw_ok = 0u;

  /*
   * Put ICM-20948 in bank 0.
   */
  status = GOOSE_ICM20948_WriteReg((uint8_t)g_icm20948_detected_addr_debug,
                                   ICM20948_REG_BANK_SEL,
                                   ICM20948_BANK0);
  g_ak09916_hal_status_debug = (uint32_t)status;

  if (status != HAL_OK)
  {
    return;
  }

  /*
   * Disable ICM internal I2C master, then enable bypass so STM32 can see AK09916.
   */
  status = GOOSE_ICM20948_WriteReg((uint8_t)g_icm20948_detected_addr_debug,
                                   ICM20948_USER_CTRL,
                                   0x00u);
  g_ak09916_hal_status_debug = (uint32_t)status;

  if (status != HAL_OK)
  {
    return;
  }

  status = GOOSE_ICM20948_WriteReg((uint8_t)g_icm20948_detected_addr_debug,
                                   ICM20948_INT_PIN_CFG,
                                   ICM20948_I2C_BYPASS_EN);
  g_ak09916_hal_status_debug = (uint32_t)status;

  if (status != HAL_OK)
  {
    return;
  }

  HAL_Delay(10u);
  g_ak09916_bypass_ok = 1u;

  /*
   * Check AK09916 identity.
   */
  status = GOOSE_AK09916_ReadReg(AK09916_REG_WIA2, &wia2);
  g_ak09916_hal_status_debug = (uint32_t)status;
  g_ak09916_wia2_debug = (uint32_t)wia2;

  if ((status != HAL_OK) || (wia2 != AK09916_WIA2_EXPECTED))
  {
    return;
  }

  g_ak09916_whoami_ok = 1u;

  /*
   * Reset/configure magnetometer.
   */
  status = GOOSE_AK09916_WriteReg(AK09916_REG_CNTL3, 0x01u);
  g_ak09916_hal_status_debug = (uint32_t)status;

  if (status != HAL_OK)
  {
    return;
  }

  HAL_Delay(50u);

  status = GOOSE_AK09916_WriteReg(AK09916_REG_CNTL2,
                                  AK09916_CNTL2_CONTINUOUS_100HZ);
  g_ak09916_hal_status_debug = (uint32_t)status;

  if (status != HAL_OK)
  {
    return;
  }

  HAL_Delay(10u);

  g_ak09916_config_ok = 1u;
}

static void GOOSE_AK09916_ReadMagOnce(void)
{
  HAL_StatusTypeDef status;
  uint8_t st1 = 0u;
  uint8_t data[8] = {0};

  g_ak09916_raw_ok = 0u;

  status = GOOSE_AK09916_ReadReg(AK09916_REG_ST1, &st1);
  g_ak09916_hal_status_debug = (uint32_t)status;
  g_ak09916_st1_debug = (uint32_t)st1;

  if (status != HAL_OK)
  {
    return;
  }

  if ((st1 & AK09916_ST1_DRDY) == 0u)
  {
    return;
  }

  /*
   * Read HXL,HXH,HYL,HYH,HZL,HZH,TMPS,ST2.
   * Reading ST2 is required to complete the measurement read.
   */
  status = GOOSE_AK09916_ReadRegs(AK09916_REG_HXL, data, 8u);
  g_ak09916_hal_status_debug = (uint32_t)status;

  if (status != HAL_OK)
  {
    return;
  }

  g_ak09916_st2_debug = (uint32_t)data[7];

  if ((data[7] & AK09916_ST2_HOFL) != 0u)
  {
    return;
  }

  g_ak09916_mag_x_raw_debug =
      (int32_t)((int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8)));

  g_ak09916_mag_y_raw_debug =
      (int32_t)((int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8)));

  g_ak09916_mag_z_raw_debug =
      (int32_t)((int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8)));

  g_ak09916_raw_ok = 1u;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  g_uart4_test_status_debug = 999u;

  ADS_App_Init();
  g_ads_init_done = 1u;

  ADS_ICM20948_ProbeDiagnostics icm_probe_diag;

  icm_probe_diag = ADS_ICM20948_Probe_I2C(&hi2c2);

  g_icm20948_probe_result_debug = (uint32_t)icm_probe_diag.result;
  g_icm20948_detected_addr_debug = (uint32_t)icm_probe_diag.detected_address_7bit;
  g_icm20948_probe_count_debug = icm_probe_diag.probe_count;
  g_icm20948_hal_status_0x69_debug = (uint32_t)icm_probe_diag.hal_status_0x69;
  g_icm20948_hal_status_0x68_debug = (uint32_t)icm_probe_diag.hal_status_0x68;

  g_icm20948_probe_done = 1u;

  if ((icm_probe_diag.result == ADS_ICM20948_PROBE_FOUND_0X69) ||
      (icm_probe_diag.result == ADS_ICM20948_PROBE_FOUND_0X68))
  {
    g_icm20948_probe_found = 1u;
  }
  else
  {
    g_icm20948_probe_found = 0u;
  }

  if (icm_probe_diag.detected_address_7bit != 0u)
  {
    ADS_ICM20948_WhoAmIDiagnostics whoami_diag;

    whoami_diag = ADS_ICM20948_ReadWhoAmI(
        &hi2c2,
        icm_probe_diag.detected_address_7bit
    );

    g_icm20948_whoami_result_debug = (uint32_t)whoami_diag.result;
    g_icm20948_whoami_value_debug = (uint32_t)whoami_diag.who_am_i_value;
    g_icm20948_whoami_hal_status_debug = (uint32_t)whoami_diag.hal_status;
    g_icm20948_whoami_read_count_debug = whoami_diag.read_count;

    if ((whoami_diag.result == ADS_ICM20948_WHOAMI_OK) &&
        (whoami_diag.who_am_i_value == 0xEAu))
    {
      g_icm20948_whoami_ok = 1u;
    }
    else
    {
      g_icm20948_whoami_ok = 0u;
    }

    if (g_icm20948_whoami_ok != 0u)
    {
      ADS_ICM20948_WakeDiagnostics wake_diag;

      wake_diag = ADS_ICM20948_Wake(
          &hi2c2,
          icm_probe_diag.detected_address_7bit
      );

      g_icm20948_wake_result_debug = (uint32_t)wake_diag.result;
      g_icm20948_wake_hal_status_debug = (uint32_t)wake_diag.hal_status;
      g_icm20948_wake_count_debug = wake_diag.wake_count;

      if (wake_diag.result == ADS_ICM20948_WAKE_OK)
      {
        ADS_ICM20948_ConfigDiagnostics config_diag;

        config_diag = ADS_ICM20948_ConfigureAccelGyro(
            &hi2c2,
            icm_probe_diag.detected_address_7bit
        );

        g_icm20948_config_result_debug = (uint32_t)config_diag.result;
        g_icm20948_config_hal_status_debug = (uint32_t)config_diag.hal_status;
        g_icm20948_config_count_debug = config_diag.config_count;

        if (config_diag.result == ADS_ICM20948_CONFIG_OK)
        {
          g_icm20948_config_ok = 1u;
        }
        else
        {
          g_icm20948_config_ok = 0u;
        }
      }
      else
      {
        g_icm20948_config_ok = 0u;
      }
    }
  }
  else
  {
    g_icm20948_whoami_result_debug = (uint32_t)ADS_ICM20948_WHOAMI_NOT_RUN;
    g_icm20948_whoami_value_debug = 0u;
    g_icm20948_whoami_hal_status_debug = (uint32_t)HAL_ERROR;
    g_icm20948_whoami_read_count_debug = 0u;
    g_icm20948_whoami_ok = 0u;
  }



  if ((g_icm20948_probe_found != 0u) &&
      (g_icm20948_whoami_ok != 0u) &&
      (g_icm20948_config_ok != 0u))
  {
    GOOSE_AK09916_InitForTelemetry();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    const uint32_t now_ms = HAL_GetTick();

    if ((g_ak09916_config_ok != 0u) &&
        ((uint32_t)(now_ms - g_ak09916_last_read_ms) >= 100u))
    {
      g_ak09916_last_read_ms = now_ms;
      GOOSE_AK09916_ReadMagOnce();
    }

    if ((uint32_t)(now_ms - g_ads_last_run_ms) >= 10u)
    {
      g_ads_last_run_ms = now_ms;
      ADS_App_Run();
      g_ads_run_count++;
    }

    if ((g_icm20948_whoami_ok != 0u) &&
        (g_icm20948_config_ok != 0u) &&
        ((uint32_t)(now_ms - g_icm20948_last_raw_read_ms) >= 100u))
    {
      ADS_ICM20948_RawDiagnostics raw_diag;

      g_icm20948_last_raw_read_ms = now_ms;

      raw_diag = ADS_ICM20948_ReadAccelGyroRaw(
          &hi2c2,
          (uint8_t)g_icm20948_detected_addr_debug
      );

      g_icm20948_raw_result_debug = (uint32_t)raw_diag.result;
      g_icm20948_raw_hal_status_debug = (uint32_t)raw_diag.hal_status;
      g_icm20948_raw_read_count_debug = raw_diag.read_count;

      g_icm20948_accel_x_raw_debug = (int32_t)raw_diag.accel_x_raw;
      g_icm20948_accel_y_raw_debug = (int32_t)raw_diag.accel_y_raw;
      g_icm20948_accel_z_raw_debug = (int32_t)raw_diag.accel_z_raw;

      g_icm20948_gyro_x_raw_debug = (int32_t)raw_diag.gyro_x_raw;
      g_icm20948_gyro_y_raw_debug = (int32_t)raw_diag.gyro_y_raw;
      g_icm20948_gyro_z_raw_debug = (int32_t)raw_diag.gyro_z_raw;

      g_icm20948_accel_x_mps2_debug =
          ((float)raw_diag.accel_x_raw / ICM20948_ACCEL_4G_LSB_PER_G) *
          ICM20948_STANDARD_GRAVITY_MPS2;

      g_icm20948_accel_y_mps2_debug =
          ((float)raw_diag.accel_y_raw / ICM20948_ACCEL_4G_LSB_PER_G) *
          ICM20948_STANDARD_GRAVITY_MPS2;

      g_icm20948_accel_z_mps2_debug =
          ((float)raw_diag.accel_z_raw / ICM20948_ACCEL_4G_LSB_PER_G) *
          ICM20948_STANDARD_GRAVITY_MPS2;

      g_icm20948_gyro_x_radps_debug =
          ((float)raw_diag.gyro_x_raw / ICM20948_GYRO_500DPS_LSB_PER_DPS) *
          DEG_TO_RAD;

      g_icm20948_gyro_y_radps_debug =
          ((float)raw_diag.gyro_y_raw / ICM20948_GYRO_500DPS_LSB_PER_DPS) *
          DEG_TO_RAD;

      g_icm20948_gyro_z_radps_debug =
          ((float)raw_diag.gyro_z_raw / ICM20948_GYRO_500DPS_LSB_PER_DPS) *
          DEG_TO_RAD;

      if (raw_diag.result == ADS_ICM20948_RAW_OK)
      {
        g_icm20948_raw_ok = 1u;
      }
      else
      {
        g_icm20948_raw_ok = 0u;
      }
    }

        {
    #if ADS_UART_TELEMETRY_50HZ_ENABLE
          const uint32_t telemetry_period_ms = 20u;
    #else
          const uint32_t telemetry_period_ms = 500u;
    #endif

        /*
     * Continuous accel + gyro CSV telemetry.
     *
     * Format:
     *   IMU,t_ms,sample,dt_ms,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw
     *
     * Units:
     *   accel raw counts, configured at +/-4g
     *   gyro raw counts, configured at +/-500 dps
     *
     * Telemetry rate:
     *   100 ms = 10 Hz
     */
    
    if ((uint32_t)(now_ms - g_imu_csv_last_ms) >= 100u)
    {
      char msg[160];

      uint32_t rel_ms;
      uint32_t dt_ms;

      if (g_imu_csv_start_ms == 0u)
      {
        g_imu_csv_start_ms = now_ms;
        g_imu_csv_last_ms = now_ms;
        g_imu_csv_sample = 0u;

        rel_ms = 0u;
        dt_ms = 0u;
      }
      else
      {
        rel_ms = (uint32_t)(now_ms - g_imu_csv_start_ms);
        dt_ms = (uint32_t)(now_ms - g_imu_csv_last_ms);

        g_imu_csv_last_ms = now_ms;
        g_imu_csv_sample++;
      }

      int len = snprintf(
          msg,
          sizeof(msg),
          "IMU,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
          (unsigned long)rel_ms,
          (unsigned long)g_imu_csv_sample,
          (unsigned long)dt_ms,
          (long)g_icm20948_accel_x_raw_debug,
          (long)g_icm20948_accel_y_raw_debug,
          (long)g_icm20948_accel_z_raw_debug,
          (long)g_icm20948_gyro_x_raw_debug,
          (long)g_icm20948_gyro_y_raw_debug,
          (long)g_icm20948_gyro_z_raw_debug
      );

      if ((len > 0) && (len < (int)sizeof(msg)))
      {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)msg,
                          (uint16_t)len,
                          100u);
      }
    }
    }

    g_ads_healthy_debug = ADS_App_IsHealthy() ? 1u : 0u;

    {
      uint32_t led_period_ms = 500u;

      if ((g_icm20948_probe_found == 0u) ||
          (g_icm20948_whoami_ok == 0u) ||
          (g_icm20948_config_ok == 0u) ||
          (g_icm20948_raw_ok == 0u))
      {
        led_period_ms = 100u;
      }

      if ((uint32_t)(now_ms - g_ads_status_last_toggle_ms) >= led_period_ms)
      {
        g_ads_status_last_toggle_ms = now_ms;
        HAL_GPIO_TogglePin(ADS_STATUS_LED_GPIO_Port, ADS_STATUS_LED_Pin);
      }
    }


        /*
     * Magnetometer CSV telemetry.
     *
     * Format:
     *   MAG,t_ms,sample,dt_ms,bypass,who,cfg,raw_ok,wia2,st1,st2,mx_raw,my_raw,mz_raw
     */
    if ((uint32_t)(now_ms - g_mag_csv_last_ms) >= 100u)
    {
      char msg[180];

      uint32_t rel_ms;
      uint32_t dt_ms;

      if (g_mag_csv_start_ms == 0u)
      {
        g_mag_csv_start_ms = now_ms;
        g_mag_csv_last_ms = now_ms;
        g_mag_csv_sample = 0u;

        rel_ms = 0u;
        dt_ms = 0u;
      }
      else
      {
        rel_ms = (uint32_t)(now_ms - g_mag_csv_start_ms);
        dt_ms = (uint32_t)(now_ms - g_mag_csv_last_ms);

        g_mag_csv_last_ms = now_ms;
        g_mag_csv_sample++;
      }

      int len = snprintf(
          msg,
          sizeof(msg),
          "MAG,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%ld\r\n",
          (unsigned long)rel_ms,
          (unsigned long)g_mag_csv_sample,
          (unsigned long)dt_ms,
          (unsigned long)g_ak09916_bypass_ok,
          (unsigned long)g_ak09916_whoami_ok,
          (unsigned long)g_ak09916_config_ok,
          (unsigned long)g_ak09916_raw_ok,
          (unsigned long)g_ak09916_wia2_debug,
          (unsigned long)g_ak09916_st1_debug,
          (unsigned long)g_ak09916_st2_debug,
          (long)g_ak09916_mag_x_raw_debug,
          (long)g_ak09916_mag_y_raw_debug,
          (long)g_ak09916_mag_z_raw_debug
      );

      if ((len > 0) && (len < (int)sizeof(msg)))
      {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)msg,
                          (uint16_t)len,
                          100u);
      }
    }
    /* USER CODE END WHILE */




    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
#if 0
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

#endif

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADS_STATUS_LED_GPIO_Port, ADS_STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADS_STATUS_LED_Pin */
  GPIO_InitStruct.Pin = ADS_STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADS_STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &c, 1u, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
