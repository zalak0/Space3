#include "payload.h"
#include "gps.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  Private defines                                                     */
/* ------------------------------------------------------------------ */
#define MCP3021_ADDR      (0x4D << 1)
#define MEAS_BUFFER_SIZE  100
#define BURN_WIRE_PIN     GPIO_PIN_11
#define BURN_WIRE_PORT    GPIOJ
#define LANGMUIR_EN_PIN   GPIO_PIN_9
#define LANGMUIR_EN_PORT  GPIOJ
#define BURN_DURATION_MS  2000

/* ------------------------------------------------------------------ */
/*  Private function prototypes                                         */
/* ------------------------------------------------------------------ */
static int32_t  mcp3021_read_raw    (I2C_HandleTypeDef *hi2c);
static float    mcp3021_read_voltage(I2C_HandleTypeDef *hi2c);
static float    read_internal_adc   (void);

static void langmuir_task(PayloadCtx *ctx, I2C_HandleTypeDef *hi2c);
static void gps_task     (UART_HandleTypeDef *huart);
static void burn_task    (void);

/* ------------------------------------------------------------------ */
/*  External ADC handle (defined in main.c / adc.c by CubeMX)         */
/* ------------------------------------------------------------------ */
extern ADC_HandleTypeDef hadc3;

/* ================================================================== */
/*  Public API                                                          */
/* ================================================================== */

void payload_init(PayloadCtx *ctx, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    (void)huart; /* reserved for future UART init steps */

    memset(ctx, 0, sizeof(PayloadCtx));

    /* Calibrate and enable Langmuir probe */
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_GPIO_WritePin(LANGMUIR_EN_PORT, LANGMUIR_EN_PIN, GPIO_PIN_SET);

    /* GPS */
    GPS_Init();
}

void payload_task(PayloadCtx *ctx, PayloadMode mode,
                  I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    switch (mode)
    {
        case MODE_SCIENCE:
            langmuir_task(ctx, hi2c);
            gps_task(huart);
            break;

        case MODE_DEPLOYMENT:
            burn_task();
            break;

        default:
            break;
    }
}

/* ================================================================== */
/*  Sub-tasks                                                           */
/* ================================================================== */

static void langmuir_task(PayloadCtx *ctx, I2C_HandleTypeDef *hi2c)
{
    /* Read raw voltages */
    ctx->voltage_triangle = mcp3021_read_voltage(hi2c);
    ctx->voltage_langmuir = read_internal_adc();

    /* Apply calibration / scaling to get physical units */
    ctx->voltage = (ctx->voltage_triangle * 169.0f) - 266.4f;
    ctx->current = (((ctx->voltage_langmuir * 70.3f) - 109.9f) - ctx->voltage) / 3000.0f;

    /* Circular buffer store */
    ctx->measurements_voltage[ctx->meas_index] = ctx->voltage;
    ctx->measurements_current[ctx->meas_index] = ctx->current;
    ctx->meas_index = (ctx->meas_index + 1) % MEAS_BUFFER_SIZE;
}

static void gps_task(UART_HandleTypeDef *huart)
{
    if (!gps_line_ready)
        return;

    gps_line_ready = 0;

    if (GPS.lock > 0) {
        char out[128];
        snprintf(out, sizeof(out),
            "LAT: %.6f  LON: %.6f  Sats: %d  Alt: %.1f m\r\n",
            GPS.dec_latitude,
            GPS.dec_longitude,
            GPS.satelites,
            GPS.msl_altitude);
        HAL_UART_Transmit(huart, (uint8_t *)out, strlen(out), 100);
    } else {
        uint8_t msg[] = "No fix\r\n";
        HAL_UART_Transmit(huart, msg, sizeof(msg) - 1, 100);
    }
}

static void burn_task(void)
{
    HAL_GPIO_WritePin(BURN_WIRE_PORT, BURN_WIRE_PIN, GPIO_PIN_SET);
    HAL_Delay(BURN_DURATION_MS);
    HAL_GPIO_WritePin(BURN_WIRE_PORT, BURN_WIRE_PIN, GPIO_PIN_RESET);
}

/* ================================================================== */
/*  Driver helpers                                                      */
/* ================================================================== */

static int32_t mcp3021_read_raw(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2] = {0};

    if (HAL_I2C_Master_Receive(hi2c, MCP3021_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return -1;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw = (raw >> 2) & 0x03FF;
    return (int32_t)raw;
}

static float mcp3021_read_voltage(I2C_HandleTypeDef *hi2c)
{
    int32_t raw = mcp3021_read_raw(hi2c);
    if (raw < 0) return -1.0f;
    return ((float)raw / 1023.0f) * 3.3f;
}

static float read_internal_adc(void)
{
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 100) != HAL_OK)
        return -1.0f;

    uint32_t raw = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    return ((float)raw / 65535.0f) * 3.3f;
}
