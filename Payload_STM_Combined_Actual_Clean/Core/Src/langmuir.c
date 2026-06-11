#include "langmuir.h"
#include "main.h"  // for hadc3
#include <string.h>

extern ADC_HandleTypeDef hadc3;

LangmuirData langmuir_data = {0};

// ---------- private helpers ----------

static int32_t MCP3021_ReadRaw(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2] = {0};
    if (HAL_I2C_Master_Receive(hi2c, MCP3021_ADDR, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return -1;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw = (raw >> 2) & 0x03FF;
    return (int32_t)raw;
}

static float MCP3021_ReadVoltage(I2C_HandleTypeDef *hi2c)
{
    int32_t raw = MCP3021_ReadRaw(hi2c);
    if (raw < 0) return -1.0f;
    return ((float)raw / 1023.0f) * 3.3f;
}

static float ReadInternalADC(void)
{
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 100) != HAL_OK)
        return -1.0f;
    uint32_t raw = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    return ((float)raw / 65535.0f) * 3.3f;
}

// ---------- public API ----------

void langmuir_init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_9, GPIO_PIN_SET);  // Activate probe
}

void langmuir_task(I2C_HandleTypeDef *hi2c)
{
    float v_triangle = MCP3021_ReadVoltage(hi2c);
    float v_langmuir = ReadInternalADC();

    float voltage = (v_triangle * 169.0f) - 266.4f;
    float current = (((v_langmuir * 70.3f) - 109.9f) - voltage) / 3000.0f;

    langmuir_data.voltage[langmuir_data.index] = voltage;
    langmuir_data.current[langmuir_data.index] = current;
    langmuir_data.index = (langmuir_data.index + 1) % MEAS_BUFFER_SIZE;
}
