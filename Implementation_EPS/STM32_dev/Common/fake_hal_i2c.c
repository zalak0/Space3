#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

GPIO_TypeDef fake_GPIOA;
GPIO_TypeDef fake_GPIOB;
GPIO_TypeDef fake_GPIOC;
GPIO_TypeDef fake_GPIOD;
GPIO_TypeDef fake_GPIOE;
GPIO_TypeDef fake_GPIOF;
GPIO_TypeDef fake_GPIOG;
GPIO_TypeDef fake_GPIOH;
GPIO_TypeDef fake_GPIOI;
GPIO_TypeDef fake_GPIOJ;
GPIO_TypeDef fake_GPIOK;

I2C_HandleTypeDef hi2c1;

#define FAKE_BQ76920_ADDR_HAL      (0x18 << 1)
#define FAKE_BQ25798_ADDR_HAL      (0x6B << 1)

static uint8_t fake_bq76920_registers[256];
static uint8_t fake_bq25798_registers[256];

static uint8_t fake_bq76920_initialised = 0;
static uint8_t fake_bq25798_initialised = 0;

static GPIO_PinState fake_gpio_state[16];

static uint32_t fake_tick_ms = 0;

static int FakeGPIO_PinToIndex(uint16_t pin)
{
    for (int i = 0; i < 16; i++) {
        if (pin == (uint16_t)(1U << i)) {
            return i;
        }
    }

    return -1;
}

void FakeGPIO_SetPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state)
{
    (void)GPIOx;

    int index = FakeGPIO_PinToIndex(GPIO_Pin);

    if (index >= 0) {
        fake_gpio_state[index] = state;
    }
}

GPIO_PinState FakeGPIO_GetPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    (void)GPIOx;

    int index = FakeGPIO_PinToIndex(GPIO_Pin);

    if (index >= 0) {
        return fake_gpio_state[index];
    }

    return GPIO_PIN_RESET;
}

static void Fake_Set16_BE(uint8_t *regs, uint8_t reg, uint16_t value)
{
    regs[reg]     = (uint8_t)((value >> 8) & 0xFF);
    regs[reg + 1] = (uint8_t)(value & 0xFF);
}

static void Fake_Set14BitRegister(uint8_t high_reg, uint16_t raw14)
{
    raw14 &= 0x3FFF;

    fake_bq76920_registers[high_reg]     = (uint8_t)((raw14 >> 8) & 0x3F);
    fake_bq76920_registers[high_reg + 1] = (uint8_t)(raw14 & 0xFF);
}

static void FakeBQ76920_InitRegisters(void)
{
    if (fake_bq76920_initialised) {
        return;
    }

    memset(fake_bq76920_registers, 0, sizeof(fake_bq76920_registers));

    fake_bq76920_registers[0x50] = 0x00;
    fake_bq76920_registers[0x51] = 0x00;
    fake_bq76920_registers[0x59] = 0x00;

    Fake_Set14BitRegister(0x0C, 10137);
    Fake_Set14BitRegister(0x0E, 10142);
    Fake_Set14BitRegister(0x10, 10131);
    Fake_Set14BitRegister(0x12, 10140);
    Fake_Set14BitRegister(0x14, 10135);

    fake_bq76920_initialised = 1;
}

void FakeBQ76920_SetCellRaw(uint8_t cell_index, uint16_t raw14)
{
    uint8_t high_reg;

    if (!fake_bq76920_initialised) {
        FakeBQ76920_InitRegisters();
    }

    if (cell_index >= 5) {
        return;
    }

    high_reg = 0x0C + (cell_index * 2);
    Fake_Set14BitRegister(high_reg, raw14);
}

void FakeBQ76920_SetSysStat(uint8_t value)
{
    if (!fake_bq76920_initialised) {
        FakeBQ76920_InitRegisters();
    }

    fake_bq76920_registers[0x00] = value;
}

void FakeBQ25798_Reset(void)
{
    memset(fake_bq25798_registers, 0, sizeof(fake_bq25798_registers));

    fake_bq25798_registers[0x48] = 0x98;

    fake_bq25798_initialised = 1;
}

void FakeBQ25798_Set8(uint8_t reg, uint8_t value)
{
    if (!fake_bq25798_initialised) {
        FakeBQ25798_Reset();
    }

    fake_bq25798_registers[reg] = value;
}

void FakeBQ25798_Set16(uint8_t reg, uint16_t value)
{
    if (!fake_bq25798_initialised) {
        FakeBQ25798_Reset();
    }

    Fake_Set16_BE(fake_bq25798_registers, reg, value);
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint32_t trials,
    uint32_t timeout
)
{
    (void)hi2c;
    (void)trials;
    (void)timeout;

    FakeBQ76920_InitRegisters();

    if (!fake_bq25798_initialised) {
        FakeBQ25798_Reset();
    }

    if (dev_addr == FAKE_BQ76920_ADDR_HAL) {
        return HAL_OK;
    }

    if (dev_addr == FAKE_BQ25798_ADDR_HAL) {
        return HAL_OK;
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_add_size,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout
)
{
    uint8_t *regs = 0;

    (void)hi2c;
    (void)mem_add_size;
    (void)timeout;

    if (data == 0) {
        return HAL_ERROR;
    }

    if (dev_addr == FAKE_BQ76920_ADDR_HAL) {
        FakeBQ76920_InitRegisters();
        regs = fake_bq76920_registers;
    } else if (dev_addr == FAKE_BQ25798_ADDR_HAL) {
        if (!fake_bq25798_initialised) {
            FakeBQ25798_Reset();
        }

        regs = fake_bq25798_registers;
    } else {
        return HAL_ERROR;
    }

    if ((mem_addr + size) > 256) {
        return HAL_ERROR;
    }

    for (uint16_t i = 0; i < size; i++) {
        data[i] = regs[mem_addr + i];
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_add_size,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout
)
{
    uint8_t *regs = 0;

    (void)hi2c;
    (void)mem_add_size;
    (void)timeout;

    if (data == 0) {
        return HAL_ERROR;
    }

    if (dev_addr == FAKE_BQ76920_ADDR_HAL) {
        FakeBQ76920_InitRegisters();
        regs = fake_bq76920_registers;
    } else if (dev_addr == FAKE_BQ25798_ADDR_HAL) {
        if (!fake_bq25798_initialised) {
            FakeBQ25798_Reset();
        }

        regs = fake_bq25798_registers;
    } else {
        return HAL_ERROR;
    }

    if ((mem_addr + size) > 256) {
        return HAL_ERROR;
    }

    for (uint16_t i = 0; i < size; i++) {
        uint16_t reg = mem_addr + i;

        if (dev_addr == FAKE_BQ76920_ADDR_HAL && reg == 0x00) {
            regs[reg] &= (uint8_t)(~data[i]);
        } else {
            regs[reg] = data[i];
        }
    }

    return HAL_OK;
}

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    (void)GPIOx;
    (void)GPIO_Init;
}

void HAL_GPIO_WritePin(
    GPIO_TypeDef *GPIOx,
    uint16_t GPIO_Pin,
    GPIO_PinState PinState
)
{
    (void)GPIOx;

    FakeGPIO_SetPin(GPIOx, GPIO_Pin, PinState);

    printf("GPIO 0x%04X = %s\n",
           GPIO_Pin,
           PinState == GPIO_PIN_SET ? "HIGH" : "LOW");
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    return FakeGPIO_GetPin(GPIOx, GPIO_Pin);
}

void HAL_Delay(uint32_t delay_ms)
{
    fake_tick_ms += delay_ms;
}

uint32_t HAL_GetTick(void)
{
    return fake_tick_ms++;
}

__attribute__((weak)) void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}

void Fake_EXTI_Trigger(uint16_t GPIO_Pin)
{
    HAL_GPIO_EXTI_Callback(GPIO_Pin);
}