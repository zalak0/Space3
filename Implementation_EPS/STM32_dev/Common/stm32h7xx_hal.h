#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>

/* ============================================================
 * BASIC HAL STATUS
 * ============================================================ */

typedef enum
{
    HAL_OK      = 0,
    HAL_ERROR   = 1,
    HAL_BUSY    = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

/* ============================================================
 * FAKE I2C HANDLE
 * ============================================================ */

typedef struct
{
    int dummy;
} I2C_HandleTypeDef;

/* ============================================================
 * FAKE GPIO TYPES
 * ============================================================ */

typedef struct
{
    int dummy;
} GPIO_TypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET   = 1
} GPIO_PinState;

typedef struct
{
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

/* ============================================================
 * GPIO PIN DEFINITIONS
 * ============================================================ */

#define GPIO_PIN_0                  ((uint16_t)0x0001)
#define GPIO_PIN_1                  ((uint16_t)0x0002)
#define GPIO_PIN_2                  ((uint16_t)0x0004)
#define GPIO_PIN_3                  ((uint16_t)0x0008)
#define GPIO_PIN_4                  ((uint16_t)0x0010)
#define GPIO_PIN_5                  ((uint16_t)0x0020)
#define GPIO_PIN_6                  ((uint16_t)0x0040)
#define GPIO_PIN_7                  ((uint16_t)0x0080)
#define GPIO_PIN_8                  ((uint16_t)0x0100)
#define GPIO_PIN_9                  ((uint16_t)0x0200)
#define GPIO_PIN_10                 ((uint16_t)0x0400)
#define GPIO_PIN_11                 ((uint16_t)0x0800)
#define GPIO_PIN_12                 ((uint16_t)0x1000)
#define GPIO_PIN_13                 ((uint16_t)0x2000)
#define GPIO_PIN_14                 ((uint16_t)0x4000)
#define GPIO_PIN_15                 ((uint16_t)0x8000)

/* ============================================================
 * GPIO MODE DEFINITIONS
 * ============================================================ */

#define GPIO_MODE_INPUT             0x00U
#define GPIO_MODE_OUTPUT_PP         0x01U
#define GPIO_MODE_OUTPUT_OD         0x02U
#define GPIO_MODE_AF_PP             0x03U
#define GPIO_MODE_AF_OD             0x04U
#define GPIO_MODE_ANALOG            0x05U

/* ============================================================
 * GPIO PULL DEFINITIONS
 * ============================================================ */

#define GPIO_NOPULL                 0x00U
#define GPIO_PULLUP                 0x01U
#define GPIO_PULLDOWN               0x02U

/* ============================================================
 * GPIO SPEED DEFINITIONS
 * ============================================================ */

#define GPIO_SPEED_FREQ_LOW         0x00U
#define GPIO_SPEED_FREQ_MEDIUM      0x01U
#define GPIO_SPEED_FREQ_HIGH        0x02U
#define GPIO_SPEED_FREQ_VERY_HIGH   0x03U

/* ============================================================
 * FAKE GPIO PORTS
 * ============================================================ */

extern GPIO_TypeDef fake_GPIOA;
extern GPIO_TypeDef fake_GPIOB;
extern GPIO_TypeDef fake_GPIOC;
extern GPIO_TypeDef fake_GPIOD;
extern GPIO_TypeDef fake_GPIOE;

#define GPIOA                       (&fake_GPIOA)
#define GPIOB                       (&fake_GPIOB)
#define GPIOC                       (&fake_GPIOC)
#define GPIOD                       (&fake_GPIOD)
#define GPIOE                       (&fake_GPIOE)

/* ============================================================
 * I2C DEFINITIONS
 * ============================================================ */

#define I2C_MEMADD_SIZE_8BIT        1U
#define I2C_MEMADD_SIZE_16BIT       2U

#define HAL_MAX_DELAY               0xFFFFFFFFU

/* ============================================================
 * I2C FUNCTION DECLARATIONS
 * ============================================================ */

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint32_t trials,
    uint32_t timeout
);

HAL_StatusTypeDef HAL_I2C_Mem_Read(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_add_size,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout
);

HAL_StatusTypeDef HAL_I2C_Mem_Write(
    I2C_HandleTypeDef *hi2c,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_add_size,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout
);

/* ============================================================
 * GPIO FUNCTION DECLARATIONS
 * ============================================================ */

void HAL_GPIO_Init(
    GPIO_TypeDef *GPIOx,
    GPIO_InitTypeDef *GPIO_Init
);

void HAL_GPIO_WritePin(
    GPIO_TypeDef *GPIOx,
    uint16_t GPIO_Pin,
    GPIO_PinState PinState
);

GPIO_PinState HAL_GPIO_ReadPin(
    GPIO_TypeDef *GPIOx,
    uint16_t GPIO_Pin
);

/* ============================================================
 * TIME FUNCTION DECLARATIONS
 * ============================================================ */

void HAL_Delay(uint32_t delay_ms);
uint32_t HAL_GetTick(void);


void FakeGPIO_SetPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state);
GPIO_PinState FakeGPIO_GetPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void FakeBQ25798_Reset(void);
void FakeBQ25798_Set8(uint8_t reg, uint8_t value);
void FakeBQ25798_Set16(uint8_t reg, uint16_t value);

void FakeBQ76920_SetCellRaw(uint8_t cell_index, uint16_t raw14);
void FakeBQ76920_SetSysStat(uint8_t value);


#endif