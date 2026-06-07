#ifndef MAIN_H
#define MAIN_H

#include "stm32h7xx_hal.h"

#define EPS_3V3_DISABLE_GPIO_Port   GPIOA
#define EPS_3V3_DISABLE_Pin         GPIO_PIN_15

#define EPS_VBUS_DISABLE_GPIO_Port  GPIOA
#define EPS_VBUS_DISABLE_Pin        GPIO_PIN_14

#define EPS_5V_DISABLE_GPIO_Port    GPIOA
#define EPS_5V_DISABLE_Pin          GPIO_PIN_13

#define EPS_3V3_PG_GPIO_Port        GPIOA
#define EPS_3V3_PG_Pin              GPIO_PIN_5

#define EPS_3V3_FLT_GPIO_Port       GPIOA
#define EPS_3V3_FLT_Pin             GPIO_PIN_6

#define EPS_5V_PG_GPIO_Port         GPIOA
#define EPS_5V_PG_Pin               GPIO_PIN_7

#define EPS_5V_FLT_GPIO_Port        GPIOA
#define EPS_5V_FLT_Pin              GPIO_PIN_8

#define EPS_VBUS_FLT_GPIO_Port      GPIOA
#define EPS_VBUS_FLT_Pin            GPIO_PIN_9

#define EPS_VBUS_PG_GPIO_Port       GPIOA
#define EPS_VBUS_PG_Pin             GPIO_PIN_10

#define BQ25798_CE_GPIO_Port        GPIOA
#define BQ25798_CE_Pin              GPIO_PIN_11

#endif