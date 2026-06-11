#include "burn.h"
#include "main.h"

void burn_task(void)
{
    static uint8_t has_fired = 0;
    if (has_fired) return;

    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_11, GPIO_PIN_RESET);

    has_fired = 1;
}
