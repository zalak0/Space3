#include "payload.h"
#include "langmuir.h"
#include "gps_task.h"
#include "burn.h"

void payload_task(PayloadContext *ctx, PayloadMode mode,
                  I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    switch (mode)
    {
        case MODE_SCIENCE:
            langmuir_task(hi2c);
            gps_task(huart);
            break;

        case MODE_DEPLOYMENT:
            burn_task();
            break;

        default:
            break;
    }
}
