#include "payload.h"
#include "langmuir.h"
#include "gps_task.h"
#include "burn.h"

#include "fsw_ctx.h"
#include "modes.h"

void payload_task(sat_mode_t mode, fsw_ctx_t *ctx,
                  I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    switch (mode)
    {
        case MODE_SCIENCE:
            if(!ctx -> has_burned){
				burn_task();
				ctx -> has_burned = 1;
            }
            langmuir_task(hi2c);
            gps_task(huart);
            break;

        default:
            break;
    }
}
