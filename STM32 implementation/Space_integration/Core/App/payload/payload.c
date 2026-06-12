#include <hardware_chips/gps/gps_task.h>
#include "payload.h"
#include "langmuir.h"
#include "burn.h"

#include "common/fsw_ctx.h"
#include "stm/modes.h"

void payload_task(sat_mode_t mode, fsw_ctx_t *ctx,
                  I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    switch (mode)
    {
    	case MODE_BURN:
			burn_task();
        case MODE_SCIENCE:
        	langmuir_init();
        	gps_task_init();

            langmuir_task(hi2c);
            gps_task(huart);
            break;

        default:
            break;
    }
}
