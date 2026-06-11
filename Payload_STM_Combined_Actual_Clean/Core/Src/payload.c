#include "payload.h"
#include "langmuir.h"
#include "gps_task.h"
#include "burn.h"
#include <string.h>

void payload_task(PayloadContext *ctx, PayloadMode mode,
                  I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart,
                  GOOSE_Payload *out)
{

    memset(out, 0, sizeof(GOOSE_Payload));
    out->timestamp_ms = HAL_GetTick();

    switch (mode)
    {
        case MODE_SCIENCE:
            langmuir_task(hi2c);

            // Copy langmuir data into output struct
            memcpy(out->meas_voltage, langmuir_data.voltage, sizeof(langmuir_data.voltage));
            memcpy(out->meas_current, langmuir_data.current, sizeof(langmuir_data.current));

            // GPS
            gps_task(huart);
            out->gps_error = (GPS.lock == 0) ? 1 : 0;
            memcpy(&out->gps, &GPS, sizeof(GPS_t));

            break;

        case MODE_DEPLOYMENT:
            burn_task();
            break;

        default:
            break;
    }
}
