#include <hardware_chips/gps/gps_task.h>
#include "payload.h"
#include "langmuir.h"
#include "burn.h"

#include "common/fsw_ctx.h"
#include "stm/modes.h"

void payload_task(sat_mode_t mode, fsw_ctx_t *ctx,
                  I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    GOOSE_Payload out;
    memset(&out, 0, sizeof(GOOSE_Payload));
    out.timestamp_ms = HAL_GetTick();

    switch (mode)
    {
    	case MODE_BURN:
			burn_task();
        case MODE_SCIENCE:
        	langmuir_init();
        	gps_task_init();

            langmuir_task(hi2c);

             // Copy langmuir data into output struct
            memcpy(out.meas_voltage, langmuir_data.voltage, sizeof(langmuir_data.voltage));
            memcpy(out.meas_current, langmuir_data.current, sizeof(langmuir_data.current));
            gps_task(huart);
            out.gps_error = (GPS.lock == 0) ? 1 : 0;
            memcpy(&out.gps, &GPS, sizeof(GPS_t));

            {
                char line[64];
                int  len;

                // Timestamp
                len = snprintf(line, sizeof(line),
                               "T=%lu ms", out.timestamp_ms);
                AX25Packaging((uint8_t*)line, len, PAYLOAD, BUFFER_SIZE);

                // Langmuir voltage and current pairs
                for (int i = 0; i < MEAS_BUFFER_SIZE; i++) {
                    len = snprintf(line, sizeof(line),
                                   "V[%d]=%.4f I[%d]=%.6f",
                                   i, out.meas_voltage[i],
                                   i, out.meas_current[i]);
                    AX25Packaging((uint8_t*)line, len, PAYLOAD, BUFFER_SIZE);
                }

                // GPS
                if (!out.gps_error) {
                    len = snprintf(line, sizeof(line),
                                   "LAT=%.6f LON=%.6f ALT=%.2f",
                                   out.gps.dec_latitude,
                                   out.gps.dec_longitude,
                                   out.gps.msl_altitude);
                } else {
                    len = snprintf(line, sizeof(line), "GPS=NO LOCK");
                }
                AX25Packaging((uint8_t*)line, len, PAYLOAD, BUFFER_SIZE);
            }

            break;

        default:
            break;
    }
}
