#include <hardware_chips/gps/gps.h>
#include <hardware_chips/gps/gps_task.h>
#include <stdio.h>
#include <string.h>

void gps_task_init(void)
{
    GPS_Init();
}

void gps_task(UART_HandleTypeDef *huart)
{
    // gps_line_ready guard is left commented out in your original;
    // wire it back in once GPS IRQ/DMA is set up properly
    gps_line_ready = 0;

    if (GPS.lock > 0) {
        char out[128];
        snprintf(out, sizeof(out),
            "LAT: %.6f  LON: %.6f  Sats: %d  Alt: %.1f m\r\n",
            GPS.dec_latitude,
            GPS.dec_longitude,
            GPS.satelites,
            GPS.msl_altitude);
        HAL_UART_Transmit(huart, (uint8_t *)out, strlen(out), 100);
    } else {
        uint8_t msg[] = "No fix\r\n";
        HAL_UART_Transmit(huart, msg, sizeof(msg) - 1, 100);
    }
}
