#include <stdio.h>
#include <string.h>
#include "gps.h"
#include "stm32h7xx_hal.h"


extern UART_HandleTypeDef huart1;  // ← add this line

uint8_t rx_data = 0;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;
volatile uint8_t gps_line_ready = 0;

GPS_t GPS;

void GPS_Init(void)
{
    HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
}

//void GPS_UART_CallBack(void)
//{
//    if (rx_data != '\n' && rx_index < (GPSBUFSIZE - 1)) {
//        rx_buffer[rx_index++] = rx_data;
//    } else {
//        rx_buffer[rx_index] = '\0'; // null terminate
//        if (GPS_validate((char *)rx_buffer)) {
//            GPS_parse((char *)rx_buffer);
//        }
//        rx_index = 0;
//        memset(rx_buffer, 0, sizeof(rx_buffer));
//    }
//    // Re-arm the interrupt for the next byte
//    HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
//}

//void GPS_UART_CallBack(void)
//{
//    if (rx_data != '\n' && rx_index < (GPSBUFSIZE - 1)) {
//        rx_buffer[rx_index++] = rx_data;
//    } else {
//        rx_buffer[rx_index] = '\0';
//        gps_line_ready = 1;  // ← add this
//        if (GPS_validate((char *)rx_buffer)) {
//            GPS_parse((char *)rx_buffer);
//        }
//        rx_index = 0;
//        memset(rx_buffer, 0, sizeof(rx_buffer));
//    }
//    HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
//}

uint8_t print_buffer[GPSBUFSIZE];
void GPS_UART_CallBack(void)
{
    if (rx_data != '\n' && rx_index < (GPSBUFSIZE - 1)) {
        rx_buffer[rx_index++] = rx_data;
    } else {
        rx_buffer[rx_index] = '\0';
        memcpy(print_buffer, rx_buffer, rx_index + 1); // copy before clearing
        gps_line_ready = 1;
        if (GPS_validate((char *)rx_buffer)) {
            GPS_parse((char *)rx_buffer);
        }
        rx_index = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));
    }
    HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
}

int GPS_validate(char *nmeastr)
{
    char check[3];
    char checkcalcstr[3];
    int i = 0;
    int calculated_check = 0;

    if (nmeastr[i] == '$')
        i++;
    else
        return 0;

    while ((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)) {
        calculated_check ^= nmeastr[i];
        i++;
    }

    if (i >= 75)
        return 0;

    if (nmeastr[i] == '*') {
        check[0] = nmeastr[i + 1];
        check[1] = nmeastr[i + 2];
        check[2] = 0;
    } else {
        return 0;
    }

    sprintf(checkcalcstr, "%02X", calculated_check);
    return ((checkcalcstr[0] == check[0]) && (checkcalcstr[1] == check[1])) ? 1 : 0;
}

void GPS_parse(char *GPSstrParse)
{
    if (!strncmp(GPSstrParse, "$GPGGA", 6)) {
        sscanf(GPSstrParse,
               "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
               &GPS.utc_time,
               &GPS.nmea_latitude, &GPS.ns,
               &GPS.nmea_longitude, &GPS.ew,
               &GPS.lock, &GPS.satelites,
               &GPS.hdop,
               &GPS.msl_altitude, &GPS.msl_units);
        GPS.dec_latitude  = GPS_nmea_to_dec(GPS.nmea_latitude,  GPS.ns);
        GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
    }
    else if (!strncmp(GPSstrParse, "$GPRMC", 6)) {
        sscanf(GPSstrParse,
               "$GPRMC,%f,%*c,%f,%c,%f,%c,%f,%f,%d",
               &GPS.utc_time,
               &GPS.nmea_latitude, &GPS.ns,
               &GPS.nmea_longitude, &GPS.ew,
               &GPS.speed_k, &GPS.course_t,
               &GPS.date);
    }
    else if (!strncmp(GPSstrParse, "$GPGLL", 6)) {
        sscanf(GPSstrParse,
               "$GPGLL,%f,%c,%f,%c,%f,%c",
               &GPS.nmea_latitude, &GPS.ns,
               &GPS.nmea_longitude, &GPS.ew,
               &GPS.utc_time, &GPS.gll_status);
    }
    else if (!strncmp(GPSstrParse, "$GPVTG", 6)) {
        sscanf(GPSstrParse,
               "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c",
               &GPS.course_t, &GPS.course_t_unit,
               &GPS.course_m, &GPS.course_m_unit,
               &GPS.speed_k, &GPS.speed_k_unit,
               &GPS.speed_km, &GPS.speed_km_unit);
    }
}

float GPS_nmea_to_dec(float deg_coord, char nsew)
{
    int   degree  = (int)(deg_coord / 100);
    float minutes = deg_coord - degree * 100;
    float decimal = degree + (minutes / 60.0f);
    if (nsew == 'S' || nsew == 'W')
        decimal *= -1;
    return decimal;
}
