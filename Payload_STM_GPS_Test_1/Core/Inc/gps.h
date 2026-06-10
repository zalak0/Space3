#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32h7xx_hal.h"

#define GPS_USART   &huart4
#define GPSBUFSIZE  128

typedef struct {
    float utc_time;
    float nmea_latitude;
    float nmea_longitude;
    float dec_latitude;
    float dec_longitude;
    char  ns;
    char  ew;
    int   lock;
    int   satelites;
    float hdop;
    float msl_altitude;
    char  msl_units;
    float speed_k;
    float speed_km;
    char  speed_k_unit;
    char  speed_km_unit;
    float course_t;
    char  course_t_unit;
    float course_m;
    char  course_m_unit;
    int   date;
    char  gll_status;
} GPS_t;

extern GPS_t GPS;
extern uint8_t rx_data;
extern uint8_t rx_buffer[GPSBUFSIZE];
extern uint8_t rx_index;
extern volatile uint8_t gps_line_ready;
extern uint8_t print_buffer[GPSBUFSIZE];

void  GPS_Init(void);
void  GPS_UART_CallBack(void);
int   GPS_validate(char *nmeastr);
void  GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);

#endif
