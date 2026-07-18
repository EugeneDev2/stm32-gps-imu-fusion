/* gps.h - NMEA GPS receiver (USART1 + circular DMA, idle-line framing) */
#ifndef GPS_H
#define GPS_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define GPS_RX_BUFFER_SIZE 512   /* circular DMA buffer */
#define GPS_LINE_MAX       128   /* max NMEA sentence length */
#define GPS_LINE_QUEUE_LEN 8     /* completed-lines ring depth */

typedef struct {
    bool   valid;      /* RMC status == 'A' */
    double lat;        /* decimal degrees, +N / -S; double: float дає квант ~0.3-0.9 м */
    double lon;        /* decimal degrees, +E / -W */
    float  speed_mps;  /* ground speed, m/s */
    float  course_deg; /* course over ground */
    float  time_utc;   /* hhmmss.sss as received */
} GPS_RMCData;

void GPS_Init(UART_HandleTypeDef *huart);
bool GPS_GetRMC(GPS_RMCData *out);
bool GPS_PopLine(char *out, size_t maxlen);
bool GPS_ParseRMC(const char *line, GPS_RMCData *out);

/* delegates from HAL callbacks in main.c */
void GPS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void GPS_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* GPS_H */
