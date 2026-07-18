/* gps.c - NMEA GPS receiver (USART1 + circular DMA, idle-line framing) */
#include "gps.h"
#include <stdlib.h>
#include <string.h>

#define KNOTS_TO_MPS 0.514444f

static UART_HandleTypeDef *gps_huart;
static uint8_t GPS_Rxdata[GPS_RX_BUFFER_SIZE];

/* line assembler: збирає байти з DMA-буфера в завершені NMEA-рядки */
static char line_buf[GPS_LINE_MAX];
static size_t line_len = 0;

/* SPSC-кільце готових рядків: head пише ISR, tail читає main loop */
static char line_queue[GPS_LINE_QUEUE_LEN][GPS_LINE_MAX];
static volatile uint8_t lq_head = 0, lq_tail = 0;

static uint16_t dma_old_pos = 0; /* докуди ми вже вичитали circular DMA-буфер */

static void GPS_StartReceive(void) {
    dma_old_pos = 0;
    line_len = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, GPS_Rxdata, GPS_RX_BUFFER_SIZE);
}

void GPS_Init(UART_HandleTypeDef *huart) {
    gps_huart = huart;
    GPS_StartReceive();
    char msg[] = "GPS: UART initialized\r\n";
    Safe_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg));
}

/* ISR-контекст: один байт у line assembler */
static void gps_feed_byte(uint8_t b) {
    if (b == '\r') {
        return;
    }
    if (b == '\n') {
        if (line_len > 0) {
            line_buf[line_len] = '\0';
            uint8_t next = (uint8_t)((lq_head + 1) % GPS_LINE_QUEUE_LEN);
            if (next != lq_tail) { /* якщо кільце повне - рядок тихо відкидається */
                memcpy(line_queue[lq_head], line_buf, line_len + 1);
                lq_head = next;
            }
            line_len = 0;
        }
        return;
    }
    if (line_len < GPS_LINE_MAX - 1) {
        line_buf[line_len++] = (char)b;
    } else {
        line_len = 0; /* задовгий/битий рядок - відкидаємо цілком */
    }
}

/* HAL кличе на idle/half/full transfer; Size = поточна позиція запису DMA.
   Circular DMA не зупиняється, тому перезапуск прийому тут не потрібен. */
void GPS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart != gps_huart) {
        return;
    }
    /* TC-подія на межі circular-буфера дає Size == GPS_RX_BUFFER_SIZE,
       а dma_old_pos рахується по модулю - без нормалізації цикл нескінченний */
    uint16_t pos = (uint16_t)(Size % GPS_RX_BUFFER_SIZE);
    while (dma_old_pos != pos) {
        gps_feed_byte(GPS_Rxdata[dma_old_pos]);
        dma_old_pos = (uint16_t)((dma_old_pos + 1) % GPS_RX_BUFFER_SIZE);
    }
}

/* ORE/framing/noise: HAL зупиняє прийом - чистимо і стартуємо заново */
void GPS_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart != gps_huart) {
        return;
    }
    HAL_UART_AbortReceive(huart);
    GPS_StartReceive();
}

bool GPS_PopLine(char *out, size_t maxlen) {
    if (lq_tail == lq_head || maxlen == 0) {
        return false;
    }
    strncpy(out, line_queue[lq_tail], maxlen - 1);
    out[maxlen - 1] = '\0';
    lq_tail = (uint8_t)((lq_tail + 1) % GPS_LINE_QUEUE_LEN);
    return true;
}

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* XOR усіх байтів між '$' і '*' має збігтись із двома hex-цифрами після '*' */
static bool nmea_checksum_ok(const char *s) {
    if (*s != '$') {
        return false;
    }
    uint8_t cs = 0;
    const char *p = s + 1;
    while (*p != '\0' && *p != '*') {
        cs ^= (uint8_t)*p++;
    }
    if (*p != '*') {
        return false;
    }
    int hi = hexval(p[1]);
    int lo = (hi >= 0) ? hexval(p[2]) : -1;
    return lo >= 0 && cs == (uint8_t)((hi << 4) | lo);
}

/* Різання по комах З підтримкою порожніх полів (",," -> порожній рядок).
   Модифікує s: коми -> '\0', хвіст чексуми ('*..') відрізається. */
static int nmea_split(char *s, char *fields[], int maxf) {
    int n = 0;
    fields[n++] = s;
    for (char *p = s; *p != '\0' && n < maxf; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        } else if (*p == '*') {
            *p = '\0';
            break;
        }
    }
    return n;
}

/* ddmm.mmmm -> decimal degrees */
static double nmea_to_deg(double v) {
    int deg = (int)(v / 100.0);
    return (double)deg + (v - deg * 100.0) / 60.0;
}

/* true = це RMC-рядок з коректною чексумою (out заповнений);
   out->valid каже, чи є fix. Порожні поля дають valid=false, а не сміття. */
bool GPS_ParseRMC(const char *line, GPS_RMCData *out) {
    if (strncmp(line, "$GNRMC,", 7) != 0 && strncmp(line, "$GPRMC,", 7) != 0) {
        return false;
    }
    if (!nmea_checksum_ok(line)) {
        return false;
    }

    char work[GPS_LINE_MAX];
    strncpy(work, line, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';

    char *f[16] = {0};
    int n = nmea_split(work, f, 16);
    /* RMC: 0-id 1-time 2-status 3-lat 4-N/S 5-lon 6-E/W 7-speed(kn) 8-course */
    if (n < 9) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->time_utc = strtof(f[1], NULL);
    out->valid = (f[2][0] == 'A');

    if (out->valid) {
        if (f[3][0] == '\0' || f[4][0] == '\0' || f[5][0] == '\0' || f[6][0] == '\0') {
            out->valid = false;
            return true;
        }
        out->lat = nmea_to_deg(strtod(f[3], NULL));
        if (f[4][0] == 'S') {
            out->lat = -out->lat;
        }
        out->lon = nmea_to_deg(strtod(f[5], NULL));
        if (f[6][0] == 'W') {
            out->lon = -out->lon;
        }
        out->speed_mps = strtof(f[7], NULL) * KNOTS_TO_MPS;
        out->course_deg = strtof(f[8], NULL);
    }
    return true;
}

/* Вигрібає всі накопичені рядки; true, якщо з минулого виклику прийшов
   новий RMC з валідною чексумою (в out - останній з них) */
bool GPS_GetRMC(GPS_RMCData *out) {
    char line[GPS_LINE_MAX];
    GPS_RMCData rmc;
    bool fresh = false;

    while (GPS_PopLine(line, sizeof(line))) {
        if (GPS_ParseRMC(line, &rmc)) {
            *out = rmc;
            fresh = true;
        }
    }
    return fresh;
}
