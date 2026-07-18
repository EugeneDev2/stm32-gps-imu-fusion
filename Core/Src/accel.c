/* accel.c - LSM6DS3 accelerometer/gyroscope driver */
#include "accel.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Скінченний таймаут I2C: залипла шина не повинна вішати плату назавжди */
#define LSM6DS3_I2C_TIMEOUT_MS 100

/* Чутливість за даташитом для конфігурації нижче:
   ±2g -> 0.061 mg/LSB, 245 dps -> 8.75 mdps/LSB */
#define ACCEL_SENS_MS2 (0.061e-3f * 9.80665f)              /* LSB -> м/с² */
#define GYRO_SENS_RADS (8.75e-3f * 3.14159265f / 180.0f)   /* LSB -> рад/с */

/* Калібрування нуля гіро на старті: плата має бути нерухома ~1 с */
#define GYRO_CAL_SAMPLES 100

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
static I2C_HandleTypeDef *accel_hi2c;
static char uartBuf[80];
static float gyro_bias_x, gyro_bias_y, gyro_bias_z;

static HAL_StatusTypeDef LSM6DS3_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t TxBuffer[2];
    TxBuffer[0] = reg;
    TxBuffer[1] = data;
    return HAL_I2C_Master_Transmit(accel_hi2c, LSM6DS3_ADDR, TxBuffer, 2, LSM6DS3_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef LSM6DS3_ReadReg(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(accel_hi2c, LSM6DS3_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, LSM6DS3_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef LSM6DS3_ReadMulti(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(accel_hi2c, LSM6DS3_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, LSM6DS3_I2C_TIMEOUT_MS);
}

static float calcAccel(int16_t raw) {
    return (float)raw * ACCEL_SENS_MS2;
}

static float calcGyro(int16_t raw) {
    return (float)raw * GYRO_SENS_RADS;
}

void ACCEL_Init(I2C_HandleTypeDef *hi2c) {
    accel_hi2c = hi2c;
    uint8_t who_am_i = 0;

    /* Помилки I2C не фатальні: плата має жити далі (GPS працює без акселерометра) */
    if (LSM6DS3_ReadReg(WHO_AM_I_REG, &who_am_i) != HAL_OK) {
        sprintf(uartBuf, "Accel: I2C timeout/fail\r\n");
        Safe_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf));
        return;
    }
    if (who_am_i != 0x69) {
        sprintf(uartBuf, "LSM6DS3 not detected! WHO_AM_I = 0x%02X\r\n", who_am_i);
        Safe_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf));
        return;
    }
    sprintf(uartBuf, "Accel: Connection OK - LSM6DS3 detected. WHO_AM_I = 0x%02X\r\n", who_am_i);
    Safe_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf));

    /* CTRL3_C: BDU=1 (узгоджені high/low байти), IF_INC=1 (автоінкремент адреси
       для burst-читання). CTRL1_XL/CTRL2_G: 104 Hz — достатньо при опитуванні 10 Гц */
    if (LSM6DS3_WriteReg(CTRL3_C, 0x44) != HAL_OK ||
        LSM6DS3_WriteReg(CTRL1_XL, 0x40) != HAL_OK ||   /* 104 Hz, +-2g */
        LSM6DS3_WriteReg(CTRL2_G, 0x40) != HAL_OK) {    /* 104 Hz, 245 dps */
        sprintf(uartBuf, "Accel: Configuration Failed\r\n");
        Safe_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf));
        return;
    }

    sprintf(uartBuf, "Accel: Gyro calibration, keep still...\r\n");
    Safe_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf));
    HAL_Delay(50); /* даємо фільтрам сенсора устаканитись після ввімкнення */

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int good = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        uint8_t raw[6];
        if (LSM6DS3_ReadMulti(OUTX_L_G, raw, sizeof(raw)) == HAL_OK) {
            sum_x += calcGyro((int16_t)((raw[1] << 8) | raw[0]));
            sum_y += calcGyro((int16_t)((raw[3] << 8) | raw[2]));
            sum_z += calcGyro((int16_t)((raw[5] << 8) | raw[4]));
            good++;
        }
        HAL_Delay(10); /* ODR 104 Hz: новий семпл кожні ~9.6 мс */
    }
    if (good >= GYRO_CAL_SAMPLES / 2) {
        gyro_bias_x = sum_x / good;
        gyro_bias_y = sum_y / good;
        gyro_bias_z = sum_z / good;
        sprintf(uartBuf, "Accel: Gyro bias [rad/s]: %.4f, %.4f, %.4f\r\n",
                gyro_bias_x, gyro_bias_y, gyro_bias_z);
    } else {
        sprintf(uartBuf, "Accel: Gyro calibration failed, bias = 0\r\n");
    }
    Safe_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf));
}

/* Оновлює accel_/gyro_ глобали; вивід даних робить main loop (CSV) */
bool ACCEL_Read(void) {
    uint8_t raw[12];

    /* Один burst з OUTX_L_G завдяки IF_INC: байти 0..5 - гіро, 6..11 - акселерометр */
    if (LSM6DS3_ReadMulti(OUTX_L_G, raw, sizeof(raw)) != HAL_OK) {
        sprintf(uartBuf, "[%lu] Error reading LSM6DS3 data\r\n", HAL_GetTick());
        HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);
        return false;
    }

    gyro_x = calcGyro((int16_t)((raw[1] << 8) | raw[0])) - gyro_bias_x;
    gyro_y = calcGyro((int16_t)((raw[3] << 8) | raw[2])) - gyro_bias_y;
    gyro_z = calcGyro((int16_t)((raw[5] << 8) | raw[4])) - gyro_bias_z;

    accel_x = calcAccel((int16_t)((raw[7] << 8) | raw[6]));
    accel_y = calcAccel((int16_t)((raw[9] << 8) | raw[8]));
    accel_z = calcAccel((int16_t)((raw[11] << 8) | raw[10]));

    return true;
}
