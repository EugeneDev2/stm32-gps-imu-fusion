/* accel.h - LSM6DS3 accelerometer/gyroscope driver */
#ifndef ACCEL_H
#define ACCEL_H

#include "main.h"
#include <stdbool.h>

#define LSM6DS3_ADDR    (0x6B << 1)   /* 7-bit адреса 0x6B у форматі HAL */
#define WHO_AM_I_REG    0x0F          /* очікується 0x69 */
#define CTRL1_XL        0x10
#define CTRL2_G         0x11
#define CTRL3_C         0x12
#define OUTX_L_G        0x22          /* початок блоку даних: гіро, далі акселерометр */
#define OUTX_L_XL       0x28

extern float accel_x, accel_y, accel_z; /* м/с² */
extern float gyro_x, gyro_y, gyro_z;    /* рад/с */

void ACCEL_Init(I2C_HandleTypeDef *hi2c);
bool ACCEL_Read(void);

#endif /* ACCEL_H */
