/* attitude.h - complementary attitude filter (roll/pitch from accel+gyro,
 * yaw from gyro with GPS course correction). Pure math, no HAL. */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#define ATT_ALPHA      0.98f   /* вага гіро в комплементарному фільтрі */
#define ATT_YAW_GAIN   0.2f    /* частка корекції yaw на один GPS fix */
#define ATT_MIN_SPEED  2.5f    /* м/с; нижче COG - сміття (мультипас у статиці дає до ~3), yaw лише інтегрується */
#define ATT_G          9.80665f

typedef struct {
    float roll;   /* rad, + = нахил у бік -Y сенсора (наш AY<0 у спокої) */
    float pitch;  /* rad */
    float yaw;    /* rad, мат. конвенція: CCW від осі East; 0 до першої GPS-корекції */
    int initialized;
} AttitudeFilter;

void ATT_Init(AttitudeFilter *att);
void ATT_Update(AttitudeFilter *att, float ax, float ay, float az,
                float gx, float gy, float gz, float dt);
void ATT_CorrectYawFromCourse(AttitudeFilter *att, float course_deg, float speed_mps);

/* Поворот виміряного прискорення body->ENU; горизонтальні компоненти.
 * Гравітація [0,0,-g] живе тільки у вертикалі, тому aE/aN її не містять. */
void ATT_BodyToENU(const AttitudeFilter *att, float ax, float ay, float az,
                   float *aE, float *aN);

#endif /* ATTITUDE_H */
