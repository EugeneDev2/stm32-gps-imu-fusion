/* attitude.c - complementary attitude filter, no HAL.
 *
 * Конвенція виведена під НАШ сенсор: у спокої він читає (ax,ay,az)~(0,0,-g).
 * Модель: f_body = R^T * f_enu, f_enu(спокій) = (0,0,-g), R = Rz(yaw)*Ry(pitch)*Rx(roll).
 * Звідси у спокої: ax = g*sin(pitch), ay = -g*sin(roll)*cos(pitch),
 * az = -g*cos(roll)*cos(pitch), і accel-оцінки кутів:
 *   roll_acc  = atan2(-ay, -az)
 *   pitch_acc = atan2(ax, sqrt(ay^2+az^2))
 * (класичні atan2(ay,az)/atan2(-ax,...) розраховані на сенсор з az=+g у спокої
 * і на наших даних дають кут ~180 градусів). */
#include "attitude.h"
#include <math.h>

#define ATT_PI 3.14159265358979323846f

static float wrap_pi(float a) {
    while (a > ATT_PI)  a -= 2.0f * ATT_PI;
    while (a < -ATT_PI) a += 2.0f * ATT_PI;
    return a;
}

void ATT_Init(AttitudeFilter *att) {
    att->roll = 0.0f;
    att->pitch = 0.0f;
    att->yaw = 0.0f;
    att->initialized = 0;
}

void ATT_Update(AttitudeFilter *att, float ax, float ay, float az,
                float gx, float gy, float gz, float dt) {
    const float roll_acc = atan2f(-ay, -az);
    const float pitch_acc = atan2f(ax, sqrtf(ay * ay + az * az));

    if (!att->initialized) {
        att->roll = roll_acc;
        att->pitch = pitch_acc;
        att->yaw = 0.0f;
        att->initialized = 1;
        return;
    }

    att->roll = ATT_ALPHA * (att->roll + gx * dt) + (1.0f - ATT_ALPHA) * roll_acc;
    att->pitch = ATT_ALPHA * (att->pitch + gy * dt) + (1.0f - ATT_ALPHA) * pitch_acc;
    att->yaw = wrap_pi(att->yaw + gz * dt);
}

void ATT_CorrectYawFromCourse(AttitudeFilter *att, float course_deg, float speed_mps) {
    if (!att->initialized || speed_mps <= ATT_MIN_SPEED) {
        return; /* на стоянці COG - сміття; yaw лишається інтегралом гіро */
    }
    /* COG: градуси CW від півночі -> мат. кут CCW від сходу */
    const float target = (90.0f - course_deg) * (ATT_PI / 180.0f);
    att->yaw = wrap_pi(att->yaw + ATT_YAW_GAIN * wrap_pi(target - att->yaw));
}

void ATT_BodyToENU(const AttitudeFilter *att, float ax, float ay, float az,
                   float *aE, float *aN) {
    const float cf = cosf(att->roll),  sf = sinf(att->roll);
    const float ct = cosf(att->pitch), st = sinf(att->pitch);
    const float cp = cosf(att->yaw),   sp = sinf(att->yaw);

    /* R = Rz(yaw)*Ry(pitch)*Rx(roll), рядки E і N */
    *aE = (cp * ct) * ax + (cp * st * sf - sp * cf) * ay + (cp * st * cf + sp * sf) * az;
    *aN = (sp * ct) * ax + (sp * st * sf + cp * cf) * ay + (sp * st * cf - cp * sf) * az;
}
