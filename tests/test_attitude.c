/* test_attitude.c - host unit tests for the complementary attitude filter */
#include "attitude.h"
#include "kalman.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD ((float)M_PI / 180.0f)
#define DEG (180.0f / (float)M_PI)

static int fails = 0;
#define CHECK(cond, msg) do { \
    if (cond) printf("PASS: %s\n", msg); \
    else { printf("FAIL: %s\n", msg); fails++; } \
} while (0)

static unsigned long long rng_state = 424242ULL;
static double frand(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (double)(rng_state >> 11) / 9007199254740992.0;
}
static double gauss(void) {
    double u1 = frand(), u2 = frand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* body-вектор спокою для заданих roll/pitch у нашій конвенції */
static void rest_vector(float roll, float pitch, float *ax, float *ay, float *az) {
    *ax = ATT_G * sinf(pitch);
    *ay = -ATT_G * sinf(roll) * cosf(pitch);
    *az = -ATT_G * cosf(roll) * cosf(pitch);
}

/* test j: static tilt roll=5, pitch=-3 - converges within 2 s */
static void test_static_tilt(void) {
    AttitudeFilter att;
    ATT_Init(&att);
    float ax, ay, az;
    rest_vector(5.0f * RAD, -3.0f * RAD, &ax, &ay, &az);

    for (int i = 0; i < 200; i++) { /* 2 s @ 100 Hz */
        ATT_Update(&att,
                   ax + 0.02f * (float)gauss(), ay + 0.02f * (float)gauss(),
                   az + 0.02f * (float)gauss(),
                   0.001f * (float)gauss(), 0.001f * (float)gauss(),
                   0.001f * (float)gauss(), 0.01f);
    }
    printf("  [j] roll=%.2f deg (want 5), pitch=%.2f deg (want -3)\n",
           att.roll * DEG, att.pitch * DEG);
    CHECK(fabsf(att.roll * DEG - 5.0f) < 0.3f, "j) roll converges to 5 deg");
    CHECK(fabsf(att.pitch * DEG + 3.0f) < 0.3f, "j) pitch converges to -3 deg");
}

/* test k: 90 deg/s about z for 1 s - yaw changes by ~90 deg */
static void test_yaw_rate(void) {
    AttitudeFilter att;
    ATT_Init(&att);
    float ax, ay, az;
    rest_vector(0.0f, 0.0f, &ax, &ay, &az);
    ATT_Update(&att, ax, ay, az, 0, 0, 0, 0.01f); /* init */

    for (int i = 0; i < 100; i++) {
        ATT_Update(&att, ax, ay, az, 0.0f, 0.0f, 90.0f * RAD, 0.01f);
    }
    printf("  [k] yaw=%.2f deg (want 90)\n", att.yaw * DEG);
    CHECK(fabsf(att.yaw * DEG - 90.0f) < 2.0f, "k) yaw integrates 90 deg/s correctly");
}

/* test l (key): our real rest vector (0.065,-0.57,-9.68) - tilt compensated */
static void test_real_tilt_compensation(void) {
    AttitudeFilter att;
    ATT_Init(&att);
    const float ax = 0.065f, ay = -0.57f, az = -9.68f;

    for (int i = 0; i < 300; i++) {
        ATT_Update(&att,
                   ax + 0.01f * (float)gauss(), ay + 0.01f * (float)gauss(),
                   az + 0.015f * (float)gauss(),
                   0.001f * (float)gauss(), 0.001f * (float)gauss(),
                   0.001f * (float)gauss(), 0.01f);
    }
    float aE, aN;
    ATT_BodyToENU(&att, ax, ay, az, &aE, &aN);
    printf("  [l] roll=%.2f pitch=%.2f deg; residual aE=%.4f aN=%.4f m/s^2\n",
           att.roll * DEG, att.pitch * DEG, aE, aN);
    CHECK(fabsf(aE) < 0.05f && fabsf(aN) < 0.05f,
          "l) real 3.3 deg tilt compensated (residual < 0.05 m/s^2)");
}

/* test m: KF fed with attitude-compensated aE/aN does not degrade vs zeros */
static void test_kf_integration(void) {
    AttitudeFilter att;
    ATT_Init(&att);
    KalmanFilter kf_att, kf_zero;
    KF_Init(&kf_att, 0.3f, 1.5f);
    KF_Init(&kf_zero, 0.3f, 1.5f);

    const float ax0 = 0.065f, ay0 = -0.57f, az0 = -9.68f; /* static, tilted */
    double se_att = 0.0, se_zero = 0.0;
    int n = 0;

    for (int i = 0; i < 6000; i++) { /* 60 s */
        float ax = ax0 + 0.05f * (float)gauss();
        float ay = ay0 + 0.05f * (float)gauss();
        float az = az0 + 0.05f * (float)gauss();
        float gx = 0.001f * (float)gauss(), gy = 0.001f * (float)gauss(),
              gz = 0.001f * (float)gauss();
        ATT_Update(&att, ax, ay, az, gx, gy, gz, 0.01f);
        float aE, aN;
        ATT_BodyToENU(&att, ax, ay, az, &aE, &aN);
        KF_Predict(&kf_att, 0.01f, aE, aN);
        KF_Predict(&kf_zero, 0.01f, 0.0f, 0.0f);

        if ((i + 1) % 100 == 0) {
            float zE = (float)(1.5 * gauss()), zN = (float)(1.5 * gauss());
            KF_UpdateGated(&kf_att, zE, zN, 1.0f, 0.0f);
            KF_UpdateGated(&kf_zero, zE, zN, 1.0f, 0.0f);
            if ((i + 1) * 0.01 > 10.0) {
                float pE, pN;
                KF_GetPosition(&kf_att, &pE, &pN);
                se_att += pE * pE + pN * pN;
                KF_GetPosition(&kf_zero, &pE, &pN);
                se_zero += pE * pE + pN * pN;
                n++;
            }
        }
    }
    double rmse_att = sqrt(se_att / n), rmse_zero = sqrt(se_zero / n);
    printf("  [m] RMSE: with attitude=%.3f m, with zeros=%.3f m\n", rmse_att, rmse_zero);
    CHECK(rmse_att <= rmse_zero * 1.2 + 0.1,
          "m) attitude-fed KF does not degrade vs aE=aN=0");
}

int main(void) {
    test_static_tilt();
    test_yaw_rate();
    test_real_tilt_compensation();
    test_kf_integration();
    printf(fails ? "\n%d TEST(S) FAILED\n" : "\nALL TESTS PASSED\n", fails);
    return fails ? 1 : 0;
}
