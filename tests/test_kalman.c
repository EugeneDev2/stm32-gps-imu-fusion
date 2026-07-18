/* test_kalman.c - host unit tests for the 4-state ENU Kalman filter */
#include "kalman.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int fails = 0;
#define CHECK(cond, msg) do { \
    if (cond) printf("PASS: %s\n", msg); \
    else { printf("FAIL: %s\n", msg); fails++; } \
} while (0)

/* deterministic xorshift64 + Box-Muller: same numbers on every platform */
static unsigned long long rng_state = 88172645463325252ULL;

static double frand(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (double)(rng_state >> 11) / 9007199254740992.0; /* [0,1) */
}

static double gauss(void) {
    double u1 = frand(), u2 = frand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

#define DT          0.01f   /* IMU 100 Hz */
#define GPS_EVERY   100     /* GPS 1 Hz */
#define ACC_NOISE   0.05
#define ACC_BIAS    0.2
#define GPS_NOISE   1.5
#define SIGMA_A     0.3f
#define SIGMA_G     1.5f

/* test a: constant velocity east, noisy+biased IMU, noisy GPS.
   Filter position RMSE must beat raw GPS RMSE. */
static void test_synthetic_motion(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);

    const double vE = 5.0;
    double sum_kf = 0.0, sum_gps = 0.0;
    int n = 0;

    for (int i = 0; i < 6000; i++) {
        double t = (i + 1) * (double)DT;
        double true_pE = vE * t;
        double aE = ACC_BIAS + ACC_NOISE * gauss(); /* true accel = 0 */
        double aN = ACC_BIAS + ACC_NOISE * gauss();
        KF_Predict(&kf, DT, (float)aE, (float)aN);

        if ((i + 1) % GPS_EVERY == 0) {
            double zE = true_pE + GPS_NOISE * gauss();
            double zN = 0.0 + GPS_NOISE * gauss();
            KF_Update(&kf, (float)zE, (float)zN);
            if (t > 10.0) { /* after convergence */
                float pE, pN;
                KF_GetPosition(&kf, &pE, &pN);
                double eK = pE - true_pE, nK = pN - 0.0;
                double eG = zE - true_pE, nG = zN - 0.0;
                sum_kf += eK * eK + nK * nK;
                sum_gps += eG * eG + nG * nG;
                n++;
            }
        }
    }
    double rmse_kf = sqrt(sum_kf / n), rmse_gps = sqrt(sum_gps / n);
    float vEest, vNest;
    KF_GetVelocity(&kf, &vEest, &vNest);
    printf("  [a] RMSE: kf=%.3f m, raw gps=%.3f m; vE est=%.2f (true 5.0)\n",
           rmse_kf, rmse_gps, vEest);
    CHECK(rmse_kf < rmse_gps, "a) filter RMSE < raw GPS RMSE");
    CHECK(fabs(vEest - vE) < 1.0, "a) velocity estimate near truth");
}

/* test b: P diagonal must not grow without bound under regular updates */
static void test_convergence(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);
    float diag_20s[4] = {0}, diag_end[4] = {0};

    for (int i = 0; i < 6000; i++) {
        KF_Predict(&kf, DT, 0.0f, 0.0f);
        if ((i + 1) % GPS_EVERY == 0) {
            KF_Update(&kf, 0.0f, 0.0f);
            if ((i + 1) == 2000) KF_GetPDiag(&kf, diag_20s);
        }
    }
    KF_GetPDiag(&kf, diag_end);
    printf("  [b] P diag @20s: %.3f %.3f %.3f %.3f; @60s: %.3f %.3f %.3f %.3f\n",
           diag_20s[0], diag_20s[1], diag_20s[2], diag_20s[3],
           diag_end[0], diag_end[1], diag_end[2], diag_end[3]);
    CHECK(diag_end[0] <= diag_20s[0] + 1e-3f && diag_end[2] <= diag_20s[2] + 1e-3f,
          "b) P diagonal settles (no unbounded growth)");
}

/* test c: 30 s GPS outage - no NaN/inf, P grows monotonically */
static void test_gps_outage(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);

    for (int i = 0; i < 3000; i++) { /* 30 s with GPS */
        KF_Predict(&kf, DT, 0.0f, 0.0f);
        if ((i + 1) % GPS_EVERY == 0) KF_Update(&kf, 0.0f, 0.0f);
    }

    float prev[4], cur[4];
    KF_GetPDiag(&kf, prev);
    int monotone = 1, finite = 1;
    for (int i = 0; i < 3000; i++) { /* 30 s without GPS */
        KF_Predict(&kf, DT, 0.0f, 0.0f);
        KF_GetPDiag(&kf, cur);
        for (int j = 0; j < 4; j++) {
            if (cur[j] + 1e-9f < prev[j]) monotone = 0;
            if (!isfinite(cur[j])) finite = 0;
            prev[j] = cur[j];
        }
        if (!isfinite(kf.x[0]) || !isfinite(kf.x[1]) ||
            !isfinite(kf.x[2]) || !isfinite(kf.x[3])) finite = 0;
    }
    printf("  [c] P diag after 30s outage: %.2f %.2f %.2f %.2f\n",
           cur[0], cur[1], cur[2], cur[3]);
    CHECK(finite, "c) no NaN/inf during 30s GPS outage");
    CHECK(monotone, "c) P grows monotonically during outage");
}

/* test d: static truth + accel bias - position stays within 3*sigma_gps */
static void test_static_bias(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);

    float max_dist = 0.0f;
    for (int i = 0; i < 12000; i++) { /* 120 s */
        double aE = ACC_BIAS + ACC_NOISE * gauss();
        double aN = ACC_BIAS + ACC_NOISE * gauss();
        KF_Predict(&kf, DT, (float)aE, (float)aN);
        if ((i + 1) % GPS_EVERY == 0) {
            KF_Update(&kf, (float)(GPS_NOISE * gauss()), (float)(GPS_NOISE * gauss()));
            if ((i + 1) * DT > 30.0f) {
                float pE, pN;
                KF_GetPosition(&kf, &pE, &pN);
                float d = sqrtf(pE * pE + pN * pN);
                if (d > max_dist) max_dist = d;
            }
        }
    }
    printf("  [d] max distance from truth with bias %.1f m/s^2: %.2f m (limit %.2f)\n",
           ACC_BIAS, max_dist, 3.0f * SIGMA_G);
    CHECK(max_dist < 3.0f * SIGMA_G, "d) static position within 3 sigma despite accel bias");
}

/* helper: converge filter on static truth with clean GPS via gated updates */
static void settle_static(KalmanFilter *kf, int seconds) {
    for (int i = 0; i < seconds * 100; i++) {
        KF_Predict(kf, DT, 0.0f, 0.0f);
        if ((i + 1) % GPS_EVERY == 0) {
            KF_UpdateGated(kf, (float)(0.3 * gauss()), (float)(0.3 * gauss()), 1.0f, 0.0f);
        }
    }
}

/* test e: single 100 m teleport is rejected, filter does not twitch */
static void test_gate_teleport(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);
    settle_static(&kf, 30);

    float pE0, pN0, pE1, pN1;
    KF_GetPosition(&kf, &pE0, &pN0);
    int acc = KF_UpdateGated(&kf, 100.0f, 0.0f, 1.0f, 0.0f); /* teleport, Doppler=0 */
    KF_GetPosition(&kf, &pE1, &pN1);
    float moved = sqrtf((pE1 - pE0) * (pE1 - pE0) + (pN1 - pN0) * (pN1 - pN0));
    printf("  [e] teleport: accepted=%d, position moved %.3f m\n", acc, moved);
    CHECK(acc == 0, "e) 100 m teleport rejected");
    CHECK(moved < 0.01f, "e) filter did not move on rejected update");

    /* після викиду нормальні виміри знову приймаються за <= 3 fix-и */
    int resumed = 0;
    for (int k = 0; k < 3; k++) {
        for (int i = 0; i < GPS_EVERY; i++) KF_Predict(&kf, DT, 0.0f, 0.0f);
        if (KF_UpdateGated(&kf, 0.1f, -0.1f, 1.0f, 0.0f)) { resumed = 1; break; }
    }
    CHECK(resumed, "e) normal updates resume after the glitch");
}

/* test f: honest maneuver (2 m/s^2, consistent IMU + Doppler) is accepted */
static void test_gate_maneuver(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);
    settle_static(&kf, 30);

    int accepted = 0, total = 0;
    double v = 0.0, p = 0.0;
    for (int i = 0; i < 1000; i++) { /* 10 s розгону */
        double a = 2.0 + ACC_NOISE * gauss();
        KF_Predict(&kf, DT, (float)a, 0.0f); /* IMU бачить реальне прискорення */
        p += v * DT + 0.5 * 2.0 * DT * DT;
        v += 2.0 * DT;
        if ((i + 1) % GPS_EVERY == 0) {
            float zE = (float)(p + GPS_NOISE * gauss());
            float zN = (float)(GPS_NOISE * gauss());
            accepted += KF_UpdateGated(&kf, zE, zN, 1.0f, (float)v);
            total++;
        }
    }
    printf("  [f] maneuver: %d/%d updates accepted\n", accepted, total);
    CHECK(accepted == total, "f) honest maneuver fully accepted");
}

/* test g: permanent 50 m shift - filter re-locks within KF_GATE_MAX_REJECTS fixes */
static void test_gate_permanent_shift(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);
    settle_static(&kf, 30);

    int first_accept = -1;
    for (int k = 1; k <= 10; k++) {
        for (int i = 0; i < GPS_EVERY; i++) KF_Predict(&kf, DT, 0.0f, 0.0f);
        int acc = KF_UpdateGated(&kf, 50.0f + (float)(0.3 * gauss()),
                                 (float)(0.3 * gauss()), 1.0f, 0.0f);
        if (acc && first_accept < 0) first_accept = k;
    }
    float pE, pN;
    KF_GetPosition(&kf, &pE, &pN);
    printf("  [g] permanent shift: first accept at fix #%d, pos after 10 fixes = (%.2f, %.2f)\n",
           first_accept, pE, pN);
    CHECK(first_accept > 0 && first_accept <= KF_GATE_MAX_REJECTS,
          "g) re-locks within KF_GATE_MAX_REJECTS fixes");
    CHECK(fabsf(pE - 50.0f) < 5.0f, "g) position converges to the new location");
}

/* test h: 60 s outage with residual velocity - filter stops instead of running away */
static void test_outage_decay(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);
    settle_static(&kf, 30);
    kf.x[2] = 0.3f; /* залишкова швидкість, як від хвоста дрейфу */
    kf.x[3] = 0.0f;

    float pE0, pN0;
    KF_GetPosition(&kf, &pE0, &pN0);
    float pos_55s = 0.0f;
    for (int i = 0; i < 6000; i++) { /* 60 s, без апдейтів */
        KF_Predict(&kf, DT, 0.0f, 0.0f);
        if (i == 5500) KF_GetPosition(&kf, &pos_55s, &pN0);
    }
    float pE1, pN1, vE1, vN1;
    KF_GetPosition(&kf, &pE1, &pN1);
    KF_GetVelocity(&kf, &vE1, &vN1);
    float shift = fabsf(pE1 - pE0);
    float tail_creep = fabsf(pE1 - pos_55s); /* рух за останні 5 с outage */
    printf("  [h] 60s outage, v0=0.3: shift=%.2f m (no decay would be 18.0), "
           "v_end=%.4f, last-5s creep=%.3f m\n", shift, vE1, tail_creep);
    CHECK(shift < 6.0f, "h) position stops during long outage (no runaway)");
    CHECK(fabsf(vE1) < 0.02f, "h) velocity decayed to ~zero");
    CHECK(tail_creep < 0.1f, "h) position stationary by end of outage");
}

/* test i: 3 s gap - velocity untouched, behaviour as with continuous fixes */
static void test_short_gap(void) {
    KalmanFilter kf;
    KF_Init(&kf, SIGMA_A, SIGMA_G);
    settle_static(&kf, 30);
    kf.x[2] = 0.3f;

    float vE_before, vN_before, vE_after, vN_after;
    KF_GetVelocity(&kf, &vE_before, &vN_before);
    for (int i = 0; i < 300; i++) { /* 3 s без апдейтів */
        KF_Predict(&kf, DT, 0.0f, 0.0f);
    }
    KF_GetVelocity(&kf, &vE_after, &vN_after);
    printf("  [i] 3s gap: v before=%.4f after=%.4f\n", vE_before, vE_after);
    CHECK(fabsf(vE_after - vE_before) < 1e-6f && fabsf(vN_after - vN_before) < 1e-6f,
          "i) short gap (<=5s) leaves velocity untouched");

    /* і перший fix після паузи приймається */
    CHECK(KF_UpdateGated(&kf, 0.9f, 0.1f, 4.0f, 0.0f) == 1,
          "i) first fix after short gap accepted");
}

int main(void) {
    test_synthetic_motion();
    test_convergence();
    test_gps_outage();
    test_static_bias();
    test_gate_teleport();
    test_gate_maneuver();
    test_gate_permanent_shift();
    test_outage_decay();
    test_short_gap();
    printf(fails ? "\n%d TEST(S) FAILED\n" : "\nALL TESTS PASSED\n", fails);
    return fails ? 1 : 0;
}
