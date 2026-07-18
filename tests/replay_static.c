/* replay_static.c - replay a recorded telemetry log through the Kalman filter.
 *
 * Usage: replay_static [input.csv] [output.csv] [gate]
 * Input rows: t_ms,ax,ay,az,gx,gy,gz,fix,east,north,spd[,...] (~10 Hz, GPS 1 Hz);
 * extra columns (live-log kf fields) are ignored.
 * Predict runs with aE=aN=0 (no orientation yet); Update fires only when
 * (east, north) change, i.e. on real GPS fixes. With the "gate" argument
 * updates go through KF_UpdateGated (Mahalanobis + Doppler cross-check).
 * Output: t_ms,raw_east,raw_north,kf_east,kf_north
 */
#include "kalman.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv) {
    const char *in_path = (argc > 1) ? argv[1] : "logs/static_test.csv";
    const char *out_path = (argc > 2) ? argv[2] : "logs/static_test_kf.csv";
    const int gated = (argc > 3) && (strcmp(argv[3], "gate") == 0);

    FILE *in = fopen(in_path, "r");
    if (!in) { fprintf(stderr, "cannot open %s\n", in_path); return 1; }
    FILE *out = fopen(out_path, "w");
    if (!out) { fprintf(stderr, "cannot open %s\n", out_path); fclose(in); return 1; }
    fprintf(out, "t_ms,raw_east,raw_north,kf_east,kf_north\n");

    KalmanFilter kf;
    KF_Init(&kf, 0.3f, 1.5f);

    char line[256];
    int started = 0, rows = 0, fixes = 0, rejected = 0;
    double prev_t = 0.0, prev_e = 0.0, prev_n = 0.0, prev_fix_t = 0.0;

    while (fgets(line, sizeof(line), in)) {
        double v[11];
        if (!isdigit((unsigned char)line[0])) continue;
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6],
                   &v[7], &v[8], &v[9], &v[10]) != 11) continue;
        double t = v[0], e = v[8], n = v[9];

        if (!started) {
            kf.x[0] = (float)e; /* стартуємо з першого відомого положення */
            kf.x[1] = (float)n;
            started = 1;
            prev_fix_t = t;
        } else {
            double dt = (t - prev_t) / 1000.0;
            if (dt > 0.0 && dt < 1.0) {
                KF_Predict(&kf, (float)dt, 0.0f, 0.0f);
            }
            if (e != prev_e || n != prev_n) {
                if (gated) {
                    float dt_fix = (float)((t - prev_fix_t) / 1000.0);
                    if (!KF_UpdateGated(&kf, (float)e, (float)n, dt_fix, (float)v[10])) {
                        rejected++;
                    }
                } else {
                    KF_Update(&kf, (float)e, (float)n);
                }
                prev_fix_t = t;
                fixes++;
            }
        }
        prev_t = t; prev_e = e; prev_n = n;
        rows++;

        float pE, pN;
        KF_GetPosition(&kf, &pE, &pN);
        fprintf(out, "%.0f,%.2f,%.2f,%.3f,%.3f\n", t, e, n, pE, pN);
    }
    fclose(in);
    fclose(out);
    if (gated) {
        printf("replayed %d rows, %d GPS fixes (%d rejected by gate) -> %s\n",
               rows, fixes, rejected, out_path);
    } else {
        printf("replayed %d rows, %d GPS fixes -> %s\n", rows, fixes, out_path);
    }
    return 0;
}
