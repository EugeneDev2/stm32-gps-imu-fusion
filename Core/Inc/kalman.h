/* kalman.h - 4-state linear Kalman filter for ENU position/velocity.
 * State x = [pE, pN, vE, vN]. Pure float math, no HAL, no allocation. */
#ifndef KALMAN_H
#define KALMAN_H

/* innovation gate: chi^2(2 dof, 99%) */
#define KF_GATE_D2_99       9.21f
/* deadlock guard: force-accept after this many consecutive rejects */
#define KF_GATE_MAX_REJECTS 5
/* Doppler cross-check: reject if jump > this AND implied speed > 5x Doppler */
#define KF_GATE_JUMP_M      10.0f
#define KF_GATE_DOPPLER_K   5.0f

/* velocity decay during GPS outage: starts after this many seconds without
 * an accepted update, exponential with this time constant */
#define KF_OUTAGE_START_S   5.0f
#define KF_VEL_DECAY_TAU_S  10.0f

typedef struct {
    float x[4];      /* pE, pN, vE, vN */
    float P[4][4];   /* covariance */
    float sa2;       /* sigma_accel^2 - process noise (residual accel error) */
    float sg2;       /* sigma_gps^2   - measurement noise */

    /* gating state */
    float prev_zE, prev_zN;            /* попереднє вимірювання (для Doppler-чеку) */
    unsigned char have_prev_z;
    unsigned char consecutive_rejects;

    /* outage state: час від останнього ПРИЙНЯТОГО апдейту */
    float time_since_update;
} KalmanFilter;

void KF_Init(KalmanFilter *kf, float sigma_accel, float sigma_gps);
void KF_Predict(KalmanFilter *kf, float dt, float aE, float aN);
void KF_Update(KalmanFilter *kf, float zE, float zN);

/* Update з відбраковкою викидів: Махаланобіс d^2 проти KF_GATE_D2_99 +
 * Doppler крос-чек (стрибок позиції проти швидкості від приймача).
 * dt_fix - час від попереднього виміру (с), doppler_mps - швидкість з RMC.
 * Повертає 1, якщо апдейт прийнято, 0 - якщо відхилено (P не чіпається). */
int KF_UpdateGated(KalmanFilter *kf, float zE, float zN, float dt_fix, float doppler_mps);

void KF_GetPosition(const KalmanFilter *kf, float *pE, float *pN);
void KF_GetVelocity(const KalmanFilter *kf, float *vE, float *vN);
void KF_GetPDiag(const KalmanFilter *kf, float diag[4]);

#endif /* KALMAN_H */
