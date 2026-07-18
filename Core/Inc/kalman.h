/* kalman.h - 4-state linear Kalman filter for ENU position/velocity.
 * State x = [pE, pN, vE, vN]. Pure float math, no HAL, no allocation. */
#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float x[4];      /* pE, pN, vE, vN */
    float P[4][4];   /* covariance */
    float sa2;       /* sigma_accel^2 - process noise (residual accel error) */
    float sg2;       /* sigma_gps^2   - measurement noise */
} KalmanFilter;

void KF_Init(KalmanFilter *kf, float sigma_accel, float sigma_gps);
void KF_Predict(KalmanFilter *kf, float dt, float aE, float aN);
void KF_Update(KalmanFilter *kf, float zE, float zN);

void KF_GetPosition(const KalmanFilter *kf, float *pE, float *pN);
void KF_GetVelocity(const KalmanFilter *kf, float *vE, float *vN);
void KF_GetPDiag(const KalmanFilter *kf, float diag[4]);

#endif /* KALMAN_H */
