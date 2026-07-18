/* kalman.c - 4-state linear KF, hand-unrolled matrix math (no loops, no HAL) */
#include "kalman.h"

void KF_Init(KalmanFilter *kf, float sigma_accel, float sigma_gps) {
    kf->sa2 = sigma_accel * sigma_accel;
    kf->sg2 = sigma_gps * sigma_gps;

    kf->x[0] = 0.0f; kf->x[1] = 0.0f; kf->x[2] = 0.0f; kf->x[3] = 0.0f;

    kf->prev_zE = 0.0f;
    kf->prev_zN = 0.0f;
    kf->have_prev_z = 0;
    kf->consecutive_rejects = 0;

    /* стартова невизначеність: позиція ~ GPS, швидкість невідома (5 м/с)^2 */
    kf->P[0][0] = kf->sg2; kf->P[0][1] = 0.0f;    kf->P[0][2] = 0.0f;  kf->P[0][3] = 0.0f;
    kf->P[1][0] = 0.0f;    kf->P[1][1] = kf->sg2; kf->P[1][2] = 0.0f;  kf->P[1][3] = 0.0f;
    kf->P[2][0] = 0.0f;    kf->P[2][1] = 0.0f;    kf->P[2][2] = 25.0f; kf->P[2][3] = 0.0f;
    kf->P[3][0] = 0.0f;    kf->P[3][1] = 0.0f;    kf->P[3][2] = 0.0f;  kf->P[3][3] = 25.0f;
}

void KF_Predict(KalmanFilter *kf, float dt, float aE, float aN) {
    const float dt2 = dt * dt;

    /* x = F*x + B*a */
    kf->x[0] += dt * kf->x[2] + 0.5f * dt2 * aE;
    kf->x[1] += dt * kf->x[3] + 0.5f * dt2 * aN;
    kf->x[2] += dt * aE;
    kf->x[3] += dt * aN;

    const float p00 = kf->P[0][0], p01 = kf->P[0][1], p02 = kf->P[0][2], p03 = kf->P[0][3];
    const float p10 = kf->P[1][0], p11 = kf->P[1][1], p12 = kf->P[1][2], p13 = kf->P[1][3];
    const float p20 = kf->P[2][0], p21 = kf->P[2][1], p22 = kf->P[2][2], p23 = kf->P[2][3];
    const float p30 = kf->P[3][0], p31 = kf->P[3][1], p32 = kf->P[3][2], p33 = kf->P[3][3];

    /* A = F*P: рядки 0,1 отримують +dt * рядки 2,3 */
    const float a00 = p00 + dt * p20, a01 = p01 + dt * p21, a02 = p02 + dt * p22, a03 = p03 + dt * p23;
    const float a10 = p10 + dt * p30, a11 = p11 + dt * p31, a12 = p12 + dt * p32, a13 = p13 + dt * p33;

    /* P = A*F^T: стовпці 0,1 отримують +dt * стовпці 2,3; потім +Q.
       Q - дискретизація НЕПЕРЕРВНОГО шуму прискорення (Q_vv ~ dt, не dt^2):
       інакше шум процесу залежить від частоти предикту, і при 100 Гц фільтр
       стає надвпевненим у 100 разів - bias акселерометра тягне позицію на метри */
    const float q_pp = kf->sa2 * dt2 * dt / 3.0f;
    const float q_pv = 0.5f * kf->sa2 * dt2;
    const float q_vv = kf->sa2 * dt;

    kf->P[0][0] = a00 + dt * a02 + q_pp;
    kf->P[0][1] = a01 + dt * a03;
    kf->P[0][2] = a02 + q_pv;
    kf->P[0][3] = a03;
    kf->P[1][0] = a10 + dt * a12;
    kf->P[1][1] = a11 + dt * a13 + q_pp;
    kf->P[1][2] = a12;
    kf->P[1][3] = a13 + q_pv;
    kf->P[2][0] = p20 + dt * p22 + q_pv;
    kf->P[2][1] = p21 + dt * p23;
    kf->P[2][2] = p22 + q_vv;
    kf->P[2][3] = p23;
    kf->P[3][0] = p30 + dt * p32;
    kf->P[3][1] = p31 + dt * p33 + q_pv;
    kf->P[3][2] = p32;
    kf->P[3][3] = p33 + q_vv;
}

/* спільне ядро апдейту: i00/i01/i11 - елементи S^-1 (S симетрична) */
static void kf_apply_update(KalmanFilter *kf, float y0, float y1,
                            float i00, float i01, float i11) {
    const float i10 = i01;

    /* K = P*H^T*S^-1: P*H^T - це перші два стовпці P */
    const float k00 = kf->P[0][0] * i00 + kf->P[0][1] * i10;
    const float k01 = kf->P[0][0] * i01 + kf->P[0][1] * i11;
    const float k10 = kf->P[1][0] * i00 + kf->P[1][1] * i10;
    const float k11 = kf->P[1][0] * i01 + kf->P[1][1] * i11;
    const float k20 = kf->P[2][0] * i00 + kf->P[2][1] * i10;
    const float k21 = kf->P[2][0] * i01 + kf->P[2][1] * i11;
    const float k30 = kf->P[3][0] * i00 + kf->P[3][1] * i10;
    const float k31 = kf->P[3][0] * i01 + kf->P[3][1] * i11;

    kf->x[0] += k00 * y0 + k01 * y1;
    kf->x[1] += k10 * y0 + k11 * y1;
    kf->x[2] += k20 * y0 + k21 * y1;
    kf->x[3] += k30 * y0 + k31 * y1;

    /* P = (I - K*H)*P: від рядка i віднімається Ki0*рядок0 + Ki1*рядок1 */
    const float p00 = kf->P[0][0], p01 = kf->P[0][1], p02 = kf->P[0][2], p03 = kf->P[0][3];
    const float p10 = kf->P[1][0], p11 = kf->P[1][1], p12 = kf->P[1][2], p13 = kf->P[1][3];

    kf->P[0][0] -= k00 * p00 + k01 * p10;
    kf->P[0][1] -= k00 * p01 + k01 * p11;
    kf->P[0][2] -= k00 * p02 + k01 * p12;
    kf->P[0][3] -= k00 * p03 + k01 * p13;
    kf->P[1][0] -= k10 * p00 + k11 * p10;
    kf->P[1][1] -= k10 * p01 + k11 * p11;
    kf->P[1][2] -= k10 * p02 + k11 * p12;
    kf->P[1][3] -= k10 * p03 + k11 * p13;
    kf->P[2][0] -= k20 * p00 + k21 * p10;
    kf->P[2][1] -= k20 * p01 + k21 * p11;
    kf->P[2][2] -= k20 * p02 + k21 * p12;
    kf->P[2][3] -= k20 * p03 + k21 * p13;
    kf->P[3][0] -= k30 * p00 + k31 * p10;
    kf->P[3][1] -= k30 * p01 + k31 * p11;
    kf->P[3][2] -= k30 * p02 + k31 * p12;
    kf->P[3][3] -= k30 * p03 + k31 * p13;
}

/* S = H*P*H^T + R (2x2), інверсія в лоб; повертає 0 при виродженні */
static int kf_innovation(const KalmanFilter *kf, float zE, float zN,
                         float *y0, float *y1, float *i00, float *i01, float *i11) {
    *y0 = zE - kf->x[0];
    *y1 = zN - kf->x[1];
    const float s00 = kf->P[0][0] + kf->sg2;
    const float s01 = kf->P[0][1];
    const float s11 = kf->P[1][1] + kf->sg2;
    const float det = s00 * s11 - s01 * s01;
    if (det <= 0.0f) {
        return 0;
    }
    const float inv_det = 1.0f / det;
    *i00 =  s11 * inv_det;
    *i01 = -s01 * inv_det;
    *i11 =  s00 * inv_det;
    return 1;
}

void KF_Update(KalmanFilter *kf, float zE, float zN) {
    float y0, y1, i00, i01, i11;
    if (!kf_innovation(kf, zE, zN, &y0, &y1, &i00, &i01, &i11)) {
        return; /* виродження - пропускаємо апдейт, фільтр лишається живим */
    }
    kf_apply_update(kf, y0, y1, i00, i01, i11);
}

int KF_UpdateGated(KalmanFilter *kf, float zE, float zN, float dt_fix, float doppler_mps) {
    float y0, y1, i00, i01, i11;
    if (!kf_innovation(kf, zE, zN, &y0, &y1, &i00, &i01, &i11)) {
        return 0;
    }

    int doppler_bad = 0;

    /* Doppler крос-чек: позиція стрибнула, а приймач каже "стою" - викид.
       Порівняння у квадратах, щоб обійтись без sqrt */
    if (kf->have_prev_z && dt_fix > 1e-3f) {
        const float je = zE - kf->prev_zE;
        const float jn = zN - kf->prev_zN;
        const float jump2 = je * je + jn * jn;
        const float dop_lim = KF_GATE_DOPPLER_K * doppler_mps * dt_fix;
        if (jump2 > KF_GATE_JUMP_M * KF_GATE_JUMP_M && jump2 > dop_lim * dop_lim) {
            doppler_bad = 1;
        }
    }
    int reject = doppler_bad;

    /* Махаланобіс: d^2 = y^T * S^-1 * y */
    if (!reject) {
        const float d2 = i00 * y0 * y0 + 2.0f * i01 * y0 * y1 + i11 * y1 * y1;
        if (d2 > KF_GATE_D2_99) {
            reject = 1;
        }
    }

    /* Запобіжник від deadlock: після N підряд відмов - жорсткий re-lock на
       вимірювання. "Просто прийняти" не годиться: розділий P дає величезний
       гейн і по швидкості (v стрибає на десятки м/с), позиція перелітає ціль
       і фільтр осцилює. Reset позиції + нульова швидкість збігаються за 2-3 fix-и.
       Re-lock дозволений ТІЛЬКИ на Doppler-консистентному вимірюванні: реальна
       зміна позиції після першого стрибка стає консистентною (нові fix-и лягають
       поруч), а multipath-марш - ні, і за нього чіплятись не можна */
    if (reject) {
        if (kf->consecutive_rejects < 255) {
            kf->consecutive_rejects++;
        }
        if (kf->consecutive_rejects >= KF_GATE_MAX_REJECTS && !doppler_bad) {
            kf->x[0] = zE; kf->x[1] = zN;
            kf->x[2] = 0.0f; kf->x[3] = 0.0f;
            kf->P[0][0] = kf->sg2; kf->P[0][1] = 0.0f;    kf->P[0][2] = 0.0f;  kf->P[0][3] = 0.0f;
            kf->P[1][0] = 0.0f;    kf->P[1][1] = kf->sg2; kf->P[1][2] = 0.0f;  kf->P[1][3] = 0.0f;
            kf->P[2][0] = 0.0f;    kf->P[2][1] = 0.0f;    kf->P[2][2] = 25.0f; kf->P[2][3] = 0.0f;
            kf->P[3][0] = 0.0f;    kf->P[3][1] = 0.0f;    kf->P[3][2] = 0.0f;  kf->P[3][3] = 25.0f;
            kf->consecutive_rejects = 0;
            reject = 0;
        }
    } else {
        kf_apply_update(kf, y0, y1, i00, i01, i11);
        kf->consecutive_rejects = 0;
    }

    kf->prev_zE = zE;
    kf->prev_zN = zN;
    kf->have_prev_z = 1;
    return !reject;
}

void KF_GetPosition(const KalmanFilter *kf, float *pE, float *pN) {
    *pE = kf->x[0];
    *pN = kf->x[1];
}

void KF_GetVelocity(const KalmanFilter *kf, float *vE, float *vN) {
    *vE = kf->x[2];
    *vN = kf->x[3];
}

void KF_GetPDiag(const KalmanFilter *kf, float diag[4]) {
    diag[0] = kf->P[0][0];
    diag[1] = kf->P[1][1];
    diag[2] = kf->P[2][2];
    diag[3] = kf->P[3][3];
}
