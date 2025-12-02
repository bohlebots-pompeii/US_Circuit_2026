#include "Ekf.h"
#include <cmath>

// small anonymous-namespace helpers with internal linkage
namespace {
    inline void mat4_zero(float M[4][4]) {
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) M[i][j] = 0.0f;
    }
    inline void mat4_copy(float dst[4][4], const float src[4][4]) {
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) dst[i][j] = src[i][j];
    }
}

US_EKF::US_EKF() {
    x_[0] = x_[1] = x_[2] = x_[3] = 0.0f;
    resetCovariance();
}

void US_EKF::resetCovariance() {
    for (int i=0;i<4;i++) for (int j=0;j<4;j++) P_[i][j] = 0.0f;
    P_[0][0] = P_[1][1] = 1e3f; // px, py variance
    P_[2][2] = P_[3][3] = 1e3f; // vx, vy variance
}

void US_EKF::init(float px, float py) {
    x_[0] = px;
    x_[1] = py;
    x_[2] = 0.0f;
    x_[3] = 0.0f;
    resetCovariance();
}

void US_EKF::predict(float dt) {
    if (dt <= 0.0f) return;

    // State transition (constant velocity)
    float x_new[4];
    x_new[0] = x_[0] + x_[2]*dt;
    x_new[1] = x_[1] + x_[3]*dt;
    x_new[2] = x_[2];
    x_new[3] = x_[3];
    for (int i=0;i<4;i++) x_[i] = x_new[i];

    // Discrete process noise Qd from continuous accel variance
    const float s2 = sigma_acc * sigma_acc;
    const float dt2 = dt*dt;
    const float dt3 = dt2*dt;
    const float dt4 = dt2*dt2;

    float Qd[4][4];
    mat4_zero(Qd);

    Qd[0][0] = 0.25f * s2 * dt4;
    Qd[1][1] = Qd[0][0];
    Qd[0][2] = 0.5f * s2 * dt3;
    Qd[1][3] = Qd[0][2];
    Qd[2][0] = Qd[0][2];
    Qd[3][1] = Qd[0][2];
    Qd[2][2] = s2 * dt2;
    Qd[3][3] = Qd[2][2];

    // Compute new P = F * P * F^T + Qd (expanded algebra for CV model)
    float Pnew[4][4];
    // initialize zero
    mat4_zero(Pnew);

    // row 0
    Pnew[0][0] = P_[0][0] + dt*(P_[2][0] + P_[0][2]) + dt2*P_[2][2] + Qd[0][0];
    Pnew[0][1] = P_[0][1] + dt*(P_[2][1] + P_[0][3]) + dt2*P_[2][3] + Qd[0][1];
    Pnew[0][2] = P_[0][2] + dt*P_[2][2] + Qd[0][2];
    Pnew[0][3] = P_[0][3] + dt*P_[2][3] + Qd[0][3];

    // row 1
    Pnew[1][0] = P_[1][0] + dt*(P_[3][0] + P_[1][2]) + dt2*P_[3][2] + Qd[1][0];
    Pnew[1][1] = P_[1][1] + dt*(P_[3][1] + P_[1][3]) + dt2*P_[3][3] + Qd[1][1];
    Pnew[1][2] = P_[1][2] + dt*P_[3][2] + Qd[1][2];
    Pnew[1][3] = P_[1][3] + dt*P_[3][3] + Qd[1][3];

    // row 2
    Pnew[2][0] = P_[2][0] + dt*P_[2][2] + Qd[2][0];
    Pnew[2][1] = P_[2][1] + dt*P_[2][3] + Qd[2][1];
    Pnew[2][2] = P_[2][2] + Qd[2][2];
    Pnew[2][3] = P_[2][3] + Qd[2][3];

    // row 3
    Pnew[3][0] = P_[3][0] + dt*P_[3][2] + Qd[3][0];
    Pnew[3][1] = P_[3][1] + dt*P_[3][3] + Qd[3][1];
    Pnew[3][2] = P_[3][2] + Qd[3][2];
    Pnew[3][3] = P_[3][3] + Qd[3][3];

    // copy back
    mat4_copy(P_, Pnew);
}

void US_EKF::update(float px_meas, float py_meas, bool use_x, bool use_y) {
    float y0 = px_meas - x_[0];
    float y1 = py_meas - x_[1];

    float R00 = meas_std_default * meas_std_default;
    float R11 = R00;
    if (!use_x) R00 *= 1e6f;
    if (!use_y) R11 *= 1e6f;

    float S00 = P_[0][0] + R00;
    float S01 = P_[0][1];
    float S10 = P_[1][0];
    float S11 = P_[1][1] + R11;

    float det = S00*S11 - S01*S10;
    if (fabs(det) < 1e-9f) return; // skip update if singular
    float invS00 =  S11 / det;
    float invS11 =  S00 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;

    float K[4][2];
    for (int i=0;i<4;i++) {
        K[i][0] = P_[i][0]*invS00 + P_[i][1]*invS10;
        K[i][1] = P_[i][0]*invS01 + P_[i][1]*invS11;
    }

    if (!use_x) {
        for (int i=0;i<4;i++) K[i][0] = 0.0f;
        y0 = 0.0f;
    }
    if (!use_y) {
        for (int i=0;i<4;i++) K[i][1] = 0.0f;
        y1 = 0.0f;
    }

    // state update
    x_[0] += K[0][0]*y0 + K[0][1]*y1;
    x_[1] += K[1][0]*y0 + K[1][1]*y1;
    x_[2] += K[2][0]*y0 + K[2][1]*y1;
    x_[3] += K[3][0]*y0 + K[3][1]*y1;

    // P = (I - K H) P  (H = selects first two states)
    float KH[4][4];
    mat4_zero(KH);
    for (int i=0;i<4;i++) {
        KH[i][0] = K[i][0];
        KH[i][1] = K[i][1];
    }

    float IminusKH[4][4];
    for (int i=0;i<4;i++) for (int j=0;j<4;j++) IminusKH[i][j] = ((i==j)?1.0f:0.0f) - KH[i][j];

    float Pnew[4][4];
    mat4_zero(Pnew);
    for (int i=0;i<4;i++) {
        for (int j=0;j<4;j++) {
            float s = 0.0f;
            for (int k=0;k<4;k++) s += IminusKH[i][k] * P_[k][j];
            Pnew[i][j] = s;
        }
    }
    mat4_copy(P_, Pnew);
}

void US_EKF::step(float dt, float px_meas, float py_meas, bool use_x, bool use_y) {
    predict(dt);
    update(px_meas, py_meas, use_x, use_y);
}

std::array<float,2> US_EKF::getPosition() const {
    return { x_[0], x_[1] };
}

std::array<float,2> US_EKF::getVelocity() const {
    return { x_[2], x_[3] };
}
