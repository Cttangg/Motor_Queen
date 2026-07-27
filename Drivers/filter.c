#include "filter.h"

void Biquad_Init(BiquadFilter *f, float b0, float b1, float b2, float a0, float a1, float a2) {
    f->b0 = b0; f->b1 = b1; f->b2 = b2;
    f->a0 = a0; f->a1 = a1; f->a2 = a2; // 确保 a0 得到的是 1.0f
    f->x1 = 0.0f; f->x2 = 0.0f;
    f->y1 = 0.0f; f->y2 = 0.0f;
}

float Biquad_Process(BiquadFilter *f, float input) {
    if (f->a0 == 0.0f) return input; // 防御性代码
    
    // 严格遵循标准差分方程解算
    float output = (f->b0 / f->a0) * input 
                 + (f->b1 / f->a0) * f->x1 
                 + (f->b2 / f->a0) * f->x2 
                 - (f->a1 / f->a0) * f->y1 
                 - (f->a2 / f->a0) * f->y2;
    
    f->x2 = f->x1;
    f->x1 = input;
    f->y2 = f->y1;
    f->y1 = output;
    
    return output;
}

void Kalman2D_Init(KalmanFilter2D *kf, float dt) {
    kf->dt = dt;
    kf->x[0] = 0.0f; kf->x[1] = 0.0f;
    
    // 初始协方差矩阵 P = Identity
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;
    
    // 过程噪声 Q = 0.0025
    kf->Q[0][0] = 0.0025f; kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;    kf->Q[1][1] = 0.0025f;
    
    // 测量噪声 R = 0.1
    kf->R[0][0] = 0.1f;    kf->R[0][1] = 0.0f;
    kf->R[1][0] = 0.0f;    kf->R[1][1] = 0.1f;
}

void Kalman2D_Predict(KalmanFilter2D *kf) {
    float dt = kf->dt;
    
    // 1. 状态预测: x = A * x
    kf->x[0] = kf->x[0] + kf->x[1] * dt;
    
    // 2. 协方差预测: P = A * P * A^T + Q
    float p00 = kf->P[0][0] + 2.0f * kf->P[0][1] * dt + kf->P[1][1] * dt * dt + kf->Q[0][0];
    float p01 = kf->P[0][1] + kf->P[1][1] * dt + kf->Q[0][1];
    float p11 = kf->P[1][1] + kf->Q[1][1];
    
    kf->P[0][0] = p00;
    kf->P[0][1] = p01;
    kf->P[1][0] = p01;
    kf->P[1][1] = p11;
}

void Kalman2D_Update(KalmanFilter2D *kf, float meas_angle, float meas_velocity) {
    // 1. 计算测量残差: y = z - H * x
    float y0 = meas_angle - kf->x[0];
    float y1 = meas_velocity - kf->x[1];
    
    // 2. 计算残差协方差: S = H * P * H^T + R
    float s00 = kf->P[0][0] + kf->R[0][0];
    float s01 = kf->P[0][1] + kf->R[0][1];
    float s10 = kf->P[1][0] + kf->R[1][0];
    float s11 = kf->P[1][1] + kf->R[1][1];
    
    // 3. 计算 S 的行列式并求逆
    float det = s00 * s11 - s01 * s10;
    if (det == 0.0f) return;
    float inv_det = 1.0f / det;
    
    float sinv00 =  s11 * inv_det;
    float sinv01 = -s01 * inv_det;
    float sinv10 = -s10 * inv_det;
    float sinv11 =  s00 * inv_det;
    
    // 4. 计算卡尔曼增益: K = P * H^T * S^-1
    float k00 = kf->P[0][0] * sinv00 + kf->P[0][1] * sinv10;
    float k01 = kf->P[0][0] * sinv01 + kf->P[0][1] * sinv11;
    float k10 = kf->P[1][0] * sinv00 + kf->P[1][1] * sinv10;
    float k11 = kf->P[1][0] * sinv01 + kf->P[1][1] * sinv11;
    
    // 5. 更新状态向量: x = x + K * y
    kf->x[0] += k00 * y0 + k01 * y1;
    kf->x[1] += k10 * y0 + k11 * y1;
    
    // 6. 更新状态协方差矩阵: P = (I - K * H) * P
    float kp00 = k00 * kf->P[0][0] + k01 * kf->P[1][0];
    float kp01 = k00 * kf->P[0][1] + k01 * kf->P[1][1];
    float kp10 = k10 * kf->P[0][0] + k11 * kf->P[1][0];
    float kp11 = k10 * kf->P[0][1] + k11 * kf->P[1][1];
    
    kf->P[0][0] -= kp00;
    kf->P[0][1] -= kp01;
    kf->P[1][0] -= kp10;
    kf->P[1][1] -= kp11;
}