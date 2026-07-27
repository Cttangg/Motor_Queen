#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <stdbool.h>

// 双二阶滤波器结构体
typedef struct {
    float a0, a1, a2, b0, b1, b2;
    float x1, x2, y1, y2;
} BiquadFilter;

// 2维卡尔曼滤波器结构体（代数展开版，免除矩阵库依赖）
typedef struct {
    float x[2];      // 状态向量: [0]角度, [1]角速度
    float P[2][2];   // 状态协方差矩阵
    float Q[2][2];   // 过程噪声协方差矩阵
    float R[2][2];   // 测量噪声协方差矩阵
    float dt;        // 采样时间间隔 (秒)
} KalmanFilter2D;

// 函数声明
void Biquad_Init(BiquadFilter *f, float b0, float b1, float b2, float a0, float a1, float a2);
float Biquad_Process(BiquadFilter *f, float input);
void Kalman2D_Init(KalmanFilter2D *kf, float dt);
void Kalman2D_Predict(KalmanFilter2D *kf);
void Kalman2D_Update(KalmanFilter2D *kf, float meas_angle, float meas_velocity);

#endif /* FILTER_H */