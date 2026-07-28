#include "gyro_pid.h"
#include "filter.h"

/* ============================================================
 * 陀螺仪直行 PID — 实现
 * ------------------------------------------------------------
 * 陀螺仪: I2C MPU6500 (I2C_GYRO)
 *   Gyro_ReadYawRate() → g_yaw_rate (°/s, 由 IMU_UpdateAttitude 更新)
 *
 * 数据流:
 *   yaw_rate → Biquad 低通 (fc≈10Hz) → heading 积分
 *     heading_locked=0: error = 0 − yaw_rate  (速率 PID, 原行为)
 *     heading_locked=1: error = target − heading (角度 PID, 航向锁定)
 * ============================================================ */

#define MAX_CORR  0.15f
#define MAX_INTEG 0.30f

extern volatile float g_yaw_rate;

static struct {
    float Kp, Ki, Kd, dt;
    float Kp_h, Ki_h;                  /* 航向模式增益 */
    float integral, prev_error;
    BiquadFilter lpf;                  /* 低通滤波器 */
    float heading;                     /* 累积航向角 (rad) */
    float target_heading;              /* 锁定目标角 (rad) */
    uint8_t init;
    uint8_t heading_locked;
    float last_yaw, last_corr;         /* 诊断直 */
    float drift_offset;                /* 漂移补偿 (°/s) */
} g_pid;

/* ==================== 陀螺仪底层 ==================== */

void Gyro_Init(void) { /* 占位 */ }

float Gyro_ReadYawRate(void) { return g_yaw_rate; }

/* ==================== PID ==================== */

void GyroPID_Init(float kp, float ki, float kd)
{
    g_pid.Kp = kp; g_pid.Ki = ki; g_pid.Kd = kd;
    g_pid.Kp_h = 2.0f; g_pid.Ki_h = 0.05f;
    g_pid.dt = 0.01f;
    g_pid.integral   = 0.0f;
    g_pid.prev_error = 0.0f;
    g_pid.heading    = 0.0f;
    g_pid.target_heading = 0.0f;
    g_pid.heading_locked = 0;
    g_pid.last_yaw  = 0.0f;
    g_pid.last_corr = 0.0f;

    /* 二阶 Butterworth 低通 fc≈10Hz, fs=采样率 (a0=1) */
    Biquad_Init(&g_pid.lpf,
        0.020083f, 0.040167f, 0.020083f,
        1.000000f, -1.561018f, 0.641352f);

    g_pid.init = 1;
}

void GyroPID_SetKp(float kp) { g_pid.Kp = kp; g_pid.integral = 0.0f; g_pid.prev_error = 0.0f; }
void GyroPID_SetKi(float ki) { g_pid.Ki = ki; g_pid.integral = 0.0f; g_pid.prev_error = 0.0f; }
void GyroPID_SetKd(float kd) { g_pid.Kd = kd; g_pid.integral = 0.0f; g_pid.prev_error = 0.0f; }

void GyroPID_Reset(void)
{
    g_pid.heading    = 0.0f;
    g_pid.integral   = 0.0f;
    g_pid.prev_error = 0.0f;
}

float GyroPID_Update(float yaw_rate)
{
    if (!g_pid.init) return 0.0f;

    /* 低通滤波, 减漂移补偿 */
    float fyaw = Biquad_Process(&g_pid.lpf, (yaw_rate - g_pid.drift_offset));
    g_pid.last_yaw = fyaw;

    /* 积分航向: 右转 ω_z 为负, 取反使 heading 正值=右偏 */
    g_pid.heading -= fyaw * (3.1415926f / 180.0f) * g_pid.dt;

    float error;
    float kp, ki, kd;
    if (g_pid.heading_locked) {
        error = g_pid.target_heading - g_pid.heading;
        kp = g_pid.Kp_h; ki = g_pid.Ki_h; kd = 0.0f;
    } else {
        error = fyaw;                       /* ω_z<0(右转)→error<0→corr<0→左转纠正 */
        kp = g_pid.Kp; ki = g_pid.Ki; kd = g_pid.Kd;
    }

    g_pid.integral += error * g_pid.dt;
    if (g_pid.integral >  MAX_INTEG) g_pid.integral =  MAX_INTEG;
    if (g_pid.integral < -MAX_INTEG) g_pid.integral = -MAX_INTEG;

    float deriv = (error - g_pid.prev_error) / g_pid.dt;
    g_pid.prev_error = error;

    float corr = kp * error + ki * g_pid.integral + kd * deriv;
    if (corr >  MAX_CORR) corr =  MAX_CORR;
    if (corr < -MAX_CORR) corr = -MAX_CORR;

    g_pid.last_corr = corr;
    return corr;
}

/* ==================== 诊断 ==================== */

float GyroPID_GetLastYawRate(void)    { return g_pid.last_yaw; }
float GyroPID_GetLastCorrection(void) { return g_pid.last_corr; }

/* ==================== 航向锁定 ==================== */

void GyroPID_EnableHeadingLock(uint8_t enable)
{
    if (enable) {
        g_pid.heading        = 0.0f;
        g_pid.target_heading = 0.0f;
        g_pid.heading_locked = 1;
    } else {
        g_pid.heading_locked = 0;
    }
    g_pid.integral   = 0.0f;
    g_pid.prev_error = 0.0f;
}

void GyroPID_SetHeadingKp(float kp) { g_pid.Kp_h = kp; g_pid.integral = 0.0f; g_pid.prev_error = 0.0f; }
void GyroPID_SetHeadingKi(float ki) { g_pid.Ki_h = ki; g_pid.integral = 0.0f; g_pid.prev_error = 0.0f; }

float GyroPID_GetHeading(void) { return g_pid.heading; }

void  GyroPID_SetDriftOffset(float offset) { g_pid.drift_offset = offset; }
float GyroPID_GetDriftOffset(void)          { return g_pid.drift_offset; }
