#include "line_pid.h"
#include "grayscale.h"

/* ============================================================
 * 灰度循线 PID — 实现
 * ------------------------------------------------------------
 * 传感器: bit0=最左, bit7=最右 (1=检测到黑线)
 * 重心法: center = Σ(bit_i × i) / Σ(bit_i)
 * 偏差:   error  = center − 3.5  (负=线偏左, 正=线偏右)
 * PID输出: corr  = Kp·error + Ki·∫error + Kd·d(error)/dt
 *          误差为负 → PID输出为负 → 左轮加速右轮减速 → 车左转追线
 * 限幅后返回, 供 trajectory_set_feedback 使用。
 * ============================================================ */

#define MAX_CORR  0.15f      /* 转向修正上限 m/s */
#define MAX_INTEG 0.30f      /* 积分限幅 */

static float g_last_abs_error = 0.0f;

static struct {
    float Kp, Ki, Kd, dt;
    float integral, prev_error;
    uint8_t init;
} g_pid;

void LinePID_Init(float kp, float ki, float kd)
{
    g_pid.Kp = kp; g_pid.Ki = ki; g_pid.Kd = kd;
    g_pid.dt = 0.01f;               /* 10ms 固定 */
    g_pid.integral   = 0.0f;
    g_pid.prev_error = 0.0f;
    g_pid.init = 1;
}

void LinePID_SetKp(float kp) { g_pid.Kp = kp; g_pid.integral = 0.0f; g_pid.prev_error = 0.0f; }
void LinePID_SetKi(float ki) { g_pid.Ki = ki; g_pid.integral = 0.0f; g_pid.prev_error = 0.0f; }
void LinePID_SetKd(float kd) { g_pid.Kd = kd; g_pid.integral = 0.0f; g_pid.prev_error = 0.0f; }

void LinePID_Reset(void)
{
    g_pid.integral   = 0.0f;
    g_pid.prev_error = 0.0f;
}

/* 重心法: bit0=最左(权重0) bit7=最右(权重7) center=3.5
 * 返回 sensor 对应的线位置偏差 (负=偏左, 正=偏右) */
static float calc_error(uint8_t sensor)
{
    int sum_n = 0, sum_w = 0;
    for (int i = 0; i < 8; i++) {
        if (sensor & (1 << i)) {
            sum_n++;
            sum_w += i;                     /* 权重 = 位号 0~7 */
        }
    }
    if (sum_n == 0) return 0.0f;            /* 无线: 不偏 */
    float center = (float)sum_w / (float)sum_n;
    return center - 3.5f;                   /* 负=偏左 正=偏右 */
}

float LinePID_Update(uint8_t sensor)
{
    if (!g_pid.init) return 0.0f;

    float error = calc_error(sensor);
    g_last_abs_error = (error < 0.0f) ? -error : error;   /* 存 |error| 供速度调制 */

    g_pid.integral += error * g_pid.dt;
    if (g_pid.integral >  MAX_INTEG) g_pid.integral =  MAX_INTEG;
    if (g_pid.integral < -MAX_INTEG) g_pid.integral = -MAX_INTEG;

    float deriv = (error - g_pid.prev_error) / g_pid.dt;
    g_pid.prev_error = error;

    /* corr 符号与 error 同号:
     * error<0(线偏左) → corr<0 → v_L↑ v_R↓ → 左转追线 ✓ */
    float corr = g_pid.Kp * error
               + g_pid.Ki * g_pid.integral
               + g_pid.Kd * deriv;

    if (corr >  MAX_CORR) corr =  MAX_CORR;
    if (corr < -MAX_CORR) corr = -MAX_CORR;

    return corr;
}

float LinePID_GetAbsError(void)
{
    return g_last_abs_error;
}

uint8_t LinePID_LineDetected(void)
{
    return Grayscale_Read() != 0;
}
