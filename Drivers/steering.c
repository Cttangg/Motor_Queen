#include "steering.h"
#include "line_pid.h"
#include "gyro_pid.h"
#include "grayscale.h"

/* ============================================================
 * 闭环转向调度器 — 实现
 * ------------------------------------------------------------
 * 每 10ms 由 trajectory_update → apply_speed 调用一次本模块的
 * Steering_GetCorrection(), 返回 corr (m/s) 叠加到前馈轮速。
 *
 * 模式切换:
 *   - 灰度检测到线 → 模式 A (LinePID)
 *   - 线连续丢失 > LOST_THRESHOLD → 模式 B (GyroPID)
 *   - 线重新出现 → 立即切回模式 A
 *
 * 限幅: 转向修正上限 LINE_PID/GYRO_PID 内部处理 (0.15 m/s)
 * ============================================================ */

static uint8_t  g_mode          = 0;   /* 0=循线 1=陀螺仪 */
static uint32_t g_lost_count    = 0;   /* 连续无线计数 */
static uint32_t g_found_count   = 0;   /* 连续有线计数 */

void Steering_Init(void)
{
    LinePID_Init(0.2f, 0.05f, 0.0f);
    GyroPID_Init(0.5f, 0.02f, 0.0f);
    GyroPID_EnableHeadingLock(1);                      /* 启用航向锁定 */
    Gyro_Init();
    g_mode        = 0;
    g_lost_count  = 0;
    g_found_count = 0;
}

float Steering_GetCorrection(void)
{
    uint8_t sensor = Grayscale_Read();
    uint8_t line   = LinePID_LineDetected();

    if (line) {
        g_found_count++;
        g_lost_count = 0;
        if (g_mode == 1 && g_found_count >= STEERING_FOUND_THRESHOLD) {
            /* 100ms 持续有线 → 切回循线 */
            LinePID_Reset();
            GyroPID_Reset();
            g_mode = 0;
        }
        if (g_mode == 0)
            return LinePID_Update(sensor);
        else
            return GyroPID_Update(Gyro_ReadYawRate());
    }

    /* 无黑线 */
    g_found_count = 0;
    g_lost_count++;
    if (g_lost_count >= STEERING_LOST_THRESHOLD) {
        if (g_mode == 0) {
            GyroPID_Reset();
        }
        g_mode = 1;
        return GyroPID_Update(Gyro_ReadYawRate());
    }

    if (g_mode == 0)
        return LinePID_Update(sensor);
    else
        return GyroPID_Update(Gyro_ReadYawRate());
}

void Steering_SetMode(uint8_t m)
{
    if (m == 0) {
        LinePID_Reset();
        g_mode = 0;
    } else {
        GyroPID_Reset();
        g_mode = 1;
    }
    g_lost_count  = 0;
    g_found_count = 0;
}

uint8_t Steering_GetMode(void)
{
    return g_mode;
}
