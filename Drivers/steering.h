#ifndef STEERING_H_
#define STEERING_H_

#include <stdint.h>

/* ============================================================
 * 闭环转向调度器
 * ------------------------------------------------------------
 * 双模式自动切换:
 *   模式 A (循线): 灰度传感器检测到黑线 → LinePID 控制
 *   模式 B (直行): 线丢失超过阈值 → GyroPID 控制 (保持直线航向)
 *
 * Steering_GetCorrection() 直接用作 trajectory_set_feedback() 的回调参数。
 * ============================================================ */

#define STEERING_LOST_THRESHOLD 10    /* 线丢失 10 次(100ms)后切陀螺仪 */
#define STEERING_FOUND_THRESHOLD 10   /* 线重新出现 10 次(100ms)后切回循线 */

void  Steering_Init(void);
float Steering_GetCorrection(void);    /* traj_feedback_fn 回调 */
void  Steering_SetMode(uint8_t m);     /* 手动切模式: 0=循线 1=陀螺仪 */
uint8_t Steering_GetMode(void);        /* 返回当前模式 */

#endif
