#ifndef STEERING_H_
#define STEERING_H_

#include <stdint.h>
#include "trajectory.h"

/* ============================================================
 * 闭环转向调度器
 * ------------------------------------------------------------
 * steering 只负责：
 *   1. 读取灰度传感器
 *   2. 判断失线/找回
 *   3. 通过 Steering_GetRequest() 上报恢复请求
 *
 * trajectory 是唯一运动控制状态机，
 * 读取 steering 请求后执行 pause / recovery_rotate / resume。
 *
 * 双模式:
 *   模式 0 (循线): LinePID 控制
 *   模式 1 (直行): GyroPID 控制 (保持航向)
 * ============================================================ */

#define STEERING_LOST_THRESHOLD  5    /* 50ms 全零 → 触发急停 */
#define STEERING_LOST_MAX        600  /* 6秒连0 → 强制重置 */
#define STEERING_RECOVER_TICKS   5    /* 连续 5 帧有线 → 确认找回 */
#define STEERING_ROTATE_SPEED    0.15f
#define STEERING_BRAKE_TICKS     10   /* 刹车保持 100ms */
#define STEERING_STABLE_TICKS    20   /* 找回后低速稳定 200ms */
#define STEERING_STABLE_SPEED    0.5f /* 稳定阶段速度因子 */

void  Steering_Init(void);
float Steering_GetCorrection(void);    /* traj_feedback_fn 回调 */
void  Steering_Poll(void);             /* 失线/找回检测, ISR 中调用 */
void  Steering_SetMode(uint8_t m);
uint8_t Steering_GetMode(void);

/* 恢复请求 (trajectory 每 tick 读取) */
steering_request_t Steering_GetRequest(void);

#endif
