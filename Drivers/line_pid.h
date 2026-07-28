#ifndef LINE_PID_H_
#define LINE_PID_H_

#include <stdint.h>

/* ============================================================
 * 模式 A — 灰度循线 PID
 * ------------------------------------------------------------
 * 输入: 灰度传感器 8bit (bit0=最左, bit7=最右, 1=黑线)
 * 重心法计算线位置偏差 → PID → 转向修正量 (m/s)
 * 供 trajectory_set_feedback() 使用
 * ============================================================ */

void    LinePID_Init(float kp, float ki, float kd);
void    LinePID_SetKp(float kp);
void    LinePID_SetKi(float ki);
void    LinePID_SetKd(float kd);
void    LinePID_Reset(void);                          /* 清积分, 切换模式时用 */
float   LinePID_Update(uint8_t sensor);               /* 返回 转向修正 m/s */
float   LinePID_GetAbsError(void);                    /* 返回最近一次 |error| (0~3.5) */
uint8_t LinePID_LineDetected(void);                   /* 是否有黑线 (任意 bit=1) */

#endif
