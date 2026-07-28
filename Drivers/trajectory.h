#ifndef __TRAJECTORY_H_
#define __TRAJECTORY_H_

#include <stdint.h>

/* ===================== 分段路径轨迹控制模块 =====================
 *
 * 循迹小车轨迹 = 若干"段"(直线/圆弧)首尾相接构成的闭合曲线 (如 8 字形)。
 *
 * 控制结构 (双环, 当前只实现外环开环):
 *   外环: 运动学前馈 (feedforward) —— 由每段几何解算出左右轮目标线速度
 *   内环: 轮速闭环 —— 由 motor_control 编码器 PID 维持转速
 *
 * 闭环预留 (后期循迹传感器接入):
 *   注册 traj_feedback_fn 回调返回"转向修正量"(m/s),
 *   trajectory_update() 每 tick 将其叠加到前馈上:
 *     v_L_cmd = ff_v_L - corr,  v_R_cmd = ff_v_R + corr
 *
 * 差速运动学:
 *   w   = direction * v / R
 *   v_L = v - w * L / 2,  v_R = v + w * L / 2
 *   n_motor = v_wheel * 60/(2*pi*r) * 减速比
 *   段时长: 圆弧 T = theta/|w|,  直线 T = distance/v
 *
 * direction: +1 = 逆时针 (左转) / -1 = 顺时针 (右转)
 */

/* ---- 段类型 ---- */
typedef enum {
    SEG_STRAIGHT = 0,   /* 直线段 */
    SEG_ARC,            /* 圆弧段 */
    SEG_ROTATE          /* 原地旋转 (陀螺仪闭环角度) */
} seg_type_t;

/* ---- 单段定义 ---- */
typedef struct {
    seg_type_t type;
    float      R;          /* 圆弧半径 (m); 直线段忽略 */
    float      length;     /* 圆弧/旋转: theta(rad) / 直线: distance(m) */
    float      v;          /* 段线速度 (m/s, > 0) */
    int        direction;  /* 圆弧/旋转: +1/-1; 直线段忽略 */
    uint8_t    use_line;   /* 1=启用灰度循线反馈 */
    uint8_t    gyro_stop;  /* 1=陀螺仪角度停止, 0=时间倒数 */
} traj_segment_t;

/* ---- 运行状态 ---- */
typedef enum {
    TRAJ_IDLE = 0,
    TRAJ_RUNNING,
    TRAJ_DONE
} traj_status_t;

/* ---- 闭环反馈回调 (预留): 返回转向修正量 (m/s) ---- */
typedef float (*traj_feedback_fn)(void);

/* ==================== 路径 API ==================== */

/* 加载并运行一条路径 (segs 需在运行期间保持有效)
 * loop = 1 : 闭合曲线, 走完最后一段回到第 0 段循环 (循迹用)
 * loop = 0 : 走完最后一段停车 (TRAJ_DONE)
 * 返回 0 成功, <0 参数非法
 */
int trajectory_run_path(const traj_segment_t *segs, uint8_t num, uint8_t loop);

/* ==================== 单段便捷 API (内部转为单段路径) ==================== */

int trajectory_arc(float R, float theta, float v_target, int direction);
int trajectory_arc_openloop(float R, float theta, float v_target, int direction);  /* 开环圆弧,无反馈 */
int trajectory_circle(float R, float v_target, int direction);   /* loop 圆周 */
int trajectory_circle_openloop(float R, float v_target, int direction); /* 开环圆周,无反馈 */
int trajectory_straight(float distance, float v_target);
int trajectory_straight_openloop(float distance, float v_target); /* 开环直行,无反馈 */
int trajectory_linefollow(float v_target);  /* 循迹模式: 指定速度, 闭环循线 */
int trajectory_rotate(float theta, float v_target, int direction); /* 原地旋转, 陀螺仪闭环 */
int trajectory_rotate_openloop(float theta, float v_target, int direction); /* 开环旋转,无反馈 */

/* 预置混合赛道 */
int trajectory_mix1(void);  /* 4段: 直1m→CW半圆→直1m→CW半圆 闭合 */
// int trajectory_mix2(void);  /* 预留 */
// int trajectory_mix3(void);  /* 预留 */

void trajectory_stop(void);

/* 在 10ms 定时器 ISR 中调用一次 (motor_control_update 之后) */
void trajectory_update(void);

/* ==================== 闭环控制预留接口 ==================== */

/* 注册循迹反馈回调 (返回转向修正 m/s); 传 NULL 关闭 */
void trajectory_set_feedback(traj_feedback_fn fn);
/* 使能/关闭闭环叠加 (0 = 纯开环前馈, 1 = 前馈 + 反馈修正) */
void trajectory_enable_closed_loop(uint8_t enable);

/* ==================== 状态查询 ==================== */

traj_status_t trajectory_get_status(void);
uint8_t       trajectory_get_segment_index(void);
uint32_t      trajectory_get_remaining_ms(void);   /* 当前段剩余时间 */

#endif
