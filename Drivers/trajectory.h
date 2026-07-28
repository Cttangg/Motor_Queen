#ifndef __TRAJECTORY_H_
#define __TRAJECTORY_H_

#include <stdint.h>

/* ---- 段类型 ---- */
typedef enum {
    SEG_STRAIGHT = 0,
    SEG_ARC,
    SEG_ROTATE
} seg_type_t;

/* ---- 单段定义 ---- */
typedef struct {
    seg_type_t type;
    float      R;
    float      length;     /* 圆弧/旋转: theta(rad) / 直线: distance(m) */
    float      v;
    int        direction;  /* 圆弧/旋转: +1/-1; 直线段忽略 */
    uint8_t    use_line;   /* 1=启用灰度循线反馈 */
    uint8_t    gyro_stop;  /* 1=陀螺仪角度停止, 0=时间倒数 */
} traj_segment_t;

/* ---- 运行状态 ---- */
typedef enum {
    TRAJ_IDLE = 0,
    TRAJ_RUNNING,
    TRAJ_PAUSED,
    TRAJ_DONE
} traj_status_t;

/* ---- 闭环反馈回调: 返回转向修正量 (m/s) ---- */
typedef float (*traj_feedback_fn)(void);

/* ---- 转向恢复请求 (steering → trajectory) ---- */
typedef enum {
    STEER_REQ_NONE = 0,
    STEER_REQ_RECOVER_LEFT,
    STEER_REQ_RECOVER_RIGHT,
    STEER_REQ_LINE_FOUND
} steering_request_t;

/* ---- 独立恢复旋转状态 (不覆盖 g_single 或 g_traj.segs) ---- */
typedef struct {
    uint8_t active;
    float   target_angle;   /* rad */
    int     direction;
    float   speed;
    float   accum;
    uint32_t timeout_ticks;  /* 超时保护 */
} recovery_rotate_t;

/* ==================== 路径 API ==================== */

int trajectory_run_path(const traj_segment_t *segs, uint8_t num, uint8_t loop);

/* ==================== 单段便捷 API ==================== */

int trajectory_arc(float R, float theta, float v_target, int direction);
int trajectory_arc_openloop(float R, float theta, float v_target, int direction);
int trajectory_circle(float R, float v_target, int direction);
int trajectory_circle_openloop(float R, float v_target, int direction);
int trajectory_straight(float distance, float v_target);
int trajectory_straight_openloop(float distance, float v_target);
int trajectory_linefollow(float v_target);
int trajectory_rotate(float theta, float v_target, int direction);
int trajectory_rotate_openloop(float theta, float v_target, int direction);

int trajectory_mix1(void);

void trajectory_stop(void);

/* ==================== 暂停/恢复 (失线恢复用) ==================== */

void trajectory_pause(void);
void trajectory_resume(void);

/* ==================== 恢复旋转 (独立于 g_single, 不覆盖原轨迹) ==================== */

void trajectory_start_recovery_rotate(float theta, float speed, int dir);
void trajectory_stop_recovery_rotate(void);

/* ==================== ISR (10ms) ==================== */
void trajectory_update(void);
void trajectory_recovery_update(void);  /* 失线恢复状态机 */

/* ==================== 闭环控制预留接口 ==================== */

void trajectory_set_feedback(traj_feedback_fn fn);
void trajectory_enable_closed_loop(uint8_t enable);

/* 速度因子 (0.0~1.0): STABLE 阶段降速 */
void trajectory_set_speed_factor(float f);

/* ==================== 状态查询 ==================== */

traj_status_t trajectory_get_status(void);
uint8_t       trajectory_get_segment_index(void);
uint32_t      trajectory_get_remaining_ms(void);

#endif
