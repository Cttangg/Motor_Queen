#include "trajectory.h"
#include "motor_control.h"
#include "line_pid.h"
#include "gyro_pid.h"
#include "grayscale.h"

/* ==================== 机器人硬件常量 (需按实车标定) ==================== */

#define WHEEL_BASE     0.14f     /* 左右轮距 L (m) */
#define WHEEL_RADIUS   0.024f    /* 轮子半径 (m), 直径 0.048m */
#define GEAR_RATIO     20.0f     /* 减速比 1:20, 电机轴 = 输出轴 × 20 */
#define CTRL_DT        0.01f     /* 控制周期 (s), 与 TIMG12 10ms ISR 一致 */

#define PI_F           3.1415926f

/* 电机 ID 与左右轮映射 (按接线调整)
 * 后驱车: 电机1=右后轮, 电机2=左后轮 */
#define MOTOR_L_ID     2
#define MOTOR_R_ID     1

/* 轮转向符号: 正 RPM = 后退, 改为 -1 使正线速度映射为负 RPM = 前进 */
#define MOTOR_L_SIGN   (-1)
#define MOTOR_R_SIGN   (-1)

/* v(m/s) -> 电机轴 RPM 的转换系数 */
#define MPS_TO_MOTOR_RPM  (60.0f / (2.0f * PI_F * WHEEL_RADIUS) * GEAR_RATIO)

/* ==================== 轨迹状态结构体 ==================== */

static struct {
    traj_status_t status;

    /* --- 路径 (分段) --- */
    const traj_segment_t *segs;     /* 段数组 (调用者持有) */
    uint8_t  num_segs;
    uint8_t  seg_index;             /* 当前段 */
    uint8_t  loop;                  /* 1 = 闭合曲线循环 */

    /* --- 当前段前馈 (开环) 轮速 m/s --- */
    float    ff_v_L;
    float    ff_v_R;
    uint32_t remaining_ticks;       /* 当前段剩余 10ms tick 数 */

    /* --- 原地旋转 (陀螺仪闭环角度) --- */
    float    rotate_accum;          /* 已累积的旋转角度 (rad) */
    float    rotate_target;         /* 目标旋转角度 (rad) */
    int      rotate_dir;            /* 旋转方向 (±1) */

    /* --- 混合路径: 线检测消抖 + rot 补角 --- */
    uint8_t  line_lost_count;       /* 连续无线计数 (消抖用) */
    uint8_t  line_lost_triggered;   /* 已触发 rot 补角标志 */

    /* --- 闭环预留 --- */
    uint8_t          closed_loop;   /* 0 = 纯开环, 1 = 叠加反馈 */
    traj_feedback_fn feedback;      /* 循迹反馈回调, 返回修正 (m/s) */
} g_traj;

/* 单段便捷 API 的内部段存储 */
static traj_segment_t g_single;

static inline float fabs_f(float x) { return (x < 0.0f) ? -x : x; }

/* ==================== 内部: 段解算与下发 ==================== */

/* 解算第 i 段的前馈轮速与持续时间, 装载为当前段 */
static void load_segment(uint8_t i)
{
    const traj_segment_t *s = &g_traj.segs[i];
    float v = s->v;
    float duration;

    if (s->type == SEG_ARC) {
        float w = (float)s->direction * (v / s->R);
        g_traj.ff_v_L = v - (w * WHEEL_BASE / 2.0f);
        g_traj.ff_v_R = v + (w * WHEEL_BASE / 2.0f);
        g_traj.rotate_accum  = 0.0f;
        g_traj.rotate_target = s->length;   /* length = theta */
        g_traj.rotate_dir    = s->direction;
        g_traj.closed_loop   = 0;           /* 旋转时不走反馈 */
        duration = 0.0f;                    /* 不用时间, 陀螺仪积分角度停止 */
    } else if (s->type == SEG_ROTATE) {
        /* 两轮反向: 左轮后退 右轮前进 = CCW(+1); 左轮前进 右轮后退 = CW(-1) */
        g_traj.ff_v_L = (float)(-s->direction) * v;
        g_traj.ff_v_R = (float)( s->direction) * v;
        g_traj.rotate_accum  = 0.0f;
        g_traj.rotate_target = s->length;   /* length = 目标弧度 theta */
        g_traj.rotate_dir    = s->direction;
        g_traj.closed_loop   = 0;           /* 旋转时不走反馈 */
        duration = 0.0f;                    /* 不用时间倒计时 */
    } else {                                                 /* SEG_STRAIGHT */
        g_traj.ff_v_L = v;
        g_traj.ff_v_R = v;
        duration = (v > 1e-6f) ? (s->length / v) : 0.0f;     /* length = distance */
    }

    uint32_t ticks = (uint32_t)((duration / CTRL_DT) + 0.5f);
    if (ticks == 0) ticks = 1;
    g_traj.remaining_ticks = ticks;
    g_traj.seg_index = i;
}

/* 每 tick 下发轮速: 按 use_line 选择反馈源 */
static void apply_speed(void)
{
    float corr = 0.0f;

    if (g_traj.num_segs > 0 && g_traj.segs) {
        const traj_segment_t *seg = &g_traj.segs[g_traj.seg_index];
        if (seg->use_line && Grayscale_Read() != 0) {
            corr = LinePID_Update(Grayscale_Read());   /* 循线 */
        } else if (!seg->use_line && !seg->gyro_stop) {
            corr = 0.0f;                               /* 开环 */
        } else if (g_traj.closed_loop && g_traj.feedback) {
            corr = g_traj.feedback();                  /* 陀螺仪 */
        }
    } else if (g_traj.closed_loop && g_traj.feedback) {
        corr = g_traj.feedback();
    }

    float v_L = g_traj.ff_v_L + corr;
    float v_R = g_traj.ff_v_R - corr;

    motor_control_update_target(MOTOR_L_ID,
        (int32_t)(v_L * MPS_TO_MOTOR_RPM) * MOTOR_L_SIGN);
    motor_control_update_target(MOTOR_R_ID,
        (int32_t)(v_R * MPS_TO_MOTOR_RPM) * MOTOR_R_SIGN);
}

/* ==================== 路径 API ==================== */

int trajectory_run_path(const traj_segment_t *segs, uint8_t num, uint8_t loop)
{
    if (segs == 0 || num == 0) return -1;

    g_traj.segs     = segs;
    g_traj.num_segs = num;
    g_traj.loop     = loop ? 1 : 0;
    g_traj.line_lost_count    = 0;
    g_traj.line_lost_triggered = 0;
    load_segment(0);
    g_traj.status = TRAJ_RUNNING;
    return 0;
}

/* ==================== 单段便捷 API ==================== */

int trajectory_arc(float R, float theta, float v_target, int direction)
{
    if (R <= 0.0f || theta <= 0.0f || v_target <= 0.0f ||
        (direction != 1 && direction != -1))
        return -1;

    g_single.type      = SEG_ARC;
    g_single.R         = R;
    g_single.length    = theta;
    g_single.v         = v_target;
    g_single.direction = direction;
    return trajectory_run_path(&g_single, 1, 0);
}

int trajectory_arc_openloop(float R, float theta, float v_target, int direction)
{
    if (R <= 0.0f || theta <= 0.0f || v_target <= 0.0f ||
        (direction != 1 && direction != -1))
        return -1;

    g_single.type      = SEG_ARC;
    g_single.R         = R;
    g_single.length    = theta;
    g_single.v         = v_target;
    g_single.direction = direction;
    g_single.use_line  = 0;
    g_single.gyro_stop = 0;
    return trajectory_run_path(&g_single, 1, 0);
}

int trajectory_circle(float R, float v_target, int direction)
{
    if (R <= 0.0f || v_target <= 0.0f ||
        (direction != 1 && direction != -1))
        return -1;

    g_single.type      = SEG_ARC;
    g_single.R         = R;
    g_single.length    = 2.0f * PI_F;
    g_single.v         = v_target;
    g_single.direction = direction;
    return trajectory_run_path(&g_single, 1, 1);   /* loop = 持续圆周 */
}

int trajectory_circle_openloop(float R, float v_target, int direction)
{
    if (R <= 0.0f || v_target <= 0.0f ||
        (direction != 1 && direction != -1))
        return -1;

    g_single.type      = SEG_ARC;
    g_single.R         = R;
    g_single.length    = 2.0f * PI_F;
    g_single.v         = v_target;
    g_single.direction = direction;
    g_single.use_line  = 0;
    g_single.gyro_stop = 0;
    return trajectory_run_path(&g_single, 1, 1);   /* loop = 持续圆周, 开环 */
}

int trajectory_straight(float distance, float v_target)
{
    if (distance <= 0.0f || v_target <= 0.0f) return -1;

    g_single.type      = SEG_STRAIGHT;
    g_single.R         = 0.0f;
    g_single.length    = distance;
    g_single.v         = v_target;
    g_single.direction = 1;
    return trajectory_run_path(&g_single, 1, 0);
}

int trajectory_straight_openloop(float distance, float v_target)
{
    if (distance <= 0.0f || v_target <= 0.0f) return -1;

    g_single.type       = SEG_STRAIGHT;
    g_single.R          = 0.0f;
    g_single.length     = distance;
    g_single.v          = v_target;
    g_single.direction  = 1;
    g_single.use_line   = 0;
    g_single.gyro_stop  = 0;
    return trajectory_run_path(&g_single, 1, 0);
}

int trajectory_linefollow(float v_target)
{
    if (v_target <= 0.0f) return -1;

    float rpm = v_target * MPS_TO_MOTOR_RPM;

    /* num_segs=0 触发 trajectory_update 中的循迹分支 */
    g_traj.segs     = 0;
    g_traj.num_segs = 0;
    g_traj.loop     = 0;

    /* ff_v_L 存储基础 RPM, trajectory_update 循迹分支读出叠加修正 */
    g_traj.ff_v_L = rpm;
    g_traj.ff_v_R = rpm;

    g_traj.status          = TRAJ_RUNNING;
    g_traj.remaining_ticks = 0;
    return 0;
}

int trajectory_rotate(float theta, float v_target, int direction)
{
    if (theta <= 0.0f || v_target <= 0.0f ||
        (direction != 1 && direction != -1))
        return -1;

    g_single.type      = SEG_ROTATE;
    g_single.R         = 0.0f;
    g_single.length    = theta;
    g_single.v         = v_target;
    g_single.direction = direction;
    return trajectory_run_path(&g_single, 1, 0);
}

int trajectory_rotate_openloop(float theta, float v_target, int direction)
{
    if (theta <= 0.0f || v_target <= 0.0f ||
        (direction != 1 && direction != -1))
        return -1;

    g_single.type      = SEG_ROTATE;
    g_single.R         = 0.0f;
    g_single.length    = theta;
    g_single.v         = v_target;
    g_single.direction = direction;
    g_single.use_line  = 0;
    g_single.gyro_stop = 0;
    return trajectory_run_path(&g_single, 1, 0);
}

int trajectory_mix1(void)
{
    static const traj_segment_t segs[] = {
        /* type,   R,    length, v,    dir, use_line, gyro_stop */
        { SEG_STRAIGHT, 0,   1.0f,  0.2f, 0,  0, 0 },  /* 开环直行 */
        { SEG_ARC,      0.4f, 3.14f, 0.15f,-1, 1, 1 },  /* 黑线半圆 CW */
        { SEG_STRAIGHT, 0,   1.0f,  0.2f, 0,  0, 0 },  /* 开环直行 */
        { SEG_ARC,      0.4f, 3.14f, 0.15f,-1, 1, 1 },  /* 黑线半圆 CW */
    };
    return trajectory_run_path(segs,
        sizeof(segs) / sizeof(segs[0]), 0);
}

void trajectory_stop(void)
{
    g_traj.status          = TRAJ_IDLE;
    g_traj.remaining_ticks = 0;
    g_traj.ff_v_L          = 0.0f;
    g_traj.ff_v_R          = 0.0f;
    motor_control_set_speed(MOTOR_L_ID, 0);
    motor_control_set_speed(MOTOR_R_ID, 0);
}

/* ==================== ISR 状态机调度 (10ms) ==================== */

void trajectory_update(void)
{
    if (g_traj.status != TRAJ_RUNNING) return;

    /* 循迹模式 (trajectory_linefollow): 无段调度, EMA低通+降频到20Hz */
    if (g_traj.num_segs == 0 && g_traj.closed_loop && g_traj.feedback) {
        static float corr_filt = 0.0f;
        static int   lf_div    = 0;
        float corr = g_traj.feedback();
        corr_filt = 0.3f * corr + 0.7f * corr_filt;   /* EMA 低通平滑 */
        if (++lf_div >= 5) {
            lf_div = 0;
            /* 速度调制: 越偏线越慢, 中心最快 */
            float err = LinePID_GetAbsError();            /* 0(居中) ~ 3.5(边缘) */
            float spd = 1.0f - 0.15f * err;               /* 中心=1.0 边缘≈0.48 */
            if (spd < 0.4f) spd = 0.4f;                   /* 最低 40% */
            float base_rpm  = g_traj.ff_v_L * spd;
            float delta_rpm = corr_filt * MPS_TO_MOTOR_RPM;
            motor_control_update_target(MOTOR_L_ID,
                (int32_t)((base_rpm + delta_rpm) * MOTOR_L_SIGN));
            motor_control_update_target(MOTOR_R_ID,
                (int32_t)((base_rpm - delta_rpm) * MOTOR_R_SIGN));
        }
        return;
    }

    apply_speed();      /* 每 tick 下发 (便于闭环叠加修正) */

    /* 圆弧/旋转: 用陀螺仪 yaw_rate 积分角度, 到达目标后停车 */
    if (g_traj.num_segs > 0 && g_traj.segs &&
        (g_traj.segs[g_traj.seg_index].type == SEG_ARC ||
         g_traj.segs[g_traj.seg_index].type == SEG_ROTATE)) {

        const traj_segment_t *seg = &g_traj.segs[g_traj.seg_index];
        float yaw = Gyro_ReadYawRate();
        g_traj.rotate_accum += fabs_f(yaw) * (3.1415926f / 180.0f) * CTRL_DT;

        /* 黑线循迹弧段: 检测线丢失 → rot 补角 */
        if (seg->type == SEG_ARC && seg->use_line && !g_traj.line_lost_triggered) {
            if (Grayscale_Read() == 0) {
                g_traj.line_lost_count++;
                if (g_traj.line_lost_count >= 10) {  /* 100ms 消抖 */
                    float remaining = seg->length - g_traj.rotate_accum;
                    if (remaining > 0.05f) {
                        motor_control_set_speed(MOTOR_L_ID, 0);
                        motor_control_set_speed(MOTOR_R_ID, 0);
                        g_traj.line_lost_triggered = 1;
                        trajectory_rotate(remaining, 0.05f, seg->direction);
                        return;
                    }
                }
            } else {
                g_traj.line_lost_count = 0;
            }
        }

        /* 角度到达 → 停车/切段 */
        if (g_traj.rotate_accum >= g_traj.rotate_target) {
            uint8_t next = (uint8_t)(g_traj.seg_index + 1);
            if (next >= g_traj.num_segs) {
                motor_control_set_speed(MOTOR_L_ID, 0);
                motor_control_set_speed(MOTOR_R_ID, 0);
                g_traj.status = TRAJ_DONE;
            } else {
                load_segment(next);
            }
        }
        return;
    }

    if (g_traj.remaining_ticks > 0)
        g_traj.remaining_ticks--;

    if (g_traj.remaining_ticks == 0) {
        uint8_t next = (uint8_t)(g_traj.seg_index + 1);
        if (next >= g_traj.num_segs) {
            if (g_traj.loop) {
                next = 0;                       /* 闭合曲线: 回到起点 */
            } else {
                motor_control_set_speed(MOTOR_L_ID, 0);
                motor_control_set_speed(MOTOR_R_ID, 0);
                g_traj.status = TRAJ_DONE;
                return;
            }
        }
        load_segment(next);
    }
}

/* ==================== 闭环控制预留接口 ==================== */

void trajectory_set_feedback(traj_feedback_fn fn) { g_traj.feedback = fn; }

void trajectory_enable_closed_loop(uint8_t enable)
{
    g_traj.closed_loop = enable ? 1 : 0;
}

/* ==================== 状态查询 ==================== */

traj_status_t trajectory_get_status(void)        { return g_traj.status; }
uint8_t       trajectory_get_segment_index(void) { return g_traj.seg_index; }

uint32_t trajectory_get_remaining_ms(void)
{
    return g_traj.remaining_ticks * (uint32_t)(CTRL_DT * 1000.0f);
}
