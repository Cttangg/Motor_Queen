#include "trajectory.h"
#include "motor_control.h"
#include "gyro_pid.h"
#include "steering.h"

/* ==================== 机器人硬件常量 ==================== */

#define WHEEL_BASE     0.14f
#define WHEEL_RADIUS   0.024f
#define GEAR_RATIO     20.0f
#define CTRL_DT        0.01f

#define PI_F           3.1415926f

#define MOTOR_L_ID     2
#define MOTOR_R_ID     1
#define MOTOR_L_SIGN   (-1)
#define MOTOR_R_SIGN   (-1)

#define MPS_TO_MOTOR_RPM  (60.0f / (2.0f * PI_F * WHEEL_RADIUS) * GEAR_RATIO)

/* ==================== 轨迹状态结构体 ==================== */

static struct {
    volatile traj_status_t status;

    const traj_segment_t *segs;
    uint8_t  num_segs;
    uint8_t  seg_index;
    uint8_t  loop;

    float    ff_v_L;
    float    ff_v_R;
    uint32_t remaining_ticks;

    float    rotate_accum;
    float    rotate_target;
    int      rotate_dir;

    uint8_t  line_lost_count;
    uint8_t  line_lost_triggered;

    uint8_t          closed_loop;
    traj_feedback_fn feedback;
    float            speed_factor;   /* 1.0=全速, <1.0=降速 */

    /* ---- 暂停/恢复 ---- */
    uint8_t  paused_seg_index;
    uint32_t paused_remaining_ticks;
    float    paused_ff_v_L;
    float    paused_ff_v_R;
    float    paused_rotate_accum;
    float    paused_rotate_target;
    int      paused_rotate_dir;

    /* ---- 独立恢复旋转 ---- */
    recovery_rotate_t recovery;
} g_traj;

/* 单段便捷 API 的内部段存储 */
static traj_segment_t g_single;

static inline float fabs_f(float x) { return (x < 0.0f) ? -x : x; }

/* ==================== 内部: 段解算 ==================== */

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
        g_traj.rotate_target = s->length;
        g_traj.rotate_dir    = s->direction;
        g_traj.closed_loop   = 0;
        duration = 0.0f;
    } else if (s->type == SEG_ROTATE) {
        g_traj.ff_v_L = (float)(-s->direction) * v;
        g_traj.ff_v_R = (float)( s->direction) * v;
        g_traj.rotate_accum  = 0.0f;
        g_traj.rotate_target = s->length;
        g_traj.rotate_dir    = s->direction;
        g_traj.closed_loop   = 0;
        duration = 0.0f;
    } else {
        g_traj.ff_v_L = v;
        g_traj.ff_v_R = v;
        duration = (v > 1e-6f) ? (s->length / v) : 0.0f;
    }

    uint32_t ticks = (uint32_t)((duration / CTRL_DT) + 0.5f);
    if (ticks == 0) ticks = 1;
    g_traj.remaining_ticks = ticks;
    g_traj.seg_index = i;
}

/* 每 tick 下发轮速: 只通过 feedback() 获取修正, 禁止直接调 LinePID */
static void apply_speed(void)
{
    float corr = 0.0f;

    if (g_traj.closed_loop && g_traj.feedback) {
        corr = g_traj.feedback();
    }

    float v_L = (g_traj.ff_v_L + corr) * g_traj.speed_factor;
    float v_R = (g_traj.ff_v_R - corr) * g_traj.speed_factor;

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
    g_traj.recovery.active = 0;
    g_traj.speed_factor    = 1.0f;
    load_segment(0);
    g_traj.status = TRAJ_RUNNING;
    return 0;
}

/* ==================== 暂停/恢复 ==================== */

void trajectory_pause(void)
{
    g_traj.paused_seg_index      = g_traj.seg_index;
    g_traj.paused_remaining_ticks = g_traj.remaining_ticks;
    g_traj.paused_ff_v_L          = g_traj.ff_v_L;
    g_traj.paused_ff_v_R          = g_traj.ff_v_R;
    g_traj.paused_rotate_accum    = g_traj.rotate_accum;
    g_traj.paused_rotate_target   = g_traj.rotate_target;
    g_traj.paused_rotate_dir      = g_traj.rotate_dir;

    g_traj.ff_v_L = 0.0f;
    g_traj.ff_v_R = 0.0f;
    motor_control_update_target(MOTOR_L_ID, 0);
    motor_control_update_target(MOTOR_R_ID, 0);
    g_traj.status = TRAJ_PAUSED;
}

void trajectory_resume(void)
{
    g_traj.seg_index        = g_traj.paused_seg_index;
    g_traj.remaining_ticks = g_traj.paused_remaining_ticks;
    g_traj.ff_v_L          = g_traj.paused_ff_v_L;
    g_traj.ff_v_R          = g_traj.paused_ff_v_R;
    g_traj.rotate_accum    = g_traj.paused_rotate_accum;
    g_traj.rotate_target   = g_traj.paused_rotate_target;
    g_traj.rotate_dir      = g_traj.paused_rotate_dir;
    g_traj.recovery.active = 0;
    g_traj.speed_factor    = 1.0f;
    g_traj.status = TRAJ_RUNNING;
}

/* ==================== 恢复旋转 (独立, 不覆盖 g_single) ==================== */

void trajectory_start_recovery_rotate(float theta, float speed, int dir)
{
    g_traj.recovery.active        = 1;
    g_traj.recovery.target_angle  = theta;
    g_traj.recovery.direction     = dir;
    g_traj.recovery.speed         = speed;
    g_traj.recovery.accum         = 0.0f;
    g_traj.recovery.timeout_ticks = 500;  /* 5 秒超时 */
}

void trajectory_stop_recovery_rotate(void)
{
    g_traj.recovery.active = 0;
    g_traj.ff_v_L = 0.0f;
    g_traj.ff_v_R = 0.0f;
    motor_control_update_target(MOTOR_L_ID, 0);
    motor_control_update_target(MOTOR_R_ID, 0);
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
    return trajectory_run_path(&g_single, 1, 1);
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
    return trajectory_run_path(&g_single, 1, 1);
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

    g_traj.segs     = 0;
    g_traj.num_segs = 0;
    g_traj.loop     = 0;
    g_traj.ff_v_L   = v_target;
    g_traj.ff_v_R   = v_target;
    g_traj.status          = TRAJ_RUNNING;
    g_traj.remaining_ticks = 0;
    g_traj.recovery.active = 0;
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
        { SEG_STRAIGHT, 0,   1.0f,  0.2f, 0,  0, 0 },
        { SEG_ARC,      0.4f, 3.14f, 0.15f,-1, 1, 1 },
        { SEG_STRAIGHT, 0,   1.0f,  0.2f, 0,  0, 0 },
        { SEG_ARC,      0.4f, 3.14f, 0.15f,-1, 1, 1 },
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
    g_traj.recovery.active = 0;
    motor_control_set_speed(MOTOR_L_ID, 0);
    motor_control_set_speed(MOTOR_R_ID, 0);
}

/* ==================== ISR 状态机调度 (10ms) ==================== */

void trajectory_recovery_update(void)
{
    enum { R_IDLE, R_BRAKE, R_ROTATE, R_LINE_FOUND, R_STABLE };
    static uint8_t phase  = R_IDLE;
    static uint8_t timer  = 0;

    steering_request_t req = Steering_GetRequest();
    traj_status_t ts = trajectory_get_status();

    if (ts == TRAJ_IDLE) {
        phase = R_IDLE;
        timer = 0;
        g_traj.speed_factor = 1.0f;
        return;
    }

    /* 重新丢线: 回到刹车 */
    if ((req == STEER_REQ_RECOVER_LEFT || req == STEER_REQ_RECOVER_RIGHT)
        && phase >= R_LINE_FOUND) {
        phase = R_BRAKE;
        timer = STEERING_BRAKE_TICKS;
    }

    switch (phase) {
    case R_IDLE:
        if (req == STEER_REQ_RECOVER_LEFT || req == STEER_REQ_RECOVER_RIGHT) {
            if (ts == TRAJ_RUNNING) {
                trajectory_pause();
                phase = R_BRAKE;
                timer = STEERING_BRAKE_TICKS;
            }
        }
        break;

    case R_BRAKE:
        if (timer > 0) {
            timer--;
            break;
        }
        /* 刹车完毕, 启动旋转 */
        {
            int dir = (req == STEER_REQ_RECOVER_LEFT) ? 1 : -1;
            trajectory_start_recovery_rotate(360.0f, STEERING_ROTATE_SPEED, dir);
            phase = R_ROTATE;
        }
        break;

    case R_ROTATE:
        if (req == STEER_REQ_LINE_FOUND) {
            trajectory_stop_recovery_rotate();
            phase = R_LINE_FOUND;
        }
        break;

    case R_LINE_FOUND:
        if (req == STEER_REQ_NONE) {
            trajectory_resume();
            g_traj.speed_factor = STEERING_STABLE_SPEED;
            phase = R_STABLE;
            timer = STEERING_STABLE_TICKS;
        }
        break;

    case R_STABLE:
        if (timer > 0) {
            timer--;
        } else {
            g_traj.speed_factor = 1.0f;
            phase = R_IDLE;
        }
        break;
    }
}

void trajectory_update(void)
{
    /* ---- 恢复旋转优先 ---- */
    if (g_traj.recovery.active) {
        if (g_traj.recovery.timeout_ticks == 0) {
            g_traj.recovery.active = 0;
            g_traj.ff_v_L = 0.0f;
            g_traj.ff_v_R = 0.0f;
            motor_control_update_target(MOTOR_L_ID, 0);
            motor_control_update_target(MOTOR_R_ID, 0);
            return;
        }
        g_traj.recovery.timeout_ticks--;

        float v   = g_traj.recovery.speed * g_traj.speed_factor;
        int   dir = g_traj.recovery.direction;
        g_traj.ff_v_L = (float)(-dir) * v;
        g_traj.ff_v_R = (float)( dir) * v;

        float w = (2.0f * v) / WHEEL_BASE;
        g_traj.recovery.accum += w * CTRL_DT;

        if (g_traj.recovery.accum >= g_traj.recovery.target_angle) {
            g_traj.ff_v_L = 0.0f;
            g_traj.ff_v_R = 0.0f;
            motor_control_update_target(MOTOR_L_ID, 0);
            motor_control_update_target(MOTOR_R_ID, 0);
            g_traj.recovery.active = 0;
            return;
        }

        motor_control_update_target(MOTOR_L_ID,
            (int32_t)(g_traj.ff_v_L * MPS_TO_MOTOR_RPM) * MOTOR_L_SIGN);
        motor_control_update_target(MOTOR_R_ID,
            (int32_t)(g_traj.ff_v_R * MPS_TO_MOTOR_RPM) * MOTOR_R_SIGN);
        return;
    }

    /* ---- 非运行态不调度 ---- */
    if (g_traj.status != TRAJ_RUNNING) return;

    /* ---- 循迹模式 (trajectory_linefollow) ---- */
    if (g_traj.num_segs == 0 && g_traj.closed_loop && g_traj.feedback) {
        static float corr_filt = 0.0f;
        static int   lf_div    = 0;
        float corr = g_traj.feedback();
        corr_filt = 0.3f * corr + 0.7f * corr_filt;
        if (++lf_div >= 5) {
            lf_div = 0;
            float base_rpm  = g_traj.ff_v_L * MPS_TO_MOTOR_RPM * g_traj.speed_factor;
            float delta_rpm = corr_filt * MPS_TO_MOTOR_RPM;
            motor_control_update_target(MOTOR_L_ID,
                (int32_t)((base_rpm + delta_rpm) * MOTOR_L_SIGN));
            motor_control_update_target(MOTOR_R_ID,
                (int32_t)((base_rpm - delta_rpm) * MOTOR_R_SIGN));
        }
        return;
    }

    apply_speed();

    /* 圆弧/旋转: 陀螺仪角度停止 */
    if (g_traj.num_segs > 0 && g_traj.segs &&
        (g_traj.segs[g_traj.seg_index].type == SEG_ARC ||
         g_traj.segs[g_traj.seg_index].type == SEG_ROTATE)) {

        float yaw = Gyro_ReadYawRate();
        g_traj.rotate_accum += fabs_f(yaw) * (PI_F / 180.0f) * CTRL_DT;

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
                next = 0;
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

/* ==================== 闭环控制接口 ==================== */

void trajectory_set_feedback(traj_feedback_fn fn) { g_traj.feedback = fn; }

void trajectory_enable_closed_loop(uint8_t enable)
{
    g_traj.closed_loop = enable ? 1 : 0;
}

void trajectory_set_speed_factor(float f)
{
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    g_traj.speed_factor = f;
}

/* ==================== 状态查询 ==================== */

traj_status_t trajectory_get_status(void)        { return g_traj.status; }
uint8_t       trajectory_get_segment_index(void) { return g_traj.seg_index; }

uint32_t trajectory_get_remaining_ms(void)
{
    return g_traj.remaining_ticks * (uint32_t)(CTRL_DT * 1000.0f);
}
