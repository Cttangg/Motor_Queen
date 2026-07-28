#include "steering.h"
#include "line_pid.h"
#include "gyro_pid.h"
#include "grayscale.h"

/* ============================================================
 * 闭环转向调度器 — 实现
 * ------------------------------------------------------------
 * steering 不直接控制运动。
 * 通过 Steering_GetRequest() 上报恢复请求，
 * trajectory_update() 读取请求后执行 pause / recovery_rotate / resume。
 *
 * 状态机:
 *   NORMAL → (失线 5 tick) → 上报 RECOVER → trajectory pause
 *   BRAKE (100ms) → 上报 ROTATE → ROTATE → 发现线 → RECOVER
 *   RECOVER (连续 5 tick) → STABLE (200ms 低速) → NORMAL (resume)
 * ============================================================ */

typedef enum {
    S_NORMAL      = 0,
    S_BRAKE       = 1,
    S_ROTATE      = 2,
    S_RECOVER     = 3,
    S_STABLE      = 4,
} steer_state_t;

static volatile steer_state_t g_state        = S_NORMAL;
static volatile uint8_t      g_mode         = 0;
static volatile uint32_t      g_lost_count   = 0;
static volatile uint32_t      g_recover_cnt  = 0;
static int                    g_last_dir     = 0;
static volatile uint32_t      g_brake_timer  = 0;
static volatile uint32_t      g_stable_timer = 0;
static volatile steering_request_t g_request = STEER_REQ_NONE;

void Steering_Init(void)
{
    LinePID_Init(0.09f, 0.05f, 0.003f);// for lf 0.7
    GyroPID_Init(0.5f, 0.02f, 0.0f);
    GyroPID_EnableHeadingLock(1);
    Gyro_Init();
    g_state       = S_NORMAL;
    g_mode        = 0;
    g_lost_count  = 0;
    g_recover_cnt = 0;
    g_last_dir    = 0;
    g_brake_timer = 0;
    g_stable_timer = 0;
    g_request     = STEER_REQ_NONE;
}

float Steering_GetCorrection(void)
{
    uint8_t sensor = Grayscale_Read();
    float corr;

    if (g_mode == 0) {
        corr = LinePID_Update(sensor);
    } else {
        corr = GyroPID_Update(Gyro_ReadYawRate());
    }

    /* 记录方向供失线恢复使用 */
    if      (corr < -0.001f) g_last_dir = -1;
    else if (corr >  0.001f) g_last_dir =  1;

    return corr;
}

void Steering_Poll(void)
{
    /* 仅在 trajectory 运行时检测 */
    traj_status_t traj_st = trajectory_get_status();
    uint8_t traj_active = (traj_st == TRAJ_RUNNING || traj_st == TRAJ_PAUSED);
    if (!traj_active) {
        g_request = STEER_REQ_NONE;
        return;
    }

    uint8_t sensor = Grayscale_Read();
    uint8_t line   = sensor != 0;

    if (line) {
        g_lost_count = 0;
    } else {
        if (g_lost_count < STEERING_LOST_MAX) g_lost_count++;
    }

    switch (g_state) {
    case S_NORMAL:
        if (g_lost_count >= STEERING_LOST_THRESHOLD) {
            g_request = STEER_REQ_RECOVER_RIGHT;
            if (g_last_dir == -1) g_request = STEER_REQ_RECOVER_LEFT;
            g_brake_timer = STEERING_BRAKE_TICKS;
            g_state = S_BRAKE;
        }
        break;

    case S_BRAKE:
        if (g_brake_timer > 0) {
            g_brake_timer--;
            break;
        }
        g_request = STEER_REQ_RECOVER_RIGHT;
        if (g_last_dir == -1) g_request = STEER_REQ_RECOVER_LEFT;
        g_state = S_ROTATE;
        break;

    case S_ROTATE:
        if (line) {
            g_request = STEER_REQ_LINE_FOUND;
            g_recover_cnt = 1;
            g_state = S_RECOVER;
        }
        break;

    case S_RECOVER:
        if (line) {
            g_recover_cnt++;
            if (g_recover_cnt >= STEERING_RECOVER_TICKS) {
                LinePID_Reset();
                GyroPID_Reset();
                g_request = STEER_REQ_NONE;
                g_stable_timer = STEERING_STABLE_TICKS;
                g_state = S_STABLE;
            }
        } else {
            g_recover_cnt = 0;
            g_request = STEER_REQ_RECOVER_RIGHT;
            if (g_last_dir == -1) g_request = STEER_REQ_RECOVER_LEFT;
            g_state = S_ROTATE;
        }
        break;

    case S_STABLE:
        if (g_stable_timer > 0) {
            g_stable_timer--;
            break;
        }
        g_request     = STEER_REQ_NONE;
        g_lost_count  = 0;
        g_recover_cnt = 0;
        g_state = S_NORMAL;
        break;
    }
}

steering_request_t Steering_GetRequest(void)
{
    return g_request;
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
    g_recover_cnt = 0;
    g_state = S_NORMAL;
    g_request = STEER_REQ_NONE;
}

uint8_t Steering_GetMode(void)
{
    return g_mode;
}
