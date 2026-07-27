/*
 * ttc2 最小迁移验证
 * - UART: ttc2 原生 DMA TX + ISR RX
 * - 灰度传感器: Track_Tracking_Car 迁移 (GPIO 引脚待 SysConfig 配置)
 */

#include "ti_msp_dl_config.h"
#include "./Drivers/uart.h"
#include "./Drivers/grayscale.h"

/* 后续迁移时取消注释:
#include "./Drivers/motor_control.h"
#include "./Drivers/trajectory.h"
#include "./Drivers/steering.h"
#include "./Drivers/line_pid.h"
#include "./Drivers/gyro_pid.h"
#include "./Drivers/mpu6500.h"
#include "./Drivers/filter.h"
*/

#include <stdio.h>
#include <string.h>
/* #include <math.h> */

#define CMD_BUF_SIZE      64
#define UART_TX_DELAY     160000

/* ==================================================================
 * 后续迁移: IMU / 滤波器 / 动态漂移变量组 (暂注释)
 * ================================================================== */
#if 0
typedef struct {
    float roll;
    float pitch;
    float yaw;
} IMU_Attitude;

volatile IMU_Attitude g_myCarAngle = {0.0f, 0.0f, 0.0f};
volatile bool g_is_calibrated = false;
volatile float g_yaw_rate = 0.0f;
static volatile int g_imu_ready = 0;

BiquadFilter xFilter, yFilter;
KalmanFilter2D kfX, kfY;
float raw_angle_z = 0.0f;
float gyro_z_offset = 0.0f;
float yaw_drift_rate = 0.0f;
uint32_t total_cycles_since_reset = 0;
uint32_t stationary_cycles = 0;
#endif

/* ==================== UART 数据上报 ==================== */

static void firewater_send(void)
{
    uint8_t sensor = Grayscale_Read();
    char b[32];
    int n = sprintf(b, "GS=0x%02X ", sensor);
    if (n > 0) UART_Puts(&g_uart0, b);
    Grayscale_PrintBinary8(sensor);

    /* 后续迁移: 加入电机/IMU 数据上报
    motor_control_get_target_rpm(1), ...
    g_myCarAngle.roll, g_myCarAngle.pitch, g_myCarAngle.yaw
    */
}

/* ==================== 命令解析 ==================== */

static void cmd_show(void)
{
    UART_Puts(&g_uart0, "ttc2 CMD: ? gs udbg\r\n");
}

static void cmd_do(const char *line)
{
    char k[16] = {0};
    float v1 = 0, v2 = 0, v3 = 0;
    int dir = 1;

    if (line[0] == '\0') return;
    sscanf(line, "%15s", k);

    if (!strcmp(k, "?")) { cmd_show(); return; }

    if (!strcmp(k, "gs")) {
        uint8_t s = Grayscale_Read();
        char b[9];
        for (int i = 7; i >= 0; i--) b[7 - i] = (s & (1 << i)) ? '1' : '0';
        b[8] = '\0';
        UART_Printf(&g_uart0, "GS=%s\r\n", b);
        return;
    }
    if (!strcmp(k, "udbg")) { UART_DumpDebug(&g_uart0); return; }

    /* ============ 后续迁移: 电机 / 轨迹 / PID 命令 (暂注释) ============ */
#if 0
    if (!strcmp(k, "stop_all")) { trajectory_stop(); UART_Puts(&g_uart0, "OK all stopped\r\n"); return; }
    if (!strcmp(k, "imu")) {
        UART_Printf(&g_uart0, "Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, YawRate: %.2f deg/s\r\n",
            g_myCarAngle.roll, g_myCarAngle.pitch, g_myCarAngle.yaw, g_yaw_rate);
        return;
    }
    if (sscanf(line, "%*s %f", &v1) >= 1) {
        if (!strcmp(k, "Lp")) { LinePID_SetKp(v1); UART_Printf(&g_uart0, "OK Lp=%.3f\r\n", v1); return; }
        if (!strcmp(k, "Li")) { LinePID_SetKi(v1); UART_Printf(&g_uart0, "OK Li=%.3f\r\n", v1); return; }
        if (!strcmp(k, "Ld")) { LinePID_SetKd(v1); UART_Printf(&g_uart0, "OK Ld=%.3f\r\n", v1); return; }
        if (!strcmp(k, "Gp")) { GyroPID_SetKp(v1); UART_Printf(&g_uart0, "OK Gp=%.3f\r\n", v1); return; }
        if (!strcmp(k, "Gi")) { GyroPID_SetKi(v1); UART_Printf(&g_uart0, "OK Gi=%.3f\r\n", v1); return; }
        if (!strcmp(k, "Gd")) { GyroPID_SetKd(v1); UART_Printf(&g_uart0, "OK Gd=%.3f\r\n", v1); return; }
    }
    if (!strcmp(k, "mode")) {
        int m;
        if (sscanf(line, "mode %d", &m) == 1) {
            Steering_SetMode((uint8_t)m);
            UART_Printf(&g_uart0, "OK mode %s\r\n", m==0?"A(line)":"B(gyro)");
        } else UART_Puts(&g_uart0, "ERR: mode 0(A/line) or 1(B/gyro)\r\n");
        return;
    }
    if (!strcmp(k, "st")) {
        if (sscanf(line, "st %f %f", &v1, &v2) == 2) {
            trajectory_straight(v1, v2);
            UART_Printf(&g_uart0, "OK st d=%.2f v=%.2f\r\n", v1, v2);
        } else UART_Puts(&g_uart0, "ERR: st <dist_m> <speed_mps>\r\n");
        return;
    }
    if (!strcmp(k, "stop")) {
        motor_control_stop(1); motor_control_stop(2);
        UART_Puts(&g_uart0, "OK both stopped\r\n"); return;
    }
    if (!strcmp(k, "Tr")) {
        if (sscanf(line, "Tr %f", &v1) == 1) {
            motor_control_set_speed(1, (int32_t)v1);
            motor_control_set_speed(2, (int32_t)v1);
            UART_Printf(&g_uart0, "OK both Tr=%d\r\n", (int)v1);
        } else UART_Puts(&g_uart0, "ERR: Tr <rpm>\r\n");
        return;
    }
    /* 单电机命令解析 */
    {
        int len = (int)strlen(k);
        int id = 0;
        if (len > 0 && (k[len - 1] == '1' || k[len - 1] == '2')) {
            id = k[len - 1] - '0';
            k[len - 1] = '\0';
        }
        if (id == 0) { UART_Printf(&g_uart0, "ERR: unknown cmd '%s'\r\n", k); return; }
        float v = 0;
        if (sscanf(line, "%*s %f", &v) < 1 && strcmp(k, "stop")) {
            UART_Printf(&g_uart0, "ERR: M%d %s needs value\r\n", id, k);
            return;
        }
        if (!strcmp(k, "Tr")) {
            motor_control_set_speed((uint8_t)id, (int32_t)v);
            UART_Printf(&g_uart0, "OK M%d Tr=%d\r\n", id, (int)v);
        } else if (!strcmp(k, "Kp")) {
            motor_control_set_kp((uint8_t)id, v);
            UART_Printf(&g_uart0, "OK M%d Kp=%.3f\r\n", id, v);
        } else if (!strcmp(k, "Ki")) {
            motor_control_set_ki((uint8_t)id, v);
            UART_Printf(&g_uart0, "OK M%d Ki=%.3f\r\n", id, v);
        } else if (!strcmp(k, "Kd")) {
            motor_control_set_kd((uint8_t)id, v);
            UART_Printf(&g_uart0, "OK M%d Kd=%.3f\r\n", id, v);
        } else if (!strcmp(k, "stop")) {
            motor_control_stop((uint8_t)id);
            UART_Printf(&g_uart0, "OK M%d stopped\r\n", id);
        } else { UART_Printf(&g_uart0, "ERR: %s\r\n", k); }
    }
#endif

    UART_Printf(&g_uart0, "ERR: unknown '%s'\r\n", k);
}

static void cmd_poll(void)
{
    static char b[CMD_BUF_SIZE]; static int i = 0;
    uint8_t c;
    int budget = 2 * CMD_BUF_SIZE;
    while (budget-- > 0 && UART_ReadByte(&g_uart0, &c)) {
        if (c == '\n' || c == '\r') {
            if (i > 0) { b[i] = 0; cmd_do(b); i = 0; }
        } else if (i < CMD_BUF_SIZE - 1) {
            b[i++] = (char)c;
        }
    }
}

/* ==================================================================
 * 后续迁移: TIMG12 ISR (100Hz / 10ms) (暂注释)
 * ================================================================== */
#if 0
static volatile int g_fw_ready = 0;
static volatile int g_fw_div    = 0;

void TIMER_0_INST_IRQHandler(void)
{
    motor_control_update();
    trajectory_update();
    g_imu_ready = 1;
    if (++g_fw_div >= 5) { g_fw_div = 0; g_fw_ready = 1; }
    DL_TimerG_clearInterruptStatus(TIMER_0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
}
#endif

/* ==================== Main ==================== */

int main(void)
{
    SYSCFG_DL_init();
    UART_Init();
    delay_cycles(UART_TX_DELAY);

    /* ttc2 的 UART_Init 已开启 RX 中断, 无需额外 UART_RxEnable */

    Grayscale_Init();

    /* 后续迁移: IMU / 电机 / 轨迹 初始化 (暂注释)
    NVIC_SetPriority(I2C_GYRO_INST_INT_IRQN, 0);
    NVIC_EnableIRQ(I2C_GYRO_INST_INT_IRQN);
    DL_SYSCTL_disableSleepOnExit();
    Biquad_Init(&xFilter, ...);
    Kalman2D_Init(&kfX, SAMPLE_DT);
    motor_control_init(1, 0.01f, 2.0f, 10.0f, 0.0f);
    motor_control_init(2, 0.01f, 2.0f, 10.0f, 0.0f);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);
    Steering_Init();
    trajectory_set_feedback(Steering_GetCorrection);
    trajectory_enable_closed_loop(1);
    MPU6500_Init();
    */

    UART_Puts(&g_uart0, "ttc2 ready\r\n");
    cmd_show();

    while (1) {
        cmd_poll();

        /* 后续迁移: IMU 姿态解算 + 定时上报
        if (g_is_calibrated && g_imu_ready) { g_imu_ready = 0; IMU_UpdateAttitude(&g_myCarAngle); }
        if (g_fw_ready) { g_fw_ready = 0; firewater_send(); }
        */
    }
}
