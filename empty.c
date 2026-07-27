/*
 * ttc2 最小迁移验证
 * - UART: ttc2 原生 DMA TX + ISR RX
 * - 灰度传感器: Track_Tracking_Car 迁移
 * - 陀螺仪: Track_Tracking_Car 迁移 (I2C 待 SysConfig 配置)
 */

#include "ti_msp_dl_config.h"
#include "./Drivers/uart.h"
#include "./Drivers/grayscale.h"
#include "./Drivers/mpu6500.h"
#include "./Drivers/filter.h"

/* 后续迁移时取消注释:
#include "./Drivers/motor_control.h"
#include "./Drivers/trajectory.h"
#include "./Drivers/steering.h"
#include "./Drivers/line_pid.h"
#include "./Drivers/gyro_pid.h"
*/

#include <stdio.h>
#include <string.h>
#include <math.h>

#define CMD_BUF_SIZE      64
#define UART_TX_DELAY     160000

/* ==================================================================
 * IMU 姿态解算变量组
 * ================================================================== */

typedef struct {
    float roll;
    float pitch;
    float yaw;
} IMU_Attitude;

volatile IMU_Attitude g_myCarAngle = {0.0f, 0.0f, 0.0f};
volatile bool g_is_calibrated = false;
volatile float g_yaw_rate = 0.0f;

#define SAMPLE_DT  0.01f

BiquadFilter xFilter, yFilter;
KalmanFilter2D kfX, kfY;
float raw_angle_z = 0.0f;
float gyro_z_offset = 0.0f;
float yaw_drift_rate = 0.0f;
uint32_t total_cycles_since_reset = 0;
uint32_t stationary_cycles = 0;

#define YAW_RESET_INTERVAL_CYCLES  3000
#define STATIONARY_THRESHOLD_CYCLES 300

/* ------------------------------------------------------------------
 * IMU 姿态更新 (100Hz 调用)
 * ------------------------------------------------------------------ */
static bool IMU_UpdateAttitude(volatile IMU_Attitude *attitude)
{
    MPU6500_IMUData imuData;
    if (!MPU6500_ReadIMU(&imuData)) return false;

    float accAngleX = atan2f(imuData.accel_y, imuData.accel_z) * 57.29578f;
    float accAngleY = atan2f(-imuData.accel_x,
        sqrtf(imuData.accel_y * imuData.accel_y +
              imuData.accel_z * imuData.accel_z)) * 57.29578f;

    float lowpassAccX = Biquad_Process(&xFilter, accAngleX);
    float lowpassAccY = Biquad_Process(&yFilter, accAngleY);

    Kalman2D_Predict(&kfX);
    Kalman2D_Predict(&kfY);
    Kalman2D_Update(&kfX, lowpassAccX, imuData.gyro_x);
    Kalman2D_Update(&kfY, lowpassAccY, imuData.gyro_y);

    attitude->roll  = kfX.x[0];
    attitude->pitch = kfY.x[0];

    float corrected_gyro_z = imuData.gyro_z - gyro_z_offset;
    bool is_stationary = (fabsf(imuData.gyro_x) < 1.5f) &&
                         (fabsf(imuData.gyro_y) < 1.5f) &&
                         (fabsf(corrected_gyro_z) < 1.5f);

    total_cycles_since_reset++;
    if (is_stationary) {
        stationary_cycles++;
        if (total_cycles_since_reset >= YAW_RESET_INTERVAL_CYCLES &&
            stationary_cycles >= STATIONARY_THRESHOLD_CYCLES) {
            float elapsed_time = total_cycles_since_reset * SAMPLE_DT;
            yaw_drift_rate = raw_angle_z / elapsed_time;
            if (fabsf(yaw_drift_rate) < 0.1f) {
                raw_angle_z = 0.0f;
                total_cycles_since_reset = 0;
                stationary_cycles = 0;
            }
        }
    } else {
        stationary_cycles = 0;
    }

    float final_gyro_z_rate = corrected_gyro_z - yaw_drift_rate;
    if (fabsf(final_gyro_z_rate) < 0.3f) final_gyro_z_rate = 0.0f;

    g_yaw_rate = final_gyro_z_rate;
    raw_angle_z += final_gyro_z_rate * SAMPLE_DT;

    while (raw_angle_z >= 180.0f) raw_angle_z -= 360.0f;
    while (raw_angle_z < -180.0f) raw_angle_z += 360.0f;

    attitude->yaw = raw_angle_z;
    return true;
}

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
    UART_Puts(&g_uart0, "ttc2 CMD: ? gs imu udbg\r\n");
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

    if (!strcmp(k, "imu")) {
        if (!g_is_calibrated) {
            UART_Puts(&g_uart0, "IMU not calibrated\r\n");
            return;
        }
        IMU_UpdateAttitude(&g_myCarAngle);
        UART_Printf(&g_uart0,
            "Roll: %.2f  Pitch: %.2f  Yaw: %.2f deg  YawRate: %.2f deg/s\r\n",
            g_myCarAngle.roll, g_myCarAngle.pitch, g_myCarAngle.yaw, g_yaw_rate);
        return;
    }

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

    /* ---- IMU 初始化 ---- */
    NVIC_SetPriority(I2C_GYRO_INST_INT_IRQN, 0);
    NVIC_EnableIRQ(I2C_GYRO_INST_INT_IRQN);
    DL_SYSCTL_disableSleepOnExit();

    if (MPU6500_Init()) {
        /* ---- 滤波器初始化 ---- */
        Biquad_Init(&xFilter, 0.0133592f, 0.0267184f, 0.0133592f,
                    1.0f, -1.64745998f, 0.70089678f);
        Biquad_Init(&yFilter, 0.0133592f, 0.0267184f, 0.0133592f,
                    1.0f, -1.64745998f, 0.70089678f);
        Kalman2D_Init(&kfX, SAMPLE_DT);
        Kalman2D_Init(&kfY, SAMPLE_DT);

        /* ---- Z 轴陀螺仪零偏校准 (保持静止) ---- */
        UART_Puts(&g_uart0, "Calibrating Z-Gyro, KEEP STILL...\r\n");
        float gyro_z_sum = 0.0f;
        int valid_samples = 0;
        while (valid_samples < 200) {
            MPU6500_IMUData calData;
            if (MPU6500_ReadIMU(&calData)) {
                gyro_z_sum += calData.gyro_z;
                valid_samples++;
            }
            delay_cycles(320000);
        }
        gyro_z_offset = gyro_z_sum / 200.0f;
        g_is_calibrated = true;
        UART_Printf(&g_uart0, "Gyro Offset: %.4f deg/s. MPU6500 OK\r\n",
                    gyro_z_offset);
    } else {
        UART_Puts(&g_uart0, "MPU6500 FAIL\r\n");
    }

    /* 后续迁移: 电机 / 轨迹 初始化 (暂注释)
    Biquad_Init(&xFilter, ...);
    Kalman2D_Init(&kfX, SAMPLE_DT);
    motor_control_init(1, 0.01f, 2.0f, 10.0f, 0.0f);
    motor_control_init(2, 0.01f, 2.0f, 10.0f, 0.0f);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);
    Steering_Init();
    trajectory_set_feedback(Steering_GetCorrection);
    trajectory_enable_closed_loop(1);
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
