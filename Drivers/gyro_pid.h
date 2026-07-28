#ifndef GYRO_PID_H_
#define GYRO_PID_H_

#include <stdint.h>

/* ============================================================
 * 模式 B — 陀螺仪直行 PID
 * ------------------------------------------------------------
 * 陀螺仪经 I2C MPU6500 接入 (I2C_GYRO)。
 * Gyro_ReadYawRate() 获取 ω_z (°/s)。
 * PID 使 ω_z → 0, 即小车保持直线航向。
 * ============================================================ */

void Gyro_Init(void);
float Gyro_ReadYawRate(void);

void  GyroPID_Init(float kp, float ki, float kd);
void  GyroPID_SetKp(float kp);
void  GyroPID_SetKi(float ki);
void  GyroPID_SetKd(float kd);
void  GyroPID_Reset(void);
float GyroPID_Update(float yaw_rate);

/* 漂移校准 */
void  GyroPID_SetDriftOffset(float offset);  /* °/s */
float GyroPID_GetDriftOffset(void);

/* 航向锁定 */
void  GyroPID_EnableHeadingLock(uint8_t enable);
void  GyroPID_SetHeadingKp(float kp);
void  GyroPID_SetHeadingKi(float ki);
float GyroPID_GetHeading(void);

/* 诊断 */
float GyroPID_GetLastYawRate(void);
float GyroPID_GetLastCorrection(void);

#endif
