#ifndef __MOTOR_CONTROL_H_
#define __MOTOR_CONTROL_H_

#include <stdint.h>

/* ===================== 双电机控制模块 ===================== */

/* motorID: 1 或 2 */

void motor_control_init(uint8_t motorID, float dt_sec,
                        float kp, float ki, float kd);
void motor_control_set_speed(uint8_t motorID, int32_t rpm);
void motor_control_update_target(uint8_t motorID, int32_t rpm); /* 仅改目标, 不重置PID */
void motor_control_stop(uint8_t motorID);
void motor_control_set_duty(uint8_t motorID, uint32_t duty);

/* 在 ISR 中调用一次, 更新所有已初始化的电机 */
void motor_control_update(void);

int32_t  motor_control_get_target_rpm(uint8_t motorID);
int32_t  motor_control_get_actual_rpm(uint8_t motorID);
uint32_t motor_control_get_duty(uint8_t motorID);
uint32_t motor_control_get_freq(uint8_t motorID);

void motor_control_set_kp(uint8_t motorID, float kp);
void motor_control_set_ki(uint8_t motorID, float ki);
void motor_control_set_kd(uint8_t motorID, float kd);

#endif
