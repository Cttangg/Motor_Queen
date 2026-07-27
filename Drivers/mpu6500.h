#ifndef MPU6500_H_
#define MPU6500_H_

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/* ------------------------------------------------------------------
 * 兼容性别名：将原始码使用的通用名称映射到 SysConfig 生成的实例名称
 * ------------------------------------------------------------------ */
#define I2C_INST               I2C_GYRO_INST
#define I2C_INST_INT_IRQN      I2C_GYRO_INST_INT_IRQN
#define I2C_INST_IRQHandler    I2C_GYRO_INST_IRQHandler

/* MPU6500 7位设备地址 */
#define MPU6500_ADDR         0x68

/* 核心寄存器地址定义 */
#define MPU6500_SMPLRT_DIV   0x19  // 采样率分频器
#define MPU6500_CONFIG       0x1A  // 配置寄存器(低通滤波器)
#define MPU6500_GYRO_CONFIG  0x1B  // 陀螺仪配置(量程)
#define MPU6500_ACCEL_CONFIG 0x1C  // 新增：加速度计配置寄存器
#define MPU6500_PWR_MGMT_1   0x6B  // 电源管理 1
#define MPU6500_WHO_AM_I     0x75  // 器件ID寄存器

/* 6轴全数据寄存器首地址（从加速度计开始） */
#define MPU6500_ACCEL_XOUT_H 0x3B  // 新增：加速度计X轴高字节

// 升级后的 6 轴数据结构体
typedef struct {
    float accel_x; // 转换后的实际加速度 (单位: g)
    float accel_y; 
    float accel_z;
    float gyro_x;  // 转换后的实际角速度 (单位: °/s)
    float gyro_y;
    float gyro_z;
} MPU6500_IMUData;

// 函数声明
bool MPU6500_Init(void);
bool MPU6500_ReadIMU(MPU6500_IMUData *data); // 升级为全数据读取接口

#endif /* MPU6500_H_ */