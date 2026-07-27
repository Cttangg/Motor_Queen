#include "mpu6500.h"
#include "uart.h"
#include <stdio.h>

/* 官方中断常规缓冲大小 */
#define I2C_TX_MAX_PACKET_SIZE (16)
#define I2C_RX_MAX_PACKET_SIZE (16)

/* 内部状态机变量与快取 */
static uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE];
static uint32_t gTxLen, gTxCount;

static uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
static uint32_t gRxLen, gRxCount;

enum I2cControllerStatus {
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_INPROGRESS,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_INPROGRESS,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR,
};

static volatile enum I2cControllerStatus gI2cControllerStatus;

/* ==================== 内部私有：I2C 驱动函数 ==================== */

static bool I2C_WritePacket(uint8_t targetAddr, uint8_t *pBuffer, uint8_t length) {
    if (length > I2C_TX_MAX_PACKET_SIZE) return false;

    // 【关键修复】显式使能外设级别的 TX_DONE、STOP 和 NACK 中断
    DL_I2C_enableInterrupt(I2C_GYRO_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_STOP    |
        DL_I2C_INTERRUPT_CONTROLLER_NACK);

    DL_I2C_disableInterrupt(I2C_GYRO_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    DL_I2C_flushControllerTXFIFO(I2C_GYRO_INST);

    for (uint8_t i = 0; i < length; i++) gTxPacket[i] = pBuffer[i];
    gTxLen   = length;
    gTxCount = DL_I2C_fillControllerTXFIFO(I2C_GYRO_INST, &gTxPacket[0], gTxLen);

    if (gTxCount < gTxLen) {
        DL_I2C_enableInterrupt(I2C_GYRO_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    }

    gI2cControllerStatus = I2C_STATUS_TX_STARTED;

    uint32_t timeout = 10000;
    while (!(DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) && --timeout);
    if (timeout == 0) {
        UART_Puts(&g_uart0, "[DEBUG] I2C Bus Not Idle Timeout!\r\n");
        gI2cControllerStatus = I2C_STATUS_IDLE;
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_GYRO_INST, targetAddr, DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen);
    DL_Common_delayCycles(32); /* Errata I2C_ERR_13 */

    timeout = 10000;
    while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
           (gI2cControllerStatus != I2C_STATUS_ERROR) &&
           (timeout > 0)) {
        __WFE();
        timeout--;
    }

    // 处理失败并打印精准原因
    if (timeout == 0 || gI2cControllerStatus == I2C_STATUS_ERROR) {
        if (timeout == 0) {
            UART_Puts(&g_uart0, "[DEBUG] Reason: ISR Wait Timeout (Interrupt Not Firing)\r\n");
        }
        if (gI2cControllerStatus == I2C_STATUS_ERROR) {
            UART_Puts(&g_uart0, "[DEBUG] Reason: I2C NACK / Slave No Response\r\n");
        }

        DL_I2C_flushControllerTXFIFO(I2C_GYRO_INST);
        gI2cControllerStatus = I2C_STATUS_IDLE;
        return false;
    }

    timeout = 10000;
    while ((DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout);

    gI2cControllerStatus = I2C_STATUS_IDLE;
    DL_Common_delayCycles(1600);
    return true;
}

/* ==================== 修复后的 I2C 读取函数 ==================== */

static bool I2C_ReadPacket(uint8_t targetAddr, uint8_t regAddr, uint8_t *pBuffer, uint8_t length) {
    if (length > I2C_RX_MAX_PACKET_SIZE) return false;

    /* ----- 第 1 阶段：发送寄存器地址 (TX) ----- */
    DL_I2C_enableInterrupt(I2C_GYRO_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_STOP    |
        DL_I2C_INTERRUPT_CONTROLLER_NACK);

    DL_I2C_disableInterrupt(I2C_GYRO_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    DL_I2C_flushControllerTXFIFO(I2C_GYRO_INST);

    gTxPacket[0] = regAddr;
    gTxLen       = 1;
    gTxCount     = DL_I2C_fillControllerTXFIFO(I2C_GYRO_INST, &gTxPacket[0], gTxLen);

    gI2cControllerStatus = I2C_STATUS_TX_STARTED;

    uint32_t timeout = 10000;
    while (!(DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) && --timeout);
    if (timeout == 0) {
        gI2cControllerStatus = I2C_STATUS_IDLE;
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_GYRO_INST, targetAddr, DL_I2C_CONTROLLER_DIRECTION_TX, gTxLen);
    DL_Common_delayCycles(32); /* Errata I2C_ERR_13 */

    timeout = 10000;
    while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
           (gI2cControllerStatus != I2C_STATUS_ERROR) &&
           (timeout > 0)) {
        __WFE();
        timeout--;
    }

    if (timeout == 0 || gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(I2C_GYRO_INST);
        gI2cControllerStatus = I2C_STATUS_IDLE;
        return false;
    }

    timeout = 10000;
    while ((DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout);

    DL_Common_delayCycles(1000);

    /* ----- 第 2 阶段：读取数据 (RX) ----- */
    
    // 【修改点 4】：增加 RX FIFO 提前清理，防止上一次残存数据干涉
    DL_I2C_flushControllerRXFIFO(I2C_GYRO_INST);

    gRxLen               = length;
    gRxCount             = 0;
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;

    // 【修改点 3】：显式使能 RXFIFO_TRIGGER 中断，确保数据到达时触发 ISR
    DL_I2C_enableInterrupt(I2C_GYRO_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE           |
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER    |
        DL_I2C_INTERRUPT_CONTROLLER_STOP              |
        DL_I2C_INTERRUPT_CONTROLLER_NACK);

    timeout = 10000;
    while (!(DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) && --timeout);
    if (timeout == 0) {
        gI2cControllerStatus = I2C_STATUS_IDLE;
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_GYRO_INST, targetAddr, DL_I2C_CONTROLLER_DIRECTION_RX, gRxLen);
    DL_Common_delayCycles(32); /* Errata I2C_ERR_13 */

    timeout = 10000;
    while ((gI2cControllerStatus != I2C_STATUS_RX_COMPLETE) &&
           (gI2cControllerStatus != I2C_STATUS_ERROR) &&
           (timeout > 0)) {
        __WFE();
        timeout--;
    }

    if (timeout == 0 || gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerRXFIFO(I2C_GYRO_INST);
        gI2cControllerStatus = I2C_STATUS_IDLE;
        return false;
    }

    timeout = 10000;
    while ((DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout);

    for (uint8_t i = 0; i < length; i++) {
        pBuffer[i] = gRxPacket[i];
    }
    gI2cControllerStatus = I2C_STATUS_IDLE;
    return true;
}
static bool I2C_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2]; buf[0] = reg; buf[1] = value;
    return I2C_WritePacket(MPU6500_ADDR, buf, 2);
}

static bool I2C_ReadRegs(uint8_t reg, uint8_t *value, uint8_t len) {
    return I2C_ReadPacket(MPU6500_ADDR, reg, value, len);
}

/* ==================== 公共功能实现 ==================== */
bool MPU6500_Init(void) {
    uint8_t who_am_i = 0;
    char log_buf[64];

    UART_Puts(&g_uart0, "\r\n===== MPU6500 Init Diagnostics =====\r\n");

    DL_Common_delayCycles(3200000);

    // 1. 复位器件
    if (!I2C_WriteReg(MPU6500_PWR_MGMT_1, 0x80)) {
        UART_Puts(&g_uart0, "[FAIL] Step 1: Software Reset (PWR_MGMT_1=0x80)\r\n");
        return false;
    }
    UART_Puts(&g_uart0, "[ OK ] Step 1: Software Reset\r\n");
    DL_Common_delayCycles(3200000);

    // 2. 解除休眠，选择时钟源
    if (!I2C_WriteReg(MPU6500_PWR_MGMT_1, 0x01)) {
        UART_Puts(&g_uart0, "[FAIL] Step 2: Clock Config (PWR_MGMT_1=0x01)\r\n");
        return false;
    }
    UART_Puts(&g_uart0, "[ OK ] Step 2: Clock Config\r\n");

    // 3. 采样率分频
    if (!I2C_WriteReg(MPU6500_SMPLRT_DIV, 0x07)) {
        UART_Puts(&g_uart0, "[FAIL] Step 3: SMPLRT_DIV\r\n");
        return false;
    }
    UART_Puts(&g_uart0, "[ OK ] Step 3: SMPLRT_DIV\r\n");

    // 4. 低通滤波配置
    if (!I2C_WriteReg(MPU6500_CONFIG, 0x03)) {
        UART_Puts(&g_uart0, "[FAIL] Step 4: CONFIG\r\n");
        return false;
    }
    UART_Puts(&g_uart0, "[ OK ] Step 4: CONFIG\r\n");

    // 5. 陀螺仪量程配置
    if (!I2C_WriteReg(MPU6500_GYRO_CONFIG, 0x18)) {
        UART_Puts(&g_uart0, "[FAIL] Step 5: GYRO_CONFIG\r\n");
        return false;
    }
    UART_Puts(&g_uart0, "[ OK ] Step 5: GYRO_CONFIG\r\n");

    // 6. 加速度计量程配置
    if (!I2C_WriteReg(MPU6500_ACCEL_CONFIG, 0x00)) {
        UART_Puts(&g_uart0, "[FAIL] Step 6: ACCEL_CONFIG\r\n");
        return false;
    }
    UART_Puts(&g_uart0, "[ OK ] Step 6: ACCEL_CONFIG\r\n");

    // 7. 读取并打印芯片 ID
    if (!I2C_ReadRegs(MPU6500_WHO_AM_I, &who_am_i, 1)) {
        UART_Puts(&g_uart0, "[FAIL] Step 7: Read WHO_AM_I (I2C Bus Error)\r\n");
        return false;
    }

    snprintf(log_buf, sizeof(log_buf), "[INFO] WHO_AM_I Read Output: 0x%02X\r\n", who_am_i);
    UART_Puts(&g_uart0, log_buf);

    // MPU6500 官方 ID 为 0x70；MPU6050 为 0x68；MPU9250 为 0x71
    if (who_am_i != 0x70 && who_am_i != 0x68 && who_am_i != 0x71) {
        UART_Puts(&g_uart0, "[FAIL] Step 7: Unrecognized WHO_AM_I Value!\r\n");
        return false;
    }

    UART_Puts(&g_uart0, "===== MPU6500 Init SUCCESS =====\r\n");
    return true;
}

bool MPU6500_ReadIMU(MPU6500_IMUData *data) {
    uint8_t buf[14];
    if (!I2C_ReadRegs(MPU6500_ACCEL_XOUT_H, buf, 14)) return false;

    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    data->accel_x = (float)ax / 16384.0f;
    data->accel_y = (float)ay / 16384.0f;
    data->accel_z = (float)az / 16384.0f;

    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
    data->gyro_x = (float)gx / 16.4f;
    data->gyro_y = (float)gy / 16.4f;
    data->gyro_z = (float)gz / 16.4f;

    return true;
}

/* I2C 中断服务常式 (ISR) */
/* ==================== 修复后的 I2C 中断服务函数 ==================== */

void I2C_GYRO_INST_IRQHandler(void) {
    switch (DL_I2C_getPendingInterrupt(I2C_GYRO_INST)) {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            while (DL_I2C_isControllerRXFIFOEmpty(I2C_GYRO_INST) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] =
                        DL_I2C_receiveControllerData(I2C_GYRO_INST);
                } else {
                    DL_I2C_receiveControllerData(I2C_GYRO_INST);
                }
            }
            gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            break;

        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(
                I2C_GYRO_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            gI2cControllerStatus = I2C_STATUS_TX_COMPLETE;
            break;

        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_RX_INPROGRESS;
            while (DL_I2C_isControllerRXFIFOEmpty(I2C_GYRO_INST) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] =
                        DL_I2C_receiveControllerData(I2C_GYRO_INST);
                } else {
                    DL_I2C_receiveControllerData(I2C_GYRO_INST);
                }
            }
            // 【修改点 2】：当 RX FIFO 读取数量达到要求时，提前切为 RX_COMPLETE，避免死等 RX_DONE
            if (gRxCount >= gRxLen) {
                gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            if (gTxCount < gTxLen) {
                gI2cControllerStatus = I2C_STATUS_TX_INPROGRESS;
                gTxCount += DL_I2C_fillControllerTXFIFO(
                    I2C_GYRO_INST, &gTxPacket[gTxCount], gTxLen - gTxCount);
            } else {
                DL_I2C_disableInterrupt(
                    I2C_GYRO_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_STOP:
            // 【修改点 1】：完善 STOP 状态机，增加对 RX 阶段的收尾闭环
            if (gI2cControllerStatus == I2C_STATUS_TX_INPROGRESS && gTxCount >= gTxLen) {
                gI2cControllerStatus = I2C_STATUS_TX_COMPLETE;
            } else if (gI2cControllerStatus == I2C_STATUS_RX_STARTED ||
                       gI2cControllerStatus == I2C_STATUS_RX_INPROGRESS) {
                gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            if (gI2cControllerStatus != I2C_STATUS_IDLE) {
                gI2cControllerStatus = I2C_STATUS_ERROR;
            }
            break;

        default:
            break;
    }
}