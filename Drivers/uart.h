#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/* ================================================================
 *  编译开关
 * ----------------------------------------------------------------
 *  默认使用 DMA 批量模式 (方案 2)：Puts / Printf 一次 DMA 发完。
 *  取消下面宏定义则回退到逐字节 DMA (方案 1)，方便调试。
 * ================================================================ */
#define UART_TX_DMA_BATCH

/* ================================================================
 *  Layer 1 — HAL
 * ================================================================ */

#define UART0_RX_SIZE         256
#define PRINTF_BUF_SIZE       128

typedef struct {
    uint8_t          *buf;
    uint16_t          mask;
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buf_t;

typedef struct _UART_Port {
    UART_Regs    *inst;
    IRQn_Type     irqn;
    uint8_t       dma_chan;

    ring_buf_t    rx;
    volatile uint32_t rx_overflow;

    volatile bool dma_done;
    volatile bool eot_done;

    /* 错误计数 */
    volatile uint32_t hw_overrun;
    volatile uint32_t framing;
    volatile uint32_t parity;

    /* 协议层将会挂接 framer 指针到此字段 */
    void *framer;
} UART_Port;

extern UART_Port g_uart0;

void     UART_Init(void);
int      UART_ReadByte(UART_Port *port, uint8_t *b);
uint16_t UART_Write(UART_Port *port, const uint8_t *data, uint16_t len);

/* ================================================================
 *  Layer 2 — 传输
 * ================================================================ */

int  UART_WriteByte(UART_Port *port, uint8_t b);
int  UART_Puts(UART_Port *port, const char *s);
int  UART_Printf(UART_Port *port, const char *fmt, ...);

/* ================================================================
 *  Layer 3 — 协议 & 调试
 * ================================================================ */

typedef enum {
    UART_FRAMER_NONE   = 0,
    UART_FRAMER_FIXED  = 1,
    UART_FRAMER_DELIM  = 2,
    UART_FRAMER_LEN    = 3,
    UART_FRAMER_CUSTOM = 4,
} UART_FramerType;

struct UART_Framer;
typedef uint16_t (*UART_FramerFeedFn)(struct UART_Framer *self, uint8_t byte);
typedef void     (*UART_FramerFrameFn)(const uint8_t *frame, uint16_t len);

typedef struct UART_Framer {
    UART_FramerType     type;
    UART_FramerFeedFn   Feed;
    UART_FramerFrameFn  OnFrame;
    uint8_t  *buf;
    uint16_t  size;
    uint16_t  idx;
    uint8_t   head, tail;
    uint16_t  frame_len;
    uint8_t   delim;
    uint16_t  payload_len;
    uint8_t   len_offset, len_size;
    uint8_t   crc_offset, crc_size;
} UART_Framer;

void     UART_FramerInitFixed(UART_Framer *f, uint8_t *buf, uint16_t size,
                              uint8_t head, uint8_t tail, uint16_t frame_len);
void     UART_FramerInitDelim(UART_Framer *f, uint8_t *buf, uint16_t size,
                              uint8_t delim);
void     UART_FramerInitLen(UART_Framer *f, uint8_t *buf, uint16_t size,
                            uint8_t head, uint8_t tail,
                            uint8_t len_offset, uint8_t len_size,
                            uint8_t crc_offset, uint8_t crc_size);
void     UART_FramerInitCustom(UART_Framer *f,
                               UART_FramerFeedFn feed,
                               UART_FramerFrameFn on_frame);
void     UART_FramerSetCallback(UART_Framer *f, UART_FramerFrameFn on_frame);
uint16_t UART_FramerPoll(UART_Port *port);
uint16_t UART_FramerPollBytes(UART_Framer *f, const uint8_t *data, uint16_t len);

uint16_t UART_CRC16(const uint8_t *data, uint16_t len);
uint16_t UART_SendFrameFixed(UART_Port *port,
                             uint8_t head, uint8_t tail,
                             const uint8_t *payload, uint16_t len);
uint16_t UART_SendFrameLenCRC(UART_Port *port,
                              uint8_t head, uint8_t tail,
                              uint8_t cmd,
                              const uint8_t *payload, uint16_t len);

const UART_Port *UART_GetErrors(UART_Port *port);
void             UART_ClearErrors(UART_Port *port);
void             UART_Recover(UART_Port *port);

void UART_DumpDebug(UART_Port *port);

#endif
