#include "uart.h"

/* ================================================================
 *  Layer 1 — HAL: 环形缓冲 / DMA / ISR
 * ================================================================ */

static uint8_t g_rx_buf[UART0_RX_SIZE];
UART_Port g_uart0;

/* ---------- 环形缓冲 ---------- */

static void ring_init(ring_buf_t *r, uint8_t *buf, uint16_t size)
{
    r->buf  = buf;
    r->mask = (uint16_t)(size - 1);
    r->head = 0;
    r->tail = 0;
}

static inline uint16_t ring_count(const ring_buf_t *r)
{
    return (uint16_t)((r->head - r->tail) & r->mask);
}

static inline int ring_push(ring_buf_t *r, uint8_t b)
{
    uint16_t next = (uint16_t)((r->head + 1) & r->mask);
    if (next == r->tail) return 0;
    r->buf[r->head] = b;
    r->head = next;
    return 1;
}

static inline int ring_pop(ring_buf_t *r, uint8_t *b)
{
    if (r->head == r->tail) return 0;
    *b = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1) & r->mask);
    return 1;
}

/* ---------- DMA TX (内部, busy-wait) ---------- */

static void dma_tx(UART_Port *port, const uint8_t *data, uint16_t len)
{
    if (len == 0) return;

    DL_DMA_setSrcAddr(DMA, port->dma_chan, (uint32_t)data);
    DL_DMA_setDestAddr(DMA, port->dma_chan,
        (uint32_t)(&port->inst->TXDATA));
    DL_DMA_setTransferSize(DMA, port->dma_chan, len);

    port->dma_done = false;
    port->eot_done = false;

    DL_DMA_enableChannel(DMA, port->dma_chan);

    while (!port->dma_done) {}
    while (!port->eot_done) {}
}

/* ---------- 公开 API ---------- */

void UART_Init(void)
{
    g_uart0.inst        = UART_0_INST;
    g_uart0.irqn        = UART_0_INST_INT_IRQN;
    g_uart0.dma_chan    = DMA_CH0_CHAN_ID;
    g_uart0.rx_overflow = 0;
    g_uart0.dma_done    = false;
    g_uart0.eot_done    = false;
    g_uart0.hw_overrun  = 0;
    g_uart0.framing     = 0;
    g_uart0.parity      = 0;
    g_uart0.framer      = 0;

    ring_init(&g_uart0.rx, g_rx_buf, UART0_RX_SIZE);

    DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);

    DL_SYSCTL_disableSleepOnExit();
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

int UART_ReadByte(UART_Port *port, uint8_t *b)
{
    return ring_pop(&port->rx, b);
}

uint16_t UART_Write(UART_Port *port, const uint8_t *data, uint16_t len)
{
    dma_tx(port, data, len);
    return len;
}

/* ================================================================
 *  Layer 2 — 传输: Puts / Printf / WriteByte
 * ================================================================ */

int UART_WriteByte(UART_Port *port, uint8_t b)
{
    dma_tx(port, &b, 1);
    return 1;
}

int UART_Puts(UART_Port *port, const char *s)
{
    uint16_t len = 0;
    while (s[len]) len++;
    if (len) dma_tx(port, (const uint8_t *)s, len);
    return (int)len;
}

/* ---------- Printf 精简版 (soft-float, 无 libc 依赖) ---------- */

typedef struct {
    UART_Port *port;
    int        count;
} fmt_ctx_t;

static void uart_emitc(UART_Port *port, char *buf, int *pos,
                       int *total, char c)
{
    buf[(*pos)++] = c;
    (*total)++;
    if (*pos >= PRINTF_BUF_SIZE) {
        dma_tx(port, (uint8_t *)buf, PRINTF_BUF_SIZE);
        *pos = 0;
    }
}

static int uart_print_u32(UART_Port *port, char *buf, int pos, int *total,
                           unsigned long v, unsigned base, int upper)
{
    char tmp[11];
    int  i = 0;
    const char *dig = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    if (v == 0) tmp[i++] = '0';
    while (v) { tmp[i++] = dig[v % base]; v /= base; }
    while (i) uart_emitc(port, buf, &pos, total, tmp[--i]);
    return pos;
}

static int uart_print_s32(UART_Port *port, char *buf, int pos, int *total,
                           long v, unsigned base, int upper)
{
    if (v < 0) {
        uart_emitc(port, buf, &pos, total, '-');
        return uart_print_u32(port, buf, pos, total, (unsigned long)(-v), base, upper);
    }
    return uart_print_u32(port, buf, pos, total, (unsigned long)v, base, upper);
}

static int uart_print_str(UART_Port *port, char *buf, int pos, int *total,
                           const char *s)
{
    if (!s) s = "(null)";
    while (*s) uart_emitc(port, buf, &pos, total, *s++);
    return pos;
}

static int uart_print_float(UART_Port *port, char *buf, int pos, int *total,
                             double v, int prec)
{
    if (prec < 0) prec = 6;
    if (v < 0) { uart_emitc(port, buf, &pos, total, '-'); v = -v; }

    double half = 0.5;
    for (int i = 0; i < prec; i++) half *= 0.1;
    v += half;

    unsigned long ip   = (unsigned long)v;
    double        frac = v - (double)ip;
    pos = uart_print_u32(port, buf, pos, total, ip, 10, 0);

    if (prec > 0) {
        uart_emitc(port, buf, &pos, total, '.');
        for (int i = 0; i < prec; i++) {
            frac *= 10.0;
            int d = (int)frac;
            uart_emitc(port, buf, &pos, total, (char)('0' + d));
            frac -= d;
        }
    }
    return pos;
}

int UART_Printf(UART_Port *port, const char *fmt, ...)
{
    char buf[PRINTF_BUF_SIZE];
    int pos = 0, total = 0;
    va_list ap;
    va_start(ap, fmt);

    for (; *fmt; fmt++) {
        if (*fmt != '%') { uart_emitc(port, buf, &pos, &total, *fmt); continue; }
        fmt++;

        int prec = -1;
        if (*fmt == '.') {
            fmt++; prec = 0;
            while (*fmt >= '0' && *fmt <= '9') { prec = prec * 10 + (*fmt - '0'); fmt++; }
        }

        int lng = 0;
        while (*fmt == 'l') { lng++; fmt++; }

        switch (*fmt) {
        case 'd': case 'i':
            pos = uart_print_s32(port, buf, pos, &total,
                                 lng ? va_arg(ap, long) : va_arg(ap, int), 10, 0);
            break;
        case 'u':
            pos = uart_print_u32(port, buf, pos, &total,
                                 lng ? va_arg(ap, unsigned long) : va_arg(ap, unsigned), 10, 0);
            break;
        case 'x':
            pos = uart_print_u32(port, buf, pos, &total,
                                 lng ? va_arg(ap, unsigned long) : va_arg(ap, unsigned), 16, 0);
            break;
        case 'X':
            pos = uart_print_u32(port, buf, pos, &total,
                                 lng ? va_arg(ap, unsigned long) : va_arg(ap, unsigned), 16, 1);
            break;
        case 'c':
            uart_emitc(port, buf, &pos, &total, (char)va_arg(ap, int));
            break;
        case 's':
            pos = uart_print_str(port, buf, pos, &total, va_arg(ap, const char *));
            break;
        case 'f': case 'F':
            pos = uart_print_float(port, buf, pos, &total, va_arg(ap, double), prec);
            break;
        case '%':
            uart_emitc(port, buf, &pos, &total, '%');
            break;
        case '\0': goto printf_done;
        default:
            uart_emitc(port, buf, &pos, &total, '%');
            uart_emitc(port, buf, &pos, &total, *fmt);
            break;
        }
    }
printf_done:
    va_end(ap);
    if (pos > 0) dma_tx(port, (uint8_t *)buf, (uint16_t)pos);
    return total;
}

/* ================================================================
 *  Layer 3 — 协议: Framer / CRC / 帧发送 / 错误 & 调试
 * ================================================================ */

/* ---------- Framer ---------- */

static uint16_t framer_fixed_feed(UART_Framer *f, uint8_t b)
{
    if (f->idx == 0) {
        if (b == f->head) f->buf[f->idx++] = b;
    } else if (f->idx < (uint16_t)(f->frame_len - 1)) {
        f->buf[f->idx++] = b;
    } else {
        if (b == f->tail) {
            f->buf[f->idx] = b;
            f->idx = 0;
            return f->frame_len;
        }
        f->idx = 0;
    }
    return 0;
}

static uint16_t framer_delim_feed(UART_Framer *f, uint8_t b)
{
    if (f->idx < f->size)
        f->buf[f->idx++] = b;
    else
        f->idx = 0;

    if (b == f->delim) {
        uint16_t len = f->idx;
        f->idx = 0;
        return len;
    }
    return 0;
}

static uint16_t framer_len_feed(UART_Framer *f, uint8_t b)
{
    if (f->idx == 0) {
        if (b == f->head) f->buf[f->idx++] = b;
        return 0;
    }

    if (f->idx < f->size)
        f->buf[f->idx++] = b;
    else
        { f->idx = 0; return 0; }

    uint16_t min_len = (uint16_t)f->len_offset + f->len_size + f->crc_size + 1;
    if (f->idx < min_len) return 0;

    if (f->idx == min_len) {
        f->payload_len = 0;
        for (uint8_t k = 0; k < f->len_size; k++)
            f->payload_len |= (uint16_t)f->buf[f->len_offset + k] << (8 * k);
        if (f->payload_len == 0) { f->idx = 0; return 0; }
    }

    uint16_t total = min_len + f->payload_len;
    if (f->idx >= total) {
        if (f->buf[total - 1] == f->tail) {
            f->idx = 0;
            return total;
        }
        f->idx = 0;
    }
    return 0;
}

void UART_FramerInitFixed(UART_Framer *f, uint8_t *buf, uint16_t size,
                          uint8_t head, uint8_t tail, uint16_t frame_len)
{
    f->type      = UART_FRAMER_FIXED;
    f->Feed      = framer_fixed_feed;
    f->OnFrame   = 0;
    f->buf       = buf;
    f->size      = size;
    f->idx       = 0;
    f->head      = head;
    f->tail      = tail;
    f->frame_len = frame_len;
    f->delim     = 0;
}

void UART_FramerInitDelim(UART_Framer *f, uint8_t *buf, uint16_t size,
                          uint8_t delim)
{
    f->type      = UART_FRAMER_DELIM;
    f->Feed      = framer_delim_feed;
    f->OnFrame   = 0;
    f->buf       = buf;
    f->size      = size;
    f->idx       = 0;
    f->head      = 0;
    f->tail      = 0;
    f->frame_len = 0;
    f->delim     = delim;
}

void UART_FramerInitLen(UART_Framer *f, uint8_t *buf, uint16_t size,
                        uint8_t head, uint8_t tail,
                        uint8_t len_offset, uint8_t len_size,
                        uint8_t crc_offset, uint8_t crc_size)
{
    f->type        = UART_FRAMER_LEN;
    f->Feed        = framer_len_feed;
    f->OnFrame     = 0;
    f->buf         = buf;
    f->size        = size;
    f->idx         = 0;
    f->head        = head;
    f->tail        = tail;
    f->frame_len   = 0;
    f->delim       = 0;
    f->payload_len = 0;
    f->len_offset  = len_offset;
    f->len_size    = len_size;
    f->crc_offset  = crc_offset;
    f->crc_size    = crc_size;
}

void UART_FramerInitCustom(UART_Framer *f, UART_FramerFeedFn feed,
                           UART_FramerFrameFn on_frame)
{
    f->type    = UART_FRAMER_CUSTOM;
    f->Feed    = feed;
    f->OnFrame = on_frame;
    f->buf     = 0;
    f->size    = 0;
    f->idx     = 0;
}

void UART_FramerSetCallback(UART_Framer *f, UART_FramerFrameFn on_frame)
{
    f->OnFrame = on_frame;
}

uint16_t UART_FramerPollBytes(UART_Framer *f, const uint8_t *data, uint16_t len)
{
    uint16_t frames = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t flen = f->Feed(f, data[i]);
        if (flen && f->OnFrame) {
            f->OnFrame(f->buf, flen);
            frames++;
        }
    }
    return frames;
}

uint16_t UART_FramerPoll(UART_Port *port)
{
    UART_Framer *f = (UART_Framer *)port->framer;
    if (!f) return 0;

    uint16_t frames = 0;
    uint16_t avail = ring_count(&port->rx);
    for (uint16_t i = 0; i < avail; i++) {
        uint8_t b;
        if (ring_pop(&port->rx, &b)) {
            uint16_t flen = f->Feed(f, b);
            if (flen && f->OnFrame) {
                f->OnFrame(f->buf, flen);
                frames++;
            }
        }
    }
    return frames;
}

/* ---------- CRC-16-CCITT ---------- */

uint16_t UART_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---------- 帧发送工具 ---------- */

uint16_t UART_SendFrameFixed(UART_Port *port,
                             uint8_t head, uint8_t tail,
                             const uint8_t *payload, uint16_t len)
{
    UART_WriteByte(port, head);
    if (payload && len) dma_tx(port, payload, len);
    UART_WriteByte(port, tail);
    return len + 2;
}

uint16_t UART_SendFrameLenCRC(UART_Port *port,
                              uint8_t head, uint8_t tail,
                              uint8_t cmd,
                              const uint8_t *payload, uint16_t len)
{
    uint16_t data_len = (uint16_t)(len + 1);

    UART_WriteByte(port, head);
    UART_WriteByte(port, (uint8_t)(data_len & 0xFF));
    UART_WriteByte(port, (uint8_t)((data_len >> 8) & 0xFF));
    UART_WriteByte(port, cmd);
    if (payload && len) dma_tx(port, payload, len);

    uint8_t crc_buf[258];
    crc_buf[0] = (uint8_t)(data_len & 0xFF);
    crc_buf[1] = (uint8_t)((data_len >> 8) & 0xFF);
    crc_buf[2] = cmd;
    if (payload && len) {
        for (uint16_t i = 0; i < len && i < 255; i++)
            crc_buf[3 + i] = payload[i];
    }
    uint16_t crc = UART_CRC16(crc_buf, (uint16_t)(3 + len));

    UART_WriteByte(port, (uint8_t)(crc & 0xFF));
    UART_WriteByte(port, (uint8_t)((crc >> 8) & 0xFF));
    UART_WriteByte(port, tail);

    return (uint16_t)(6 + len);
}

/* ---------- 错误 & 恢复 ---------- */

const UART_Port *UART_GetErrors(UART_Port *port)
{
    return port;
}

void UART_ClearErrors(UART_Port *port)
{
    port->rx_overflow = 0;
    port->hw_overrun  = 0;
    port->framing     = 0;
    port->parity      = 0;
}

void UART_Recover(UART_Port *port)
{
    UART_Regs *inst = port->inst;

    DL_UART_Main_disableInterrupt(inst,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_TX);

    while (!DL_UART_isRXFIFOEmpty(inst))
        DL_UART_Main_receiveData(inst);
    while (!DL_UART_isTXFIFOEmpty(inst))
        (void)DL_UART_isTXFIFOEmpty(inst);

    port->rx.tail = port->rx.head;
    port->dma_done = false;
    port->eot_done = false;

    while (DL_UART_Main_getPendingInterrupt(inst) != DL_UART_IIDX_NO_INTERRUPT) {}

    DL_UART_Main_enableInterrupt(inst, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(port->irqn);
}

/* ---------- 调试 ---------- */

static char *u32toa(char *p, unsigned long v)
{
    char tmp[11]; int i = 0;
    if (v == 0) tmp[i++] = '0';
    while (v) { tmp[i++] = (char)('0' + (v % 10)); v /= 10; }
    while (i) *p++ = tmp[--i];
    return p;
}

static char *ustr(char *p, const char *s)
{
    while (*s) *p++ = *s++;
    return p;
}

void UART_DumpDebug(UART_Port *port)
{
    char b[160], *p = b;
    uint16_t rx_used = ring_count(&port->rx);

    p = ustr(p, "\r\n--- UART DEBUG ---\r\n");
    p = ustr(p, "RX ring: "); p = u32toa(p, rx_used);
    p = ustr(p, "/");          p = u32toa(p, port->rx.mask);
    p = ustr(p, " used\r\n");
    p = ustr(p, "Errors: OE="); p = u32toa(p, (unsigned long)port->hw_overrun);
    p = ustr(p, " FE=");        p = u32toa(p, (unsigned long)port->framing);
    p = ustr(p, " PE=");        p = u32toa(p, (unsigned long)port->parity);
    p = ustr(p, " OVF=");       p = u32toa(p, (unsigned long)port->rx_overflow);
    p = ustr(p, "\r\n------------------\r\n");

    dma_tx(port, (uint8_t *)b, (uint16_t)(p - b));
}

/* ================================================================
 *  ISR (库持有)
 * ================================================================ */

static inline void rx_isr(UART_Port *p, uint8_t b)
{
    if (!ring_push(&p->rx, b))
        p->rx_overflow++;
}

void UART_0_INST_IRQHandler(void)
{
    UART_Port *p = &g_uart0;
    DL_UART_IIDX idx;
    int limit = 128;

    while (limit-- > 0 &&
           (idx = DL_UART_Main_getPendingInterrupt(UART_0_INST)) != DL_UART_IIDX_NO_INTERRUPT) {

        switch (idx) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_isRXFIFOEmpty(UART_0_INST))
                rx_isr(p, DL_UART_Main_receiveData(UART_0_INST));
            break;

        case DL_UART_MAIN_IIDX_EOT_DONE:
            p->eot_done = true;
            break;

        case DL_UART_MAIN_IIDX_DMA_DONE_TX:
            p->dma_done = true;
            break;

        case DL_UART_IIDX_OVERRUN_ERROR:
            p->hw_overrun++;
            if (!DL_UART_isRXFIFOEmpty(UART_0_INST))
                (void)DL_UART_Main_receiveData(UART_0_INST);
            break;

        case DL_UART_IIDX_FRAMING_ERROR:
            p->framing++;
            if (!DL_UART_isRXFIFOEmpty(UART_0_INST))
                (void)DL_UART_Main_receiveData(UART_0_INST);
            break;

        case DL_UART_IIDX_PARITY_ERROR:
            p->parity++;
            if (!DL_UART_isRXFIFOEmpty(UART_0_INST))
                (void)DL_UART_Main_receiveData(UART_0_INST);
            break;

        default:
            break;
        }
    }
}
