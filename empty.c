#include "ti_msp_dl_config.h"
#include "./Drivers/uart.h"

#define UART_TX_DELAY (160000)

static uint8_t gByte;
static uint16_t gCount;
static uint8_t gBuf[128];

int main(void)
{
    SYSCFG_DL_init();
    UART_Init();
    delay_cycles(UART_TX_DELAY);

    UART_Puts(&g_uart0, "MSP!\r\n");

    DL_GPIO_clearPins(GPIO_LEDS_PORT,
        (GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN));

    while (1) {
        if (UART_ReadByte(&g_uart0, &gByte)) {
            if (gCount < sizeof(gBuf))
                gBuf[gCount++] = gByte;

            if (gByte == 0x0A) {
                UART_Write(&g_uart0, gBuf, gCount);
                gCount = 0;
            }
        }
    }
}
