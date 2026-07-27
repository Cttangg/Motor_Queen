#include "grayscale.h"
#include "uart.h"

void Grayscale_Init(void)
{
}

uint8_t Grayscale_Read(void)
{
    uint8_t result = 0;

    if (DL_GPIO_readPins(GPIO_SENSOR_PIN_0_PORT, GPIO_SENSOR_PIN_0_PIN))
        result |= (1 << 0);
    if (DL_GPIO_readPins(GPIO_SENSOR_PIN_1_PORT, GPIO_SENSOR_PIN_1_PIN))
        result |= (1 << 1);
    if (DL_GPIO_readPins(GPIO_SENSOR_PIN_2_PORT, GPIO_SENSOR_PIN_2_PIN))
        result |= (1 << 2);
    if (DL_GPIO_readPins(GPIO_SENSOR_PIN_3_PORT, GPIO_SENSOR_PIN_3_PIN))
        result |= (1 << 3);
    if (DL_GPIO_readPins(GPIO_SENSOR_PIN_4_PORT, GPIO_SENSOR_PIN_4_PIN))
        result |= (1 << 4);
    if (DL_GPIO_readPins(GPIO_SENSOR_PIN_5_PORT, GPIO_SENSOR_PIN_5_PIN))
        result |= (1 << 5);
    if (DL_GPIO_readPins(GPIO_SENSOR_PIN_6_PORT, GPIO_SENSOR_PIN_6_PIN))
        result |= (1 << 6);
    if (DL_GPIO_readPins(GPIO_SENSOR_PIN_7_PORT, GPIO_SENSOR_PIN_7_PIN))
        result |= (1 << 7);

    return result;
}

void Grayscale_PrintBinary8(uint8_t value)
{
    static const char bit_chars[2] = { '0', '1' };
    UART_WriteByte(&g_uart0, bit_chars[(value >> 7) & 1]);
    UART_WriteByte(&g_uart0, bit_chars[(value >> 6) & 1]);
    UART_WriteByte(&g_uart0, bit_chars[(value >> 5) & 1]);
    UART_WriteByte(&g_uart0, bit_chars[(value >> 4) & 1]);
    UART_WriteByte(&g_uart0, bit_chars[(value >> 3) & 1]);
    UART_WriteByte(&g_uart0, bit_chars[(value >> 2) & 1]);
    UART_WriteByte(&g_uart0, bit_chars[(value >> 1) & 1]);
    UART_WriteByte(&g_uart0, bit_chars[(value >> 0) & 1]);
    UART_WriteByte(&g_uart0, '\r');
    UART_WriteByte(&g_uart0, '\n');
}
