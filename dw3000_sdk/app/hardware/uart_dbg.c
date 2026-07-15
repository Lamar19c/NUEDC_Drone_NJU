/**
 * uart_dbg.c — USART2 Debug 串口 (PA2=TX, PA3=RX) 115200
 *   重定向 printf → _write()
 */
#include "hardware/board.h"

UART_HandleTypeDef huart_dbg;

void uart_dbg_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin   = DBG_TX_PIN;
    HAL_GPIO_Init(DBG_TX_PORT, &g);

    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin  = DBG_RX_PIN;
    HAL_GPIO_Init(DBG_RX_PORT, &g);

    huart_dbg.Instance          = USART2;
    huart_dbg.Init.BaudRate     = DBG_UART_BAUD;
    huart_dbg.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_dbg.Init.StopBits     = UART_STOPBITS_1;
    huart_dbg.Init.Parity       = UART_PARITY_NONE;
    huart_dbg.Init.Mode         = UART_MODE_TX_RX;
    HAL_UART_Init(&huart_dbg);
}

/* ── printf 重定向 ── */
int _write(int fd, char *ptr, int len) {
    HAL_UART_Transmit(&huart_dbg, (uint8_t *)ptr, len, 100);
    return len;
}
