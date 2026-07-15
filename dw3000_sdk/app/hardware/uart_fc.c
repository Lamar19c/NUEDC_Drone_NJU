/**
 * uart_fc.c — USART1 飞控通信 (PA9=TX, PA10=RX) 115200
 *   发送 mc 帧到 stm32_uwb / FC
 */
#include "hardware/board.h"

UART_HandleTypeDef huart_fc;

void uart_fc_send(const uint8_t *buf, uint16_t len) {
    HAL_UART_Transmit(&huart_fc, (uint8_t *)buf, len, 100);
}

void uart_fc_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* TX — 复用推挽 */
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin   = FC_TX_PIN;
    HAL_GPIO_Init(FC_TX_PORT, &g);

    /* RX — 上拉输入 */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin  = FC_RX_PIN;
    HAL_GPIO_Init(FC_RX_PORT, &g);

    huart_fc.Instance          = USART1;
    huart_fc.Init.BaudRate     = FC_UART_BAUD;
    huart_fc.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_fc.Init.StopBits     = UART_STOPBITS_1;
    huart_fc.Init.Parity       = UART_PARITY_NONE;
    huart_fc.Init.Mode         = UART_MODE_TX_RX;
    HAL_UART_Init(&huart_fc);
}
