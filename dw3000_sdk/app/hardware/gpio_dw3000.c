/**
 * gpio_dw3000.c — DW3000 控制引脚 (RSTN, WAKEUP, IRQ)
 *   PB0=IRQ(EXTI0上升沿) PB1=RSTN(输出) PB10=WAKEUP(输出)
 */
#include "hardware/board.h"

void dw3000_reset(void) {
    HAL_GPIO_WritePin(DWM_RSTN_PORT, DWM_RSTN_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(DWM_RSTN_PORT, DWM_RSTN_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
}

void dw3000_wakeup_pin(uint8_t level) {
    HAL_GPIO_WritePin(DWM_WAKEUP_PORT, DWM_WAKEUP_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void gpio_dw3000_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* RSTN + WAKEUP — 推挽输出 */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = DWM_RSTN_PIN | DWM_WAKEUP_PIN;
    HAL_GPIO_Init(DWM_RSTN_PORT, &g);
    HAL_GPIO_WritePin(DWM_RSTN_PORT, DWM_RSTN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DWM_WAKEUP_PORT, DWM_WAKEUP_PIN, GPIO_PIN_RESET);

    /* IRQ — 上升沿中断输入 */
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_PULLDOWN;
    g.Pin  = DWM_IRQ_PIN;
    HAL_GPIO_Init(DWM_IRQ_PORT, &g);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
