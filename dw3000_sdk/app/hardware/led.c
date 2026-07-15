/**
 * led.c — LED 指示灯 (高电平点亮)
 *   PB8=RUN(绿) PB9=UWB(绿) PB11=FIX(红)
 *
 * 宏: LED_RUN_ON/OFF  LED_UWB_ON/OFF  LED_FIX_ON/OFF
 */
#include "hardware/board.h"

void led_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = LED_RUN_PIN | LED_UWB_PIN | LED_FIX_PIN;
    HAL_GPIO_Init(LED_RUN_PORT, &g);

    HAL_GPIO_WritePin(LED_RUN_PORT, LED_RUN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_UWB_PORT, LED_UWB_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_FIX_PORT, LED_FIX_PIN, GPIO_PIN_RESET);
}

void led_run(uint8_t on)   { HAL_GPIO_WritePin(LED_RUN_PORT, LED_RUN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void led_uwb(uint8_t on)   { HAL_GPIO_WritePin(LED_UWB_PORT, LED_UWB_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void led_fix(uint8_t on)   { HAL_GPIO_WritePin(LED_FIX_PORT, LED_FIX_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
