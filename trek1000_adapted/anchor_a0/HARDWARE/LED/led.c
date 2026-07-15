/* ── Schematic2 适配 ──
 * LED1=PC15(蓝)  LED2=PB8(绿,TX)  LED3=PB9(红,RX)
 * 原 PB6→I2C_SCL(EEPROM), PB7→I2C_SDA(EEPROM), 不可用
 * LED_PC7 映射到 PB1 (无物理LED, 防编译错误)
 */
#include "led.h"

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    /* PB8+PB9 (LED2+LED3) + PB1 (phantom LED) */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* PC15 (LED1) */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void led_off(led_t led)
{
    switch (led) {
    case LED_PC6:  GPIO_ResetBits(GPIOC, GPIO_Pin_15); break;
    case LED_PC7:  GPIO_ResetBits(GPIOB, GPIO_Pin_1);  break;
    case LED_PC8:  GPIO_ResetBits(GPIOB, GPIO_Pin_8);  break;
    case LED_PC9:  GPIO_ResetBits(GPIOB, GPIO_Pin_9);  break;
    case LED_ALL:
        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
        break;
    default: break;
    }
}

void led_on(led_t led)
{
    switch (led) {
    case LED_PC6:  GPIO_SetBits(GPIOC, GPIO_Pin_15); break;
    case LED_PC7:  GPIO_SetBits(GPIOB, GPIO_Pin_1);  break;
    case LED_PC8:  GPIO_SetBits(GPIOB, GPIO_Pin_8);  break;
    case LED_PC9:  GPIO_SetBits(GPIOB, GPIO_Pin_9);  break;
    case LED_ALL:
        GPIO_SetBits(GPIOC, GPIO_Pin_15);
        GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
        break;
    default: break;
    }
}
