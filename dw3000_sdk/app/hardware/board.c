/**
 * board.c — NJU UWB Tag 顶层初始化
 *   调用各外设子模块: spi / gpio / uart / i2c / led
 */
#include "hardware/board.h"

/* ================================================================
 * 时钟: 8MHz HSE → PLL×9 → 72MHz
 * ================================================================ */
void board_clock_init(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

/* ================================================================
 * 板级初始化
 * ================================================================ */
void board_init(void) {
    HAL_Init();
    board_clock_init();

    gpio_dw3000_init();   /* RSTN / WAKEUP / IRQ */
    led_init();           /* PB8/9/11 */
    spi_dw3000_init();    /* SPI1 → DW3000 */
    uart_fc_init();       /* USART1 → FC */
    uart_dbg_init();      /* USART2 → Debug */
    i2c_eep_init();       /* I2C1 → EEPROM */
}
