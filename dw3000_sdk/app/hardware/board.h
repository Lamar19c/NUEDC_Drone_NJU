/**
 * board.h — NJU UWB Tag 引脚定义
 * Netlist_UWB_NJU_2026-07-14  STM32F103CBT6 + DWM3000
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f1xx_hal.h"

/* ================================================================
 * 时钟: 8MHz HSE → PLL×9 → 72MHz
 * ================================================================ */
#define HSE_VALUE    8000000UL

/* ================================================================
 * SPI1 — DWM3000 (PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI)
 * ================================================================ */
#define DWM_SPI            SPI1
#define DWM_CS_PORT        GPIOA
#define DWM_CS_PIN         GPIO_PIN_4
#define DWM_SCK_PORT       GPIOA
#define DWM_SCK_PIN        GPIO_PIN_5
#define DWM_MISO_PORT      GPIOA
#define DWM_MISO_PIN       GPIO_PIN_6
#define DWM_MOSI_PORT      GPIOA
#define DWM_MOSI_PIN       GPIO_PIN_7

/* ================================================================
 * DWM3000 控制 GPIO (PB0=IRQ, PB1=RSTN, PB10=WAKEUP)
 * ================================================================ */
#define DWM_IRQ_PORT       GPIOB
#define DWM_IRQ_PIN        GPIO_PIN_0
#define DWM_RSTN_PORT      GPIOB
#define DWM_RSTN_PIN       GPIO_PIN_1
#define DWM_WAKEUP_PORT    GPIOB
#define DWM_WAKEUP_PIN     GPIO_PIN_10

/* ================================================================
 * USART1 — FC mc 输出 (PA9=TX, PA10=RX)  115200
 * ================================================================ */
#define FC_UART            USART1
#define FC_UART_BAUD       115200
#define FC_TX_PORT         GPIOA
#define FC_TX_PIN          GPIO_PIN_9
#define FC_RX_PORT         GPIOA
#define FC_RX_PIN          GPIO_PIN_10

/* ================================================================
 * USART2 — Debug printf (PA2=TX, PA3=RX)  115200
 * ================================================================ */
#define DBG_UART           USART2
#define DBG_UART_BAUD      115200
#define DBG_TX_PORT        GPIOA
#define DBG_TX_PIN         GPIO_PIN_2
#define DBG_RX_PORT        GPIOA
#define DBG_RX_PIN         GPIO_PIN_3

/* ================================================================
 * I2C1 — EEPROM AT24C64 (PB6=SCL, PB7=SDA)  0x50
 * ================================================================ */
#define EEP_I2C            I2C1
#define EEP_SCL_PORT       GPIOB
#define EEP_SCL_PIN        GPIO_PIN_6
#define EEP_SDA_PORT       GPIOB
#define EEP_SDA_PIN        GPIO_PIN_7
#define EEP_I2C_ADDR       (0x50 << 1)

/* ================================================================
 * LED (高电平点亮: PB8=RUN, PB9=UWB, PB11=FIX)
 * ================================================================ */
#define LED_RUN_PORT       GPIOB
#define LED_RUN_PIN        GPIO_PIN_8
#define LED_UWB_PORT       GPIOB
#define LED_UWB_PIN        GPIO_PIN_9
#define LED_FIX_PORT       GPIOB
#define LED_FIX_PIN        GPIO_PIN_11

/* ================================================================
 * 全局句柄 (extern)
 * ================================================================ */
extern SPI_HandleTypeDef  hspi_dwm;
extern UART_HandleTypeDef huart_fc;
extern UART_HandleTypeDef huart_dbg;
extern I2C_HandleTypeDef  hi2c_eep;

/* ================================================================
 * 初始化函数
 * ================================================================ */
void board_clock_init(void);
void board_init(void);

void spi_dw3000_init(void);
void gpio_dw3000_init(void);
void uart_fc_init(void);
void uart_dbg_init(void);
void i2c_eep_init(void);
void led_init(void);

#endif
