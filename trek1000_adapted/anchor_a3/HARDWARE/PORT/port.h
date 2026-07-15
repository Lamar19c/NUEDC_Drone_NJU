/**
 * port.h — TREK1000 移植版 (适配 Schematic2 网表引脚)
 *
 * 基于 TREK1000 Station/Tag 原始 port.h，修改 DW1000 GPIO 定义以适配:
 *   Netlist_Schematic2_2026-07-13.enet
 *   STM32F103RCT6 + DW1000 裸片 + HHM1595A1 Balun
 *
 * 改动摘要:
 *   RSTn:   PA0 → PB4  (原Trek1000用PA0, Schematic2网表用PB4)
 *   IRQ:    PB5 → PB0  (原Trek1000用PB5, Schematic2网表用PB0)
 *   WAKEUP: DW1000内部寄存器控制, 非GPIO (Schematic2网表PB3→DW1000 WAKEUP pin23)
 *   EXTON:  DW1000内部输出, 非GPIO (Schematic2网表PA15→DW1000 EXTON pin21)
 *
 * 替换方法:
 *   将本文件覆盖到 TREK1000 源码的 HARDWARE/PORT/port.h
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

//#define USB_SUPPORT

//#define LCD_UPDATE_ON (0)
#define DMA_ENABLE	(0)

#if (DMA_ENABLE == 1)
 #define writetospi		writetospi_dma
 #define readfromspi	readfromspi_dma
 void dma_init(void);
#else
#define writetospi		writetospi_serial
#define readfromspi	  readfromspi_serial
#endif

int SPI_Configuration(void);
void SPI_ChangeRate(uint16_t scalingfactor);
void SPI_ConfigFastRate(uint16_t scalingfactor);

#define SPIy_PRESCALER				SPI_BaudRatePrescaler_128

#define SPIx_PRESCALER		SPI_BaudRatePrescaler_8

/* ── SPI1: 与Schematic2匹配 PA4/PA5/PA6/PA7 ── */
#define SPIx							SPI1
#define SPIx_GPIO					GPIOA
#define SPIx_CS						GPIO_Pin_4
#define SPIx_CS_GPIO			GPIOA
#define SPIx_SCK					GPIO_Pin_5
#define SPIx_MISO					GPIO_Pin_6
#define SPIx_MOSI					GPIO_Pin_7

/* ── DW1000 RSTn: Schematic2 网表用 PB4 (原 Trek1000 用 PA0) ── */
#define DW1000_RSTn				GPIO_Pin_4
#define DW1000_RSTn_GPIO	GPIOB

/* ── DW1000 RSTn IRQ: 与 RSTn 同引脚, 用于检测 DW1000 复位完成 ── */
#define DECARSTIRQ                  GPIO_Pin_4
#define DECARSTIRQ_GPIO             GPIOB
#define DECARSTIRQ_EXTI             EXTI_Line4
#define DECARSTIRQ_EXTI_PORT        GPIO_PortSourceGPIOB
#define DECARSTIRQ_EXTI_PIN         GPIO_PinSource4
#define DECARSTIRQ_EXTI_IRQn        EXTI4_IRQn

/* ── DW1000 IRQ: Schematic2 网表用 PB0 (原 Trek1000 用 PB5) ── */
#define DECAIRQ                     GPIO_Pin_0
#define DECAIRQ_GPIO                GPIOB
#define DECAIRQ_EXTI                EXTI_Line0
#define DECAIRQ_EXTI_PORT           GPIO_PortSourceGPIOB
#define DECAIRQ_EXTI_PIN            GPIO_PinSource0
#define DECAIRQ_EXTI_IRQn           EXTI0_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE
#define DECAIRQ_EXTI_NOIRQ          DISABLE

/* ── SPI 操作宏 (不变) ── */
#define port_SPIx_busy_sending()		(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIx_no_data()				(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIx_send_data(x)			SPI_I2S_SendData((SPIx),(x))
#define port_SPIx_receive_data()		SPI_I2S_ReceiveData(SPIx)
#define port_SPIx_disable()				SPI_Cmd(SPIx,DISABLE)
#define port_SPIx_enable()              SPI_Cmd(SPIx,ENABLE)
#define port_SPIx_set_chip_select()		GPIO_SetBits(SPIx_CS_GPIO,SPIx_CS)
#define port_SPIx_clear_chip_select()	GPIO_ResetBits(SPIx_CS_GPIO,SPIx_CS)

#define port_SPIy_busy_sending()		(SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIy_no_data()				(SPI_I2S_GetFlagStatus((SPIy),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIy_send_data(x)			SPI_I2S_SendData((SPIy),(x))
#define port_SPIy_receive_data()		SPI_I2S_ReceiveData(SPIy)
#define port_SPIy_disable()				SPI_Cmd(SPIy,DISABLE)
#define port_SPIy_enable()              SPI_Cmd(SPIy,ENABLE)
#define port_SPIy_set_chip_select()		GPIO_SetBits(SPIy_CS_GPIO,SPIy_CS)
#define port_SPIy_clear_chip_select()	GPIO_ResetBits(SPIy_CS_GPIO,SPIy_CS)

/* ── WAKEUP + EXTON 说明 ──
 * Schematic2 网表中 WAKEUP→PB3, EXTON→PA15 均有物理连接,
 * 但 Trek1000 固件通过 DW1000 SPI 寄存器 (AON_CFG0, dwt_configuresleep)
 * 和 dwt_spicswakeup() 控制睡眠/唤醒, 不直接操作 WAKEUP GPIO.
 *
 * 如果需要用 GPIO 显式控制 WAKEUP:
 *   #define DW1000_WAKEUP      GPIO_Pin_3
 *   #define DW1000_WAKEUP_GPIO GPIOB
 * 并在 peripherals_init() 中配置为推挽输出.
 */

#define S1_SWITCH_ON  (1)
#define S1_SWITCH_OFF (0)
int is_switch_on(uint16_t GPIOpin);

#define port_IS_TAG_pressed()		is_switch_on(TA_SW1_4)
#define port_IS_LONGDLY_pressed()	is_dlybutton_low()

#define port_GET_stack_pointer()		__get_MSP()
#define port_GET_rtc_time()				RTC_GetCounter()
#define port_SET_rtc_time(x)			RTC_SetCounter(x)

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_AUDIBLE_enable()			// not used
#define port_AUDIBLE_disable()			// not used
#define port_AUDIBLE_set_interval_ms(x)	// not used
#define port_AUDIBLE_get_interval_ms(x)	// not used
#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);

void __weak process_dwRSTn_irq(void);
void __weak process_deca_irq(void);

void __weak button_callback(void);
int is_IRQ_enabled(void);

int is_button_low(uint16_t GPIOpin);
#define is_fastrng_on(x)  is_button_low(x)

int peripherals_init(void);
void spi_peripheral_init(void);

unsigned long portGetTickCnt(void);

#define portGetTickCount() 			portGetTickCnt()

void reset_DW1000(void);
void setup_DW1000RSTnIRQ(int enable);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
