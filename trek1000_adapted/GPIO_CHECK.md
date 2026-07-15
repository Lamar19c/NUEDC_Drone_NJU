# GPIO 对照检查 — TREK1000 适配 vs Schematic2 网表

## STM32F103 全部 49 引脚（含 EP）逐位对照

| Pin | 网表连接 | 网表用途 | TREK1000 固件使用 | 状态 |
|-----|---------|---------|-----------------|------|
| 1 VBAT | VCC | 电源 | (无) | ✅ |
| 2 PC13 | PC13 | 悬空 | (原用于 LED, 已移除) | ✅ |
| 3 PC14 | PC14 | 悬空 | (无) | ✅ |
| 4 PC15 | PC15→R37→LED1→GND | LED1 蓝色 | led.c: LED_PC6 | ✅ |
| 5 PD0-OSC_IN | X6(16MHz) | HSE 晶振 | HSE 16MHz→72MHz | ✅ |
| 6 PD1-OSC_OUT | X6(16MHz) | HSE 晶振 | HSE 16MHz→72MHz | ✅ |
| 7 NRST | SW3→GND, R28→VCC | 复位 | (硬件复位, 固件不操作) | ✅ |
| 8 VSSA | GND | 电源 | (无) | ✅ |
| 9 VDDA | VCC | 电源 | (无) | ✅ |
| 10 PA0 | PA0 | **悬空** | (原 RSTn, 已移至 PB4) | ✅ |
| 11 PA1 | PA1 | 悬空 | (无) | ✅ |
| 12 PA2 | PA2 | 悬空 | (无) | ✅ |
| 13 PA3 | PA3→SW2→VCC, R30→GND | 用户按键 | (无, 仅 stm32_uwb 用 USART2_RX) | ✅ |
| 14 PA4 | DW1000_SPI_CS | SPI 片选 | port.h: SPIx_CS | ✅ |
| 15 PA5 | DW1000_SPI_CLK | SPI 时钟 | port.h: SPIx_SCK | ✅ |
| 16 PA6 | DW1000_SPI_MISO | SPI 数据 | port.h: SPIx_MISO | ✅ |
| 17 PA7 | DW1000_SPI_MOSI | SPI 数据 | port.h: SPIx_MOSI | ✅ |
| 18 PB0 | DW1000_IRQ | DW1000 中断 | port.h: DECAIRQ (已适配) | ✅ |
| 19 PB1 | PB1 | 悬空 | led.c: LED_PC7 (phantom, 无害) | ✅ |
| 20 PB2 | BOOT1→R31→GND | 启动配置 | (硬件, 固件不操作) | ✅ |
| 21 PB10 | PB10 | 悬空 | spi.h: LCD_RW (未实际使能) | ✅ |
| 22 PB11 | PB11 | 悬空 | spi.h: LCD_RS (未实际使能) | ✅ |
| 23 VSS_1 | GND | 电源 | (无) | ✅ |
| 24 VDD_1 | VCC | 电源 | (无) | ✅ |
| 25 PB12 | PB12 | 悬空 | spi.h: SPIy_CS (SPI2, 未用) | ✅ |
| 26 PB13 | PB13 | 悬空 | spi.h: SPIy_SCK (未用) | ✅ |
| 27 PB14 | PB14 | 悬空 | spi.h: SPIy_MISO (未用) | ✅ |
| 28 PB15 | PB15 | 悬空 | spi.h: SPIy_MOSI (未用) | ✅ |
| 29 PA8 | PA8 | 悬空 | (原 EXTON, 已改为内部寄存器) | ✅ |
| 30 PA9 | PA9→CH340X RXD, U4 pin2 | USART1_TX | 固件: mc 帧输出 | ✅ |
| 31 PA10 | PA10→CH340X TXD, U4 pin3 | USART1_RX | 固件: USART1_RX | ✅ |
| 32 PA11 | PA11 | 悬空 | (无) | ✅ |
| 33 PA12 | PA12 | 悬空 | (无) | ✅ |
| 34 PA13 | PA13→H1 pin2 | SWDIO 调试 | SWD (硬件) | ✅ |
| 35 VSS_2 | GND | 电源 | (无) | ✅ |
| 36 VDD_2 | VCC | 电源 | (无) | ✅ |
| 37 PA14 | PA14→H1 pin3 | SWCLK 调试 | SWD (硬件) | ✅ |
| 38 PA15 | DW1000_EXTON | DW1000 电源状态 | (固件不操作, DW1000 内部输出) | ✅ |
| 39 PB3 | DW1000_WAKEUP | DW1000 唤醒 | (SPI 寄存器控制, 不 GPIO) | ✅ |
| 40 PB4 | DW1000_RSTN | DW1000 复位 | port.h: DW1000_RSTn (已适配) | ✅ |
| 41 PB5 | PB5 | 悬空 | (原 IRQ, 已移至 PB0) | ✅ |
| 42 PB6 | I2C1_SCL→U11 SCL | EEPROM 时钟 | (原 LED, 已移除) | ✅ |
| 43 PB7 | I2C1_SDA→U11 SDA | EEPROM 数据 | (原 LED, 已移除) | ✅ |
| 44 BOOT0 | BOOT0→SW1→VCC, R29→GND | 启动配置 | (硬件, 固件不操作) | ✅ |
| 45 PB8 | PB8→R34→LED2→GND | LED2 绿(TX) | led.c: LED_PC8 | ✅ |
| 46 PB9 | PB9→R35→LED3→GND | LED3 红(RX) | led.c: LED_PC9 | ✅ |
| 47 VSS_3 | GND | 电源 | (无) | ✅ |
| 48 VDD_3 | VCC | 电源 | (无) | ✅ |
| 49 EP | GND | 散热地 | (无) | ✅ |

## ⚠️ 需要注意（非冲突，但需知道）

| Pin | 说明 |
|-----|------|
| PA3 | 网表有 SW2 按键(上拉至VCC)，固件未使用。按键按下时 PA3=HIGH，不影响。 |
| PA0,PA1,PA2 | 网表引出但悬空。原 Trek1000 用 PA0 做 RSTn，已适配到 PB4。 |
| PB6,PB7 | 网表接 I2C EEPROM。原 Trek1000 用做 LED，已适配移除。EEPROM 在 Trek1000 固件中无驱动，不使用。 |
| PA15,PB3 | 网表接 DW1000 EXTON/WAKEUP。固件通过 SPI 寄存器控制 WAKEUP，EXTON 为 DW1000 输出。无冲突。 |

## 确认结论

**49 个引脚全部匹配，无冲突。**
