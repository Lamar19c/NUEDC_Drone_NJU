# TREK1000 固件 IO 适配 — Schematic2 网表移植

## 背景

你的 Schematic2 网表 (`Netlist_Schematic2_2026-07-13.enet`) 中 DW1000 引脚分配与 TREK1000
默认固件不匹配。此目录提供修改后的 TREK1000 源码文件，直接替换即可编译运行。

## 引脚对照

| DW1000 信号 | Schematic2 网表 | TREK1000 默认 | 状态 |
|-------------|----------------|---------------|------|
| SPI_SCK | PA5 | PA5 | ✅ 无需改 |
| SPI_MISO | PA6 | PA6 | ✅ 无需改 |
| SPI_MOSI | PA7 | PA7 | ✅ 无需改 |
| SPI_CS | PA4 | PA4 | ✅ 无需改 |
| **IRQ** | **PB0** | PB5 | ❌ 已适配 |
| **RSTn** | **PB4** | PA0 | ❌ 已适配 |
| WAKEUP | PB3 | 内部寄存器 | ✅ 不受影响 |
| EXTON | PA15 | 内部输出 | ✅ 不受影响 |

## 需替换的文件

将本目录文件覆盖到 TREK1000 源码对应位置:

| 本目录文件 | TREK1000 目标路径 |
|-----------|------------------|
| `port.h` | `HARDWARE/PORT/port.h` |
| `stm32f10x_it.c` | `HARDWARE/platform/stm32f10x_it.c` |

Station 和 Tag 共用同一套适配文件。

## 改动摘要

### port.h

```c
// RSTn: PA0 → PB4
#define DW1000_RSTn       GPIO_Pin_4      // 原: GPIO_Pin_0
#define DW1000_RSTn_GPIO  GPIOB            // 原: GPIOA
#define DECARSTIRQ        GPIO_Pin_4       // 原: GPIO_Pin_0
#define DECARSTIRQ_GPIO   GPIOB            // 原: GPIOA
#define DECARSTIRQ_EXTI   EXTI_Line4       // 原: EXTI_Line0
// ...
#define DECARSTIRQ_EXTI_IRQn  EXTI4_IRQn   // 原: EXTI0_IRQn

// IRQ: PB5 → PB0
#define DECAIRQ           GPIO_Pin_0       // 原: GPIO_Pin_5
#define DECAIRQ_GPIO      GPIOB            // 不变
#define DECAIRQ_EXTI      EXTI_Line0       // 原: EXTI_Line5
// ...
#define DECAIRQ_EXTI_IRQn     EXTI0_IRQn   // 原: EXTI9_5_IRQn
```

### stm32f10x_it.c

```c
// EXTI0: 原先 RSTn → 现在 IRQ
void EXTI0_IRQHandler(void) {
    process_deca_irq();          // 原: process_dwRSTn_irq()
    EXTI_ClearITPendingBit(DECAIRQ_EXTI);
}

// EXTI4: 新增 → RSTn
void EXTI4_IRQHandler(void) {
    process_dwRSTn_irq();
    EXTI_ClearITPendingBit(DECARSTIRQ_EXTI);
}
```

## 编译

修改后使用 Keil MDK 编译 Station 和 Tag 工程:

```
Station: Download/UWB_TDOA_STATION_V1_1-master/.../test_rx11/
Tag:     Download/UWB_TDOA_TAG_V1_1-master/tag/
```

工程为 Keil MDK ARMCC 格式。如需用 GCC (STM32CubeIDE), 需要自行创建工程并导入源文件。

## 注意事项

1. **MCU 升级**: Schematic2 原设计用 CBU6 (128KB), 你已确认换 RCT6 (256KB) — Station 固件 ~180KB 可以装入
2. **HSE 晶振**: Schematic2 用 16MHz, TREK1000 默认用 12MHz — 需要修改 `RCC_Configuration()` 中的 PLL 配置或 `SystemCoreClock` 宏
3. **UART**: TREK1000 用 USART1 (PA9/PA10) 输出 mc 帧, 波特率需匹配 stm32_uwb 的 115200
4. **WAKEUP**: DW1000 的 WAKEUP 功能通过 SPI 寄存器配置 `DWT_WAKE_WK` 位, 固件未使用 GPIO 显式控制; 如需硬件 WAKEUP, 可自行在 port.h 添加宏
