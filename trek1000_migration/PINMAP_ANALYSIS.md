# 接线网表 vs TREK1000 引脚对比分析

> 基于 `Netlist_Schematic2_2026-07-13.enet` 实际接线数据
> 对比目标: TREK1000 固件默认引脚分配

---

## 一、Schematic2 实际 DW1000 → STM32 接线

从网表提取的 U1(STM32F103CBU6) ←→ U2(DW1000) 连接:

| DW1000 信号 | DW1000 引脚 | 网络名 | STM32 引脚 | STM32 功能 |
|-------------|------------|--------|-----------|-----------|
| SPICSn | pin24 | `DW1000_SPI_CS` | **PA4** (pin14) | GPIO |
| SPICLK | pin41 | `DW1000_SPI_CLK` | **PA5** (pin15) | SPI1_SCK |
| SPIMISO | pin40 | `DW1000_SPI_MISO` | **PA6** (pin16) | SPI1_MISO |
| SPIMOSI | pin39 | `DW1000_SPI_MOSI` | **PA7** (pin17) | SPI1_MOSI |
| IRQ/GPIO8 | pin45 | `DW1000_IRQ` | **PB0** (pin18) | EXTI |
| EXTON | pin21 | `DW1000_EXTON` | **PA15** (pin38) | GPIO |
| WAKEUP | pin23 | `DW1000_WAKEUP` | **PB3** (pin39) | GPIO |
| RSTn | pin27 | `DW1000_RSTN` | **PB4** (pin40) | GPIO |

---

## 二、Schematic2 vs TREK1000 默认引脚对比

| 信号 | Schematic2 实际 | TREK1000 默认 | 是否匹配 |
|------|----------------|---------------|---------|
| SPI_SCK | PA5 | PA5 | ✅ |
| SPI_MISO | PA6 | PA6 | ✅ |
| SPI_MOSI | PA7 | PA7 | ✅ |
| SPI_CS | PA4 | PA4 | ✅ |
| IRQ | PB0 | PB0 | ✅ |
| **WAKEUP** | **PB3** | **PB1** | ❌ **不匹配** |
| **RSTn** | **PB4** | **PC5** | ❌ **不匹配** |
| **EXTON** | **PA15** | **PA8** | ❌ **不匹配** |

> **结论**: 5/8 引脚匹配，3/8 不匹配。TREK1000 固件不能直接烧录到此 PCB。

---

## 三、MCU 差异

| 属性 | Schematic2 实际 | TREK1000 要求 |
|------|----------------|---------------|
| 型号 | STM32F103**CBU6** | STM32F103**RCT6** |
| 封装 | UFQFPN-48 (7×7mm) | LQFP64 |
| Flash | 128KB | 256KB |
| RAM | 20KB | 48KB |
| GPIO | 37 | 51 |

> **Flash 影响**: TREK1000 Station 固件 ~180KB，CBU6 的 128KB 装不下。Tag 固件 ~80KB，勉强可行。

---

## 四、移植方案

### 方案 1: 修改 TREK1000 固件引脚定义（推荐）

在 TREK1000 源码中修改 GPIO 初始化，适配 Schematic2 实际接线:

```c
// 需要修改的宏/定义 (以 TREK1000 Station 为例)
// 原值 → 新值

#define DW_RST_PIN    PC5   →  PB4    // RSTn
#define DW_WAKE_PIN   PB1   →  PB3    // WAKEUP  
#define DW_EXTON_PIN  PA8   →  PA15   // EXTON
```

**影响范围**:
- SPI + IRQ: 无需改动 ✅
- RSTn: 改 GPIO 初始化 + 操作宏
- WAKEUP: 改 GPIO 初始化 + 操作宏
- EXTON: 改 GPIO 初始化 + 操作宏

**风险**: 低。只是 GPIO 重映射，不影响时序逻辑。

### 方案 2: 重新设计 PCB 匹配 TREK1000 默认引脚

改 PCB 走线将 WAKEUP→PB1, RSTn→PC5, EXTON→PA8。同时升级 MCU 为 RCT6。

**风险**: 需要重新打样，周期 2-3 周。

### 方案 3: 仅修改 stm32_uwb (不改 Tag 板)

如果 Tag 板使用其他固件（如 Arduino DW1000 库）输出 mc 协议:
- stm32_uwb 的 mc 解析器无需改动
- Tag 板固件自行处理引脚差异

---

## 五、stm32_uwb 板 (C8T6) 影响分析

Schematic2 是 **Tag 板**的设计，stm32_uwb 是**接收板**，二者通过 UART 连接:

```
Tag 板 (Schematic2)           stm32_uwb (已有 C8T6)
PA9 (USART1_TX)  ──杜邦线──→  PA3 (USART2_RX)
GND              ─────────→  GND
```

| stm32_uwb 改动项 | 状态 |
|------------------|------|
| uwb_solver.c 加 mc 解析 | ✅ 已含 |
| USART2 baud = 115200 | ✅ usart.c 已改 |
| .ioc USART2 baud | ⚠️ 仍为 19200 |
| DIST_TIMEOUT_MS | ⚠️ 150ms (建议 400ms) |
| 缓冲区 64 字节 | ⚠️ mc 帧 61 字节，余量仅 2 字节 |

stm32_uwb 的改动不受 Tag 板引脚影响 — 它只收 UART 数据。
