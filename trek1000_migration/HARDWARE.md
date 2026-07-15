# TREK1000 迁移 — 硬件连接方案

## 方案总览

| 方案 | UWB 方案 | RF 复杂度 | 推荐场景 |
|------|---------|----------|---------|
| A | 裸 DW1000 + HHM1595A1 Balun | 高 (自设计RF) | 追求最远距离 |
| B | DWM1000/BU01 模块 | 无 (模块内含) | **快速验证, 比赛首选** |
| C | DWM3000 模块 | 无 (模块内含) | 长远布局 |

---

## 方案 A：裸 DW1000 + RCT6（基于 dwm.pdf 原理图）

### MCU 引脚映射

```
STM32F103RCT6 (LQFP64) ←→ DW1000 (QFN48) + 外设

【SPI 通信】
PA5(SPI1_SCK)  ←→ DW1000 SPICLK  (Pin 39)
PA6(SPI1_MISO) ←→ DW1000 SPIMISO (Pin 40)
PA7(SPI1_MOSI) ←→ DW1000 SPIMOSI (Pin 41)
PA4(GPIO)      ──→ DW1000 SPICSn  (Pin 38)  [片选, 低有效]

【控制信号】
PB0(EXTI)      ←── DW1000 IRQ     (Pin 45)  [中断, 10kΩ上拉]
PB1(GPIO)      ──→ DW1000 WAKEUP  (Pin 37)  [唤醒, 高有效]
PC5(GPIO)      ──→ DW1000 RSTn    (Pin 36)  [复位, 低有效]
PA8(GPIO)      ←── DW1000 EXTON   (Pin 35)  [电源状态, 控制1.8V使能]

【UART 输出 (Tag 板 → stm32_uwb 板)】
PA9(USART1_TX) ──→ stm32_uwb PA3(USART2_RX)  [mc 数据, 115200]
PA10(USART1_RX)── (悬空, 保留)

【UART 输出 (Anchor A0 → 地面站)】
PA9(USART1_TX) ──→ USB-TTL → PC  [调试用]

【SWD 调试】
PA13(SWDIO)    ←── ST-Link SWDIO
PA14(SWCLK)    ←── ST-Link SWCLK
NRST           ←── ST-Link NRST (10kΩ上拉)

【供电】
3.3V ← TPS73601DBVR ← 5V USB / 充电宝

【晶振】
PD0(OSC_IN)  ←→ 12MHz XTAL ←→ PD1(OSC_OUT)  (各22pF到GND)
```

### DW1000 射频链路

```
DW1000 RF_P/RF_N (差分100Ω, Pin 1/2)
    → HHM1595A1 Balun (RF_P/RF_N → BPORT → RF_OUT)
    → 0402 匹配网络: 1.2pF, 10pF, 12pF, 18pF, 27pF
    → 滤波: 330pF, 820pF
    → LXDC2HL_18A (RF 滤波器)
    → 0Ω 跳线 → SMA 母座 → 外置天线
```

### DW1000 电源去耦

| 引脚 | 网络 | 去耦电容 |
|------|------|---------|
| VDDBAT(45,46) | VDDBAT | 10µF + 0.1µF |
| VDDDIG | VDDDIG | 0.1µF |
| VDDMS | VDDMS | 10000pF |
| VDDIF | VDDIF | 10pF |
| VDDCLK | VDDCLK | 330pF |
| VDDSYN | VDDSYN | 0.1µF |
| VDDVCO | VDDVCO | 0.1µF |
| VDDLNA | VDDLNA | 330pF |
| VDDPA | VDDPA | 47µF |
| VDD1V8 | VDD1V8 | 4.7µF ×2 |

### 晶振电路

```
DW1000 38.4MHz:
  XTAL1 ← X1.1 → XTAL2  (配 270Ω 电阻到 VDDCLK)
  CLKTUNE: C14 1.2pF 到 GND
  VCOTUNE: R12 16kΩ 到 GND + C15 820pF/C16 27pF/C17 18pF 到 GND

STM32 12MHz:
  PD0(OSC_IN) ← X2.1 → X2.2 → PD1(OSC_OUT)
  C3 10pF (OSC_IN→GND), C10 10pF (OSC_OUT→GND)
```

---

## 方案 B：DWM1000/BU01 模块 + RCT6

### 与方案 A 的关键差异

| | 方案 A | 方案 B |
|---|---|---|
| UWB 芯片 | DW1000 QFN48 裸片 | DWM1000/BU01 模块 (23×13mm) |
| RF 链路 | 自设计 Balun+匹配+天线 | 模块内含, 免设计 |
| 晶振 | 外接 38.4MHz | 模块内含 |
| 天线 | SMA 外置 | 模块 PCB 天线（或 U.FL 外接） |
| 焊接 | QFN48 难焊 | 邮票孔/排针 易焊 |
| PCB 层数 | 建议 4 层 | 2 层即可 |

### DWM1000 模块引脚 (24-pin 邮票孔)

```
【电源】
Pin 1,2,23,24: VCC (3.3V)
Pin 3,4,21,22: GND

【SPI】
Pin 5(SCK)   ← PA5(SPI1_SCK)
Pin 6(MISO)  → PA6(SPI1_MISO)
Pin 7(MOSI)  ← PA7(SPI1_MOSI)
Pin 8(CS)    ← PA4(GPIO)

【控制】
Pin 9(IRQ)   → PB0(EXTI), 10kΩ上拉
Pin 10(WAKEUP) ← PB1(GPIO) (可悬空)
Pin 11(RSTn) ← PC5(GPIO)
Pin 12(EXTON) → PA8(GPIO) (可悬空, 模块内含电源管理)

【保留/扩展 (可悬空)】
Pin 13~20: GPIO0~7, SYNC, 保留
```

### 精简原理图

```
PCB 仅需放置:
  STM32F103RCT6 (LQFP64)
  + DWM1000/BU01 模块 (24-pin)
  + TPS73601DBVR (5V→3.3V LDO)
  + 12MHz XTAL (STM32 HSE)
  + 阻容 (3.3V去耦 ×10, 上拉 ×4, LED ×4)
  + SWD 4-pin + UART 引出 + Micro USB

删除 (模块内含):
  ✗ DW1000 QFN48 裸片
  ✗ HHM1595A1 Balun
  ✗ 38.4MHz XTAL
  ✗ 0402 RF 匹配电容
  ✗ LXDC2HL_18A
  ✗ SMA 母座
```

---

## 方案 C：DWM3000 模块 + RCT6

引脚与 DWM1000 完全兼容。固件不兼容 TREK1000，需要 Qorvo DW3000 SDK 重新适配。

---

## 供电方案 (三版共用)

```
Micro USB 5V
    │
    ├── F1 (0.5A 保险丝)
    │
    ▼
VCC5V_IN → TPS73601DBVR LDO
    │         R1=52.3kΩ (OUT→FB)
    │         R2=30.1kΩ (FB→GND)
    │         VOUT ≈ 3.3V
    │
    ├── C25 10µF/16V 钽 (IN→GND)
    ├── C26 1µF (OUT→GND)
    │
    ▼
VDD3V3 (全板供电)
    ├── STM32 VDD ×4 + VDDA (各 0.1µF 去耦)
    ├── DWM1000/DW1000 供电
    ├── LED ×4 (阳极接 3.3V, 阴极各串 270Ω → GPIO)
    └── 上拉电阻: NRST(10k), BOOT0(10k), IRQ(10k)
```

---

## 无人机上连接

```
Tag 板 (新PCB, RCT6)              stm32_uwb 板 (已有, C8T6)
┌──────────────────────┐        ┌───────────────────────────┐
│ PA9 (USART1_TX) ─────┼─杜邦线─┼── PA3 (USART2_RX)         │
│ GND              ────┼─杜邦线─┼── GND                      │
│                      │        │                           │
│ 5V ← 无人机电源       │        │ PB10 (USART3_TX) ──→ FC  │
│ DWM1000 ←→ 天线(内含) │       │ PA9 (USART1_TX) → debug   │
└──────────────────────┘        └───────────────────────────┘
```

## 地面 Anchor 连接

```
每个 Anchor (新PCB, RCT6)
┌──────────────────────────────┐
│ Micro USB ←── 充电宝 (5V)     │
│ DWM1000 模块 ←→ 天线(内含)    │
│                              │
│ A0: PA9(USART1_TX) → USB-TTL │  调试用
│ A1/A2/A3: UART 悬空          │
└──────────────────────────────┘

Anchor 之间：无任何连线。全部通过 UWB 无线电通信。
```
