# EasyEDA 原理图手工绘制指南 — 方案 B: DWM1000 + RCT6

## 准备工作

1. EasyEDA Pro → 新建工程 `UWB_TRANSCEIVER`
2. 新建原理图 → 命名 `UWB_MAIN`

---

## Step 1：放置核心元件（EasyEDA 库中搜索）


| 位号 | EasyEDA 搜索关键词 | 坐标 |
|------|-------------------|------|
| U5 | `STM32F103RCT6` | (4000, 4000) |
| U1 | `DWM1000` 或 `BU01` | (9000, 4000) |
| U3 | `TPS73601DBVR` | (2000, 2500) |
| CN1 | `MICRO USB B 5PIN` | (1000, 1000) |
| X2 | `12MHz 3225` | (4000, 2000) |
| F1 | `PTC 0805 0.5A` | (1500, 1500) |
| P1 | `排针 2x4` | (1000, 5000) |


如搜不到精确型号，用同类替代（如 TPS73601 → 任意 SOT-23-5 LDO、DWM1000 → 24pin 模块符号）。

---

## Step 2：放置无源元件


每个元件在 EasyEDA 库搜索 "*值* *封装*"，例如：

| 位号 | 搜索 | 数量 |
|------|------|------|
| C25 | `10uF TAN A` | 1 |
| C26 | `1uF 0402` | 1 |
| C2 | `10uF 0805` | 1 |
| C32,C35 | `4.7uF 0603` | 2 |
| C19,C20,C34,C31 | `0.1uF 0402` | 4 |
| C3,C10 | `10pF 0402` | 2 |
| R10,R13,R11 | `10k 0402` | 3 |
| R_LED1~4 | `270 0402` | 4 |
| D1~4 | `LED RED 0603` | 4 |
| C11 | `0.1uF 0402` | 1 (MCU去耦) |


---

## Step 3：连线（按网络逐一画 WIRE）

### 3.1 供电网络

```
CN1.VBUS ──→ F1.1
F1.2 ──→ U3.IN (VIN)
U3.OUT ──→ VDD3V3 (全板 3.3V 总线)

VDD3V3 ──→ U5: VDD_1, VDD_2, VDD_3, VDD_4, VDDA
VDD3V3 ──→ U1: VCC (pin 1,2,23,24)
VDD3V3 ──→ LED 阳极 (D1~D4 通过 270Ω)

GND ──→ U3.GND, U5.VSS×3+VSSA, U1.GND(pin 3,4,21,22)
GND ──→ 所有电容负极, 所有 LED 阴极(通过270Ω)
```


### 3.2 C25, C26 — LDO 周边

```
U3.IN  ── C25(+) ── GND        C25: 10µF/16V 钽
U3.OUT ── C26(+) ── GND        C26: 1µF
```

### 3.3 DWM1000 去耦电容

```
U1.VCC(pin1) ── C2(+) ── GND   C2: 10µF
U1.VCC(pin2) ── C19(+) ── GND  C19: 0.1µF
U1.VCC(pin23) ── C20(+) ── GND C20: 0.1µF
U1.VCC(pin24) ── C34(+) ── GND C34: 0.1µF
C31: 0.1µF, C32/C35: 4.7µF 就近接 U1 VCC
```

### 3.4 SPI 总线（核心）

```
U5.PA5(SPI1_SCK)  ──→ U1.SCK  (pin 5)   [网络: DW_SCK]
U5.PA6(SPI1_MISO) ──→ U1.MISO (pin 6)   [网络: DW_MISO]
U5.PA7(SPI1_MOSI) ──→ U1.MOSI (pin 7)   [网络: DW_MOSI]
U5.PA4(GPIO)      ──→ U1.CS   (pin 8)   [网络: DW_NSS]
U5.PC5(GPIO)      ──→ U1.RSTn (pin 11)  [网络: DW_RSTn]
U5.PB0(EXTI)      ←── U1.IRQ  (pin 9)   [网络: DW_IRQn] ← 10kΩ上拉
```

### 3.5 晶振

```
X2.1 ──→ U5.PD0(OSC_IN)
X2.1 ── C3 ── GND           C3: 10pF
X2.2 ──→ U5.PD1(OSC_OUT)
X2.2 ── C10 ── GND          C10: 10pF
```

### 3.6 LED 指示灯

```
U5.PC13 ── R_LED1(270Ω) ── D1(K) ── D1(A) ── VDD3V3
U5.PC14 ── R_LED2(270Ω) ── D2(K) ── D2(A) ── VDD3V3
U5.PC15 ── R_LED3(270Ω) ── D3(K) ── D3(A) ── VDD3V3
U5.PA0  ── R_LED4(270Ω) ── D4(K) ── D4(A) ── VDD3V3
```

> 注: STM32 GPIO 低电平时 LED 亮。

### 3.7 上拉电阻

```
U5.NRST ── R10(10kΩ) ── VDD3V3
U5.BOOT0 ── R13(10kΩ) ── VDD3V3   (或 GND, 看启动模式)
DW_IRQn ── R11(10kΩ) ── VDD3V3
```

### 3.8 SWD 调试口

```
P1.1(VCC)  ── VDD3V3
P1.2(SWDIO) ── U5.PA13
P1.3(SWCLK) ── U5.PA14
P1.4(GND)  ── GND
P1.5(RESET) ── U5.NRST
```

### 3.9 UART 输出（Tag板 → stm32_uwb板）

```
U5.PA9(USART1_TX) ──→ J1.1 (排针, 引出到外部)
J1.2 ── GND
```

### 3.10 MCU 去耦

```
U5.VDD_1 ── C11(0.1µF) ── GND
U5.VDD_2 ── C11b(0.1µF) ── GND
U5.VDD_3 ── C11c(0.1µF) ── GND
U5.VDDA  ── C11d(0.1µF) + 1µF ── GND
```

---

## Step 4：添加网络标签

| 网络名 | 位置 | 说明 |
|--------|------|------|
| VCC5V_USB | CN1.VBUS 附近 | 5V 输入 |
| VDD3V3 | U3.OUT 附近 | 3.3V 主电源 |
| GND | 多处 | 地 |
| DW_SCK | SPI 线上 | PA5 → U1.SCK |
| DW_MISO | SPI 线上 | PA6 → U1.MISO |
| DW_MOSI | SPI 线上 | PA7 → U1.MOSI |
| DW_NSS | SPI 线上 | PA4 → U1.CS |
| DW_RSTn | 控制线 | PC5 → U1.RSTn |
| DW_IRQn | 中断线 | PB0 ← U1.IRQ |
| NRST | 复位 | U5.NRST |
| SWDIO, SWCLK | SWD 口 | P1 附近 |
| UART1_TX | UART | PA9 → J1 |

---

## Step 5：元件清单核对

完成绘制后，对照此表逐项确认：

- [ ] U5 STM32F103RCT6 LQFP64 — 位号正确
- [ ] U1 DWM1000/BU01 模块 — 24pin, SPI 连接正确
- [ ] U3 TPS73601DBVR — 5V→3.3V LDO, R1/R2 反馈电阻
- [ ] CN1 Micro USB — VBUS/GND/DM/DP
- [ ] X2 12MHz — 配 C3/C10 各 10pF
- [ ] F1 0.5A PTC — VCC5V_USB 串联
- [ ] P1 SWD 2x4 — SWDIO/SWCLK/NRST/VCC/GND
- [ ] C25 10µF/16V TAN — U3.IN 去耦
- [ ] C26 1µF — U3.OUT 去耦
- [ ] 去耦电容 ×8 — 各 VDD 引脚
- [ ] R10/R13 10kΩ — NRST/BOOT0 上拉
- [ ] R11 10kΩ — DW_IRQn 上拉
- [ ] D1~4 + R_LED1~4 — 状态指示
- [ ] J1 UART 排针 — PA9 TX 引出
- [ ] 所有 GND 连通
- [ ] 所有 VDD3V3 连通

完成。
