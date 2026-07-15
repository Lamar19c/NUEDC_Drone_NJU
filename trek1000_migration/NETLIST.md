# DWM1000模块 + STM32F103RCT6 开发板 结构化连线网表
采用EDA标准「网络名-连通节点」格式，可直接用于引脚映射、驱动开发、电路仿真解析。

---

## 一、有源器件总清单

| 位号 | 完整型号 | 器件类型 | 核心功能 |
|------|----------|----------|----------|
| U1 | DWM1000 (BU01) | UWB收发模块 | UWB测距/通信，内置DW1000+晶振+巴伦+天线 |
| U5 | STM32F103RCT6 | 32位主控MCU | 系统控制、SPI通信、外设管理，LQFP64 |
| U3 | TPS73601DBVR | LDO线性稳压芯片 | 5V转3.3V系统主电源，SOT-23-5 |
| CN1 | Micro-B USB | 输入接口 | 5V供电+USB 2.0全速数据通信 |
| X2 | 12MHz无源晶振 | 时钟源 | U5的主系统时钟，3225封装 |
| P1 | 4×2直插排针 | 调试接口 | SWD程序下载与在线调试 |
| F1 | 0.5A保险丝 | 保护器件 | USB输入过流保护，0805 PTC |
| J1 | 1×4直插排针 | 输出接口 | UART输出 (Tag→stm32_uwb) |
| D1 D2 D3 D4 | 红色LED | 状态指示 | 低电平点亮，0603封装 |

> 注：DWM1000模块内含 DW1000芯片 + 38.4MHz晶振 + RF巴伦 + 匹配网络 + PCB天线，以下RF相关引脚为模块内部连接，PCB设计时无需处理。

---

## 二、无源器件连接规则

### 1. 串联型

| 位号 | 参数 | 端1连接网络 | 端2连接网络 |
|------|------|------------|------------|
| F1 | 0.5A PTC | VCC5V_USB | VCC5V_IN |
| R11 | 10kΩ | VDD3V3 | DW_IRQn |
| R1_LED | 270Ω | U5.PC13 | D1_K |
| R2_LED | 270Ω | U5.PC14 | D2_K |
| R3_LED | 270Ω | U5.PC15 | D3_K |
| R4_LED | 270Ω | U5.PA0 | D4_K |

### 2. 并联型（上端接目标网络，下端接GND）

| 位号 | 参数 | 上端连接网络 |
|------|------|------------|
| C25 | 10µF/16V TAN | VCC5V_IN (U3.IN端) |
| C26 | 1µF 0402 | VDD3V3 (U3.OUT端) |
| C2 | 10µF 0805 | VDD3V3 (U1.VCC去耦) |
| C19 | 0.1µF 0402 | VDD3V3 (U1.VCC去耦) |
| C20 | 0.1µF 0402 | VDD3V3 (U1.VCC去耦) |
| C34 | 0.1µF 0402 | VDD3V3 (U1.VCC去耦) |
| C31 | 0.1µF 0402 | VDD3V3 (U1.VCC去耦) |
| C32 | 4.7µF 0603 | VDD3V3 (U1.VCC去耦) |
| C35 | 4.7µF 0603 | VDD3V3 (U1.VCC去耦) |
| C3 | 10pF 0402 | OSC_IN |
| C10 | 10pF 0402 | OSC_OUT |
| C11 | 0.1µF 0402 | VDD3V3 (U5.VDD去耦) |
| C12 | 0.1µF 0402 | VDD3V3 (U5.VDD去耦) |
| C13 | 0.1µF 0402 | VDD3V3 (U5.VDD去耦) |
| C14 | 1µF+0.1µF | VDD3V3 (U5.VDDA去耦) |
| R10 | 10kΩ 0402 | NRST (上拉至VDD3V3) |
| R13 | 10kΩ 0402 | BOOT0 (下拉至GND) |

---

## 三、全局网络连接总表

### 1. 全局地网络 GND
直接连通：U1.GND×4, U3.GND, U5.VSS×3+VSSA, CN1.GND, P1.GND, J1.GND, 所有去耦电容下端, R13.GND端, 所有LED阳极(通过VDD3V3)

---

### 2. 电源域网络

#### 2.1 5V输入域
- **VCC5V_USB**: CN1.VBUS → F1.1
- **VCC5V_IN**: F1.2 → U3.IN

#### 2.2 3.3V主电源域（全局电源总线）
- **VDD3V3**: U3.OUT → U5.VDD×4(1,24,36,48脚) → U5.VDDA(9脚) → U1.VCC×4(pin1,2,23,24) → R10上端 → P1.3V3 → C25/C26/C2/C19/C20/C34/C31/C32/C35/C11/C12/C13/C14上端 → D1~D4阳极(经270Ω)

#### 2.3 LDO反馈网络
- **VDD3V3_A**: U3.OUT → U3.FB (通过R1=52.3kΩ)
- U3.FB → GND (通过R2=30.1kΩ)
- VOUT = 1.204 × (1 + 52.3/30.1) ≈ 3.3V

---

### 3. 核心互联网络（STM32 ↔ DWM1000）

| 网络名 | U5(STM32F103RCT6) | U1(DWM1000) | 信号方向 | 功能 |
|--------|-------------------|-------------|----------|------|
| DW_SCK | PA5 (SPI1_SCK) | SCK (pin5) | MCU→DWM | SPI时钟 |
| DW_MISO | PA6 (SPI1_MISO) | MISO (pin6) | DWM→MCU | SPI数据输入 |
| DW_MOSI | PA7 (SPI1_MOSI) | MOSI (pin7) | MCU→DWM | SPI数据输出 |
| DW_NSS | PA4 (GPIO) | CS (pin8) | MCU→DWM | SPI片选,低有效 |
| DW_RSTn | PC5 (GPIO) | RSTn (pin11) | MCU→DWM | 复位,低有效 |
| DW_IRQn | PB0 (EXTI) | IRQ (pin9) | DWM→MCU | 中断,10kΩ上拉 |
| DW_WUP | PB1 (GPIO) | WAKEUP (pin10) | MCU→DWM | 唤醒(可悬空) |

---

### 4. 时钟电路网络

- **OSC_IN**: X2.1 → U5.PD0(OSC_IN) → C3(10pF)→GND
- **OSC_OUT**: X2.2 → U5.PD1(OSC_OUT) → C10(10pF)→GND

---

### 5. MCU外设接口网络

#### 5.1 USB 2.0全速接口
- USB_DM: CN1.DM → U5.PA11
- USB_DP: CN1.DP → U5.PA12

#### 5.2 SWD调试接口
- SWDIO: P1.2 → U5.PA13(JTMS/SWDIO)
- SWCLK: P1.3 → U5.PA14(JTCK/SWCLK)
- NRST: P1.5 → U5.NRST(7脚) → R10(10kΩ)→VDD3V3
- P1.1: VDD3V3
- P1.4,P1.6,P1.7,P1.8: GND

#### 5.3 UART输出（Tag→stm32_uwb）
- UART1_TX: U5.PA9(USART1_TX) → J1.1
- J1.2: GND

#### 5.4 启动配置
- BOOT0: U5.BOOT0(44脚) → R13(10kΩ)→GND
- BOOT1: U5.PB2 → (悬空或GND)

#### 5.5 状态指示LED
- D1_A → VDD3V3 | D1_K → R1_LED(270Ω) → U5.PC13
- D2_A → VDD3V3 | D2_K → R2_LED(270Ω) → U5.PC14
- D3_A → VDD3V3 | D3_K → R3_LED(270Ω) → U5.PC15
- D4_A → VDD3V3 | D4_K → R4_LED(270Ω) → U5.PA0

#### 5.6 U1保留引脚
- U1.EXTON (pin12): 悬空 (模块内含电源管理)
- U1.GPIO0~7 (pin13~20): 悬空 (预留扩展)
- U1.SYNC (pin22): 悬空 (多设备同步,预留)

---

## 四、STM32F103RCT6 完整引脚分配

| 引脚 | 网络 | 方向 | 功能 |
|------|------|------|------|
| 1(VBAT) | VDD3V3 | PWR | 备份电源 |
| 5(OSC_IN) | OSC_IN | IN | 12MHz晶振 |
| 6(OSC_OUT) | OSC_OUT | OUT | 12MHz晶振 |
| 7(NRST) | NRST | IN | 复位 |
| 9(VDDA) | VDD3V3 | PWR | 模拟电源 |
| 13(SWDIO) | SWDIO | I/O | 调试 |
| 14(SWCLK) | SWCLK | IN | 调试 |
| 20(PA4) | DW_NSS | OUT | DW CS |
| 21(PA5) | DW_SCK | OUT | DW SPI CLK |
| 22(PA6) | DW_MISO | IN | DW SPI MISO |
| 23(PA7) | DW_MOSI | OUT | DW SPI MOSI |
| 24(VDD_1) | VDD3V3 | PWR | 数字电源 |
| 25(PA0) | LED4 | OUT | LED4 |
| 28(PB1) | DW_WUP | OUT | DW WAKEUP |
| 29(PB0) | DW_IRQn | IN | DW中断 |
| 34(PA9) | UART1_TX | OUT | Tag→stm32_uwb |
| 35(PA10) | NC | — | 悬空 |
| 36(VDD_2) | VDD3V3 | PWR | 数字电源 |
| 40(PC5) | DW_RSTn | OUT | DW复位 |
| 42(PA11) | USB_DM | I/O | USB |
| 43(PA12) | USB_DP | I/O | USB |
| 44(BOOT0) | BOOT0 | IN | 启动选择(GND) |
| 48(VDD_3) | VDD3V3 | PWR | 数字电源 |
| 53(PC13) | LED1 | OUT | LED1 |
| 54(PC14) | LED2 | OUT | LED2 |
| 55(PC15) | LED3 | OUT | LED3 |

---

## 五、与方案A的差异对照

| 项目 | 方案A (裸DW1000+T8U6) | 方案B (DWM1000+RCT6) |
|------|----------------------|---------------------|
| UWB | DW1000 QFN48 裸片 | DWM1000 模块 24-pin |
| RF | HHM1595A1+匹配电容+SMA | 模块内含,无需外接 |
| 晶振 | 38.4MHz(X1)+12MHz(X2) | 仅12MHz(X2), 38.4内置于模块 |
| 1.8V电源 | LXDC2HL_18A DC-DC | 模块内含,无需外接 |
| MCU | STM32F103T8U6 QFN36 | STM32F103RCT6 LQFP64 |
| FLASH | 64KB | 256KB |
| GPIO | 26 | 51 |
| PCB层数 | 建议4层 | 2层即可 |
| 元件总数 | ~65 | ~40 |
