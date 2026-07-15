# JZM01 → TREK1000 迁移设计文档

**日期**: 2026-07-13
**状态**: 设计完成，待实施
**目标**: 用 TREK1000 开源方案替换 JZM01 黑盒 UWB 模块

---

## 一、系统拓扑

### 1.1 总体架构

```
地面 Anchor ×4 (各自充电宝供电，之间无连线)
  Anchor S1(A0)  Anchor S2(A1)  Anchor S3(A2)  Anchor S4(A3)
  统一PCB+Station固件  统一PCB+Station固件  统一PCB+Station固件  统一PCB+Station固件
        │              │              │              │
        └──────────────┼──────────────┼──────────────┘
                       │  UWB 无线电 DS-TWR
                       │  (CH5, 110K, 1024 preamble)
                       │
              ┌────────▼──────────┐
              │     无人机         │
              │                   │
              │ Tag 板 (统一PCB)    │  UART     stm32_uwb 板 (已有)
              │ TREK1000 Tag固件   │──mc数据──  C8T6, CubeMX HAL
              │                   │  115200   mc→距离→三边→NMEA
              │                   │              │ USART3 DMA
              │                   │              ▼ FC GPS 口
              └───────────────────┘
```

### 1.2 角色分配

| 板子 | 位置 | 固件 | 功能 |
|------|------|------|------|
| Tag ×1 | 无人机 | TREK1000 Tag | UWB TWR测距, UART输出mc |
| Anchor S1(A0) | 地面原点 | TREK1000 Station | Gateway, 收齐4锚点距离, UART输出mc |
| Anchor S2(A1) | 地面X轴 | TREK1000 Station | 回应Tag Poll, A2A TWR |
| Anchor S3(A2) | 地面Y轴 | TREK1000 Station | 回应Tag Poll, A2A TWR |
| Anchor S4(A3) | 地面对角 | TREK1000 Station | Listener, 仅收不发 |
| stm32_uwb | 无人机 | 现有固件+mc补丁 | 解析mc → 三边定位 → NMEA → FC |

### 1.3 连接关系

| 连接 | 介质 | 说明 |
|------|------|------|
| Tag DW1000 ↔ Anchor DW1000 | UWB无线电 | DS-TWR双向测距 |
| Anchor A0 ↔ Anchor A1/A2 | UWB无线电 | A2A TWR时钟同步 |
| Tag板 UART → stm32_uwb板 UART | 杜邦线(TX+GND) | mc测距数据, 115200 |
| stm32_uwb UART → FC | 杜邦线(TX+GND) | NMEA GPS语句, 57600 |
| 地面Anchor供电 | 充电宝 USB | 5V, 各自独立 |

---

## 二、硬件设计

### 2.1 统一 PCB 方案

Tag 和 4 个 Anchor 使用**完全相同的 PCB**，一次打样 5 片。角色由烧录的固件区分。

提供**三版硬件方案**，按 RF 难度和灵活性递减排列：

---

### 方案 A：裸 DW1000 + 自设计 RF（已有原理图）

基于现有 `dwm.pdf` 原理图，自行设计 DW1000 射频匹配网络和天线。

#### 核心芯片

| 元件 | 型号 | 封装 | 说明 |
|------|------|------|------|
| MCU | STM32F103RCT6 | LQFP64 | 256KB Flash, 48KB SRAM, 72MHz |
| UWB | DW1000 裸片 | QFN48 | 802.15.4-2011 UWB 收发器 |
| LDO | TPS73601DBVR | SOT-23-5 | TI 低噪声可调 LDO, 5V→3.3V |
| Balun | HHM1595A1 | 0805 | TDK 多层芯片巴伦 |

#### 晶振

| 用途 | 频率 | 封装 | 说明 |
|------|------|------|------|
| DW1000 时钟 | 38.4MHz | XTAL 38.4Mhz | 精度 ±20ppm, 影响测距 |
| STM32 HSE | 12MHz | 3225 | STM32 系统时钟 |

#### RF 链路

```
DW1000 RF_P/N (差分100Ω)
	→ HHM1595A1 Balun (差分→单端转换)
	→ 0402 匹配电容网络 (1.2pF, 10pF, 12pF, 18pF, 27pF)
	→ 滤波电容 (330pF, 820pF)
	→ LXDC2HL_18A (RF 电感/滤波器)
	→ SMA 母座 → 外置天线
```

#### 供电

| 电容 | 数值 | 封装 | 位置 |
|------|------|------|------|
| 钽电容 | 10µF/16V | TAN-A-3216 | LDO 输入 |
| 陶瓷电容 | 47µF | 0805 | LDO 输出 |
| 陶瓷电容 | 4.7µF, 10µF, 1µF | 0402/0603 | 电源滤波 |
| 去耦电容 | 0.1µF ×N | 0402 | 各 VDD 引脚 |
| 电感 | 100µH | 0402 | 电源滤波 |

#### 其他元件

| 元件 | 数值/型号 | 封装 | 说明 |
|------|----------|------|------|
| 上拉/下拉电阻 | 10kΩ ×N | 0402 | NRST, BOOT0, IRQ, CS 等 |
| LED 限流 | 270R | 0402 | LED 串联 PC13 |
| 反馈电阻 | 30.1kΩ, 52.3kΩ | 0402 | LDO VOUT 设定 |
| LED | RED ×4 | 0603D | 状态指示 |
| SWD | 2×4 排针 | 2.54mm | 3.3V/SWDIO/SWCLK/GND |
| USB | Molex 1050170001 | — | Micro USB 供电+数据 |
| SMA | SMA 母座 | SMA-2 | 外置 UWB 天线 |

#### 优势

| 维度 | 评价 |
|------|------|
| 灵活性 | ★★★ 可自选天线、匹配网络、板材 |
| RF 门槛 | 高 — 需要 50Ω 阻抗控制、匹配网络调试 |
| 体积 | ★★★ RF 部分占板面积中等, 外置天线独立放置 |
| 成本/块 | ~¥60 (不含充电宝) |
| 调试难度 | 高 — 可能需要频谱仪测试 RF 匹配 |
| 外置天线 | ✓ SMA 接口, 可选高增益棒状天线 |

---

### 方案 B：DWM1000 模块

使用 Qorvo 官方 DWM1000 模块（23×13×2.9mm, 24-pin 邮票孔）。模块内含 DW1000 芯片 + 38.4MHz 晶振 + RF 匹配网络 + PCB 天线。

#### 与方案 A 的区别

| 项目 | 方案 A (裸DW1000) | 方案 B (DWM1000) |
|------|------------------|------------------|
| DW1000 | QFN48 裸片, 需手工焊 | 模块 24-pin, 邮票孔或排针焊 |
| 晶振 | 外接 38.4MHz XTAL | 模块内含 |
| RF 匹配 | 自设计 (Balun + 0402匹配网络) | 模块内含, 出厂已调好 |
| 天线 | 外置 SMA | 模块 PCB 天线 / 可选 U.FL |
| PCB 层数 | 建议 4 层 (RF 阻抗控制) | 2 层即可 |
| RF 调试 | 需要测试验证 | 免调试 |
| 体积 | 中等 | 小 (模块 23×13mm) |

#### 原理图简化

```
PCB 上仅需:
	STM32F103RCT6 (LQFP64)
	+ DWM1000 模块 (SPI + IRQ + CS + RSTn + 3.3V + GND)
	+ TPS73601DBVR (5V→3.3V)
	+ 12MHz XTAL (STM32 HSE)
	+ 阻容 + LED + SWD + USB

删除:
	- DW1000 QFN48 裸片
	- HHM1595A1 Balun
	- 38.4MHz XTAL
	- 0402 RF 匹配电容网络
	- LXDC2HL_18A
	- SMA 母座
```

#### DWM1000 模块引脚 (标准 24-pin)

| 引脚 | 功能 | 连接 STM32 |
|------|------|-----------|
| VCC | 3.3V | 3.3V |
| GND | 地 | GND |
| SCK | SPI 时钟 | PA5 (SPI1_SCK) |
| MISO | SPI 数据 | PA6 (SPI1_MISO) |
| MOSI | SPI 数据 | PA7 (SPI1_MOSI) |
| CS | 片选 | PA4 (GPIO) |
| IRQ | 中断 | PB0 (EXTI) |
| RSTn | 复位 | PC5 (GPIO) |
| WAKEUP | 唤醒 | 可悬空或 PC4 |
| EXTON | 外部时钟 | 悬空 (模块内含晶振) |
| GPIO0~7 | 可编程IO | 悬空 (可选 LED 指示) |

#### 采购预算

| 物料 | 数量 | 单价 | 合计 |
|------|------|------|------|
| PCB 打样 (双面板) | 5片 | ¥5/5片 | ¥5 |
| STM32F103RCT6 | 5 | ¥12 | ¥60 |
| DWM1000 模块 | 5 | ¥85 | ¥425 |
| TPS73601DBVR | 5 | ¥3 | ¥15 |
| 12MHz XTAL 3225 | 5 | ¥1 | ¥5 |
| 阻容 + 其他 | 5套 | ¥5 | ¥25 |
| 充电宝 (Anchor) | 4 | ¥20 | ¥80 |
| **总计** | | | **~¥615** |

> 注：DWM1000 比 BU01/裸DW1000 贵不少。可用国产兼容模块 (如 BU01) 替代，价格 ~¥50。

---

### 方案 C：DWM3000 模块

使用 Qorvo 新一代 DWM3000 模块（23×13×2.9mm, 24-pin 邮票孔, **与 DWM1000 引脚兼容**）。内置 DW3110 IC, 支持 CH5+CH9。

#### 与 DWM1000 的关键差异

| 维度 | DWM1000 | DWM3000 |
|------|---------|---------|
| 核心 IC | DW1000 | DW3110 |
| IEEE 标准 | 802.15.4-2011 | 802.15.4z-2020 (BPRF) |
| 支持频段 | CH1-5 (3.5~6.5GHz) | CH5 (6.5GHz) + CH9 (8GHz) |
| 4GHz 低频段 | ✓ | ✗ |
| 数据速率 | 110K/850K/6.81M | 850K/6.81M (无110K) |
| 功耗 | TX ~140mA | 降低约 1/3~1/2 |
| 时间戳补偿 | 需软件手动补偿 | 硬件自动补偿 |
| 安全加密 | 基础 | AES 128/256 + 安全时间戳 |
| Apple U1 互通 | ✗ | ✓ |
| FiRa 认证 | ✗ | ✓ |
| 引脚兼容 | — | 与 DWM1000 完全兼容 |
| 驱动 | 开源, 社区成熟 | 新版闭源, 有旧版开源 |
| 价格 | ~¥85 (官方) | ~¥120 (官方) |

#### 注意事项

1. **TREK1000 固件是为 DW1000 写的, DWM3000 不兼容**。需要用 Qorvo 官方 DW3000 SDK 或开源驱动重新适配 Tag/Anchor 逻辑
2. **不再支持 110K 和 4GHz 频段** — 远距离场景不如 DWM1000
3. **CH9 (8GHz)** 全球免许可, 完全避开 5G 信号干扰
4. **与 DWM1000 引脚兼容** — 如果方案 B 的 PCB 预留好, 可后续替换

#### 采购预算

| 物料 | 数量 | 单价 | 合计 |
|------|------|------|------|
| PCB 打样 | 5片 | ¥5/5片 | ¥5 |
| STM32F103RCT6 | 5 | ¥12 | ¥60 |
| DWM3000 模块 | 5 | ¥120 | ¥600 |
| TPS73601DBVR | 5 | ¥3 | ¥15 |
| 12MHz XTAL 3225 | 5 | ¥1 | ¥5 |
| 阻容 + 其他 | 5套 | ¥5 | ¥25 |
| 充电宝 (Anchor) | 4 | ¥20 | ¥80 |
| **总计** | | | **~¥790** |

> 警告：DWM3000 固件与 TREK1000 不兼容，需要重新移植 Tag/Anchor 逻辑。开发周期显著增加。

---

### 三版对比总览

| 维度 | A: 裸 DW1000 | B: DWM1000 模块 | C: DWM3000 模块 |
|------|-------------|----------------|----------------|
| RF 门槛 | 高 (自设计匹配网络) | 无 (模块内含) | 无 (模块内含) |
| 天线 | 外置 SMA (灵活) | PCB 天线 / U.FL 可选 | PCB 天线 / U.FL 可选 |
| 最小 PCB 层数 | 4 层 (RF 阻抗) | 2 层 | 2 层 |
| 单板成本 | ~¥60 | ~¥125 | ~¥160 |
| 5板总成本 | ~¥435 | ~¥615 | ~¥790 |
| TREK1000 兼容 | ✓ (需要 Keil) | ✓ (需要 Keil) | ✗ (需要新 SDK) |
| 社区资源 | 丰富 (Decawave 参考设计) | 丰富 (官方模块) | 较少 (新一代) |
| 焊接难度 | 高 (QFN48) | 低 (邮票孔/排针) | 低 (同 DWM1000) |
| 调试难度 | 高 (RF 匹配) | 低 (免调) | 低 (免调) |
| 开发周期 | 4~6 周 | 2~3 周 | 6~8 周 |
| 远距离能力 | ★★★ (外置天线+SMA) | ★★ (内置天线) | ★★ (无110K模式) |
| 未来扩展 | — | 可换 DWM3000 (同引脚) | Apple U1 互通 |

### 2.2 MCU 选择 (三版共用)

STM32F103RCT6 (LQFP64, 256KB Flash, 48KB SRAM)。原版原理图为 T8U6 (QFN36, 64KB)，更换原因：

1. **TREK1000 Station 原生支持** — 为 LQFP64 256KB 设计，无需裁剪
2. **Flash 充足** — T8U6 装不下 Station 完整固件
3. **易于焊接** — LQFP64 可手焊，QFN36 底部焊盘难
4. **GPIO 充裕** — 51 vs 26
5. 三版方案共用此 MCU

### 2.3 推荐方案

| 场景 | 推荐 | 理由 |
|------|------|------|
| 快速验证、比赛赶时间 | **B (DWM1000)** | 免 RF 调试, 2 周可出, TREK1000 直接兼容 |
| 追求最远距离、RF 有经验 | **A (裸 DW1000)** | 外置天线, SMA 可换高增益, 已有原理图 |
| 长远布局、Apple 生态 | **C (DWM3000)** | 新一代标准, 但开发周期长, 固件需重写 |

> **默认建议 B (DWM1000/BU01)**：开发最快, RF 免调, TREK1000 固件直接烧录, 成本可控。

---

## 三、软件改动

### 3.1 改动范围

**只改 stm32_uwb 板**, Tag 板和 Anchor 板烧 TREK1000 官方固件不动。

### 3.2 改动文件

| 文件 | 改动 | 行数 |
|------|------|------|
| `uwb_solver.c` | 新增 `uwb_parser_parse_mc()` + 修改 `uwb_parser_feed()` 两行路由 | +62 行 |
| 其余全部 .c/.h | 不改 | 0 |

### 3.3 mc 协议格式

```
mc 0f 0000064a 000005f2 00000680 00000610 000a 01 000001f4 t0:0\r\n
│  │   │         │         │         │        │    │    │       │
│  │   S1(mm)    S2        S3        S4       lcnt rnum time   t<tag>:a<anchor>
│  └─ valid_mask: bit0=S1, bit1=S2, bit2=S3, bit3=S4
└── 帧头

数值全部 HEX，距离单位 mm。
```

### 3.4 新增函数

```c
/**
 * mc 协议解析: mc <valid_hex> <S1_mm> <S2_mm> <S3_mm> <S4_mm> ...
 */
static int uwb_parser_parse_mc(struct UWB_Parser *p, const char *line,
                                uint32_t now_ms) {
    // 1. 验证帧头 "mc "
    if (line[0] != 'm' || line[1] != 'c' || line[2] != ' ') return 0;
    const char *ptr = line + 3;
    char *end;

    // 2. valid_mask (1 hex char)
    int mask = (int)strtol(ptr, &end, 16);
    if (ptr == end) return 0;
    ptr = end;

    // 3. 4 个距离值 (HEX, mm)
    uint32_t dists[4] = {0};
    for (int i = 0; i < 4; i++) {
        while (*ptr == ' ') ptr++;
        if (*ptr < '0') return 0;
        dists[i] = (uint32_t)strtol(ptr, &end, 16);
        if (ptr == end) return 0;
        ptr = end;
    }

    // 4. 存储 (mm → m), 仅 valid 的锚点
    for (int i = 0; i < 4; i++) {
        if (mask & (1 << i)) {
            float d = dists[i] / 1000.0f;
            if (d > 0.0f && d < 50.0f) {
                p->distances[i]      = d;
                p->last_update_ms[i] = now_ms;
            }
        }
    }
    return 1;
}
```

### 3.5 修改 parser_feed()

```c
int uwb_parser_feed(struct UWB_Parser *p, char c, uint32_t now_ms) {
    if (c == '\n' || c == '\r') {
        if (p->buffer_idx > 0) {
            p->buffer[p->buffer_idx] = '\0';
            p->buffer_idx = 0;
            // ★ 新增两行
            if (p->buffer[0] == 'm' && p->buffer[1] == 'c')
                return uwb_parser_parse_mc(p, p->buffer, now_ms);
            if (p->buffer[0] == '$')                       // 兼容 JZM01
                return uwb_parser_parse_line(p, p->buffer, now_ms);
            return 0;
        }
        return 0;
    }
    // 其余不变...
}
```

### 3.6 信号链（不变）

```
mc(HEX) → parse_mc → 距离(m) → DistanceFilter(中值W=8+EMA) →
UWBSolver(WLSQ三边+α-β) → local_to_gps → NMEA_Generator →
USART3 DMA chain → FC GPS 口
```

### 3.7 stm32_uwb 配置调整

USART2 baud 从 19200 改为 **115200** (匹配 TREK1000 UART 输出)。

---

## 四、固件部署

### 4.1 固件分配

| 板子 | 固件 | 编译工具 | 角色 | UART配置 |
|------|------|---------|------|---------|
| Anchor S1-S4 (×4) | TREK1000 Station | Keil MDK | A0/A1/A2/A3 (DIP) | A0输出mc 115200 |
| Tag (×1) | TREK1000 Tag | Keil MDK | 固定Tag | 输出mc 115200 |
| stm32_uwb (×1) | 现有 + mc补丁 | STM32CubeIDE | 定位+NMEA | USART2→115200, USART3→57600 |

### 4.2 TREK1000 Station 工程

- **路径**: `Download/UWB_TDOA_STATION_V1_1-master/Code/gataway20181020/test_rx11/`
- **MCU**: STM32F103RC/LQFP64 (与 RCT6 兼容)
- **关键宏**: `ANCTOANCTWR=1`, `DEEP_SLEEP=1`
- **编译**: Keil MDK ARMCC
- **烧录**: ST-Link SWD, 4块板烧同一hex
- **角色选择**: DIP 开关 (S4 ON=Anchor模式,S5~S7=地址编码)
- **推荐模式**: 模式3 (S2=OFF 110K, S3=ON CH5) — 远距离+避开5G干扰

### 4.3 TREK1000 Tag 工程

- **路径**: `Download/UWB_TDOA_TAG_V1_1-master/`
- **MCU**: STM32F103/LQFP64
- **OS**: FreeRTOS (独立task控制发射)
- **编译**: Keil MDK ARMCC
- **输出**: mc 帧, 约 3.57Hz (110K模式)

### 4.4 部署步骤

1. PCB 打样 → 焊接 5 块 RCT6 板 → 万用表量测供电不短路
2. ST-Link SWD 烧录 4 块 Station + 1 块 Tag
3. 4 Anchor 各接充电宝 + 外置天线, DIP 设好角色
4. A0 UART → USB-TTL → 电脑串口助手, 验证 mc 输出
5. Tag 板上电, A0 串口确认 4 个距离有效
6. 修改 stm32_uwb 固件 (加 parse_mc), 编译烧录
7. Tag 板 UART TX → stm32_uwb USART2 RX, 确认 printf 距离正确
8. stm32_uwb USART3 → FC GPS 口, Mission Planner 确认 GPS Status 3D Fix
9. 移动 Tag, 确认定位跟踪正常

---

## 五、测试方案

### 5.1 逐级验证

| 层级 | 测试项 | 通过标准 | 工具 |
|------|--------|---------|------|
| L1 | DW1000 SPI 连通 | `dwt_readdevid()` = 0xDECA0130 | ST-Link 断点 |
| L2 | A0 UART 输出 mc | 串口助手看到 mc 帧, ~3.57Hz | USB-TTL + 串口助手 |
| L3 | TWR 测距精度 | 已知距离 1m, mc 输出误差 ±10cm | 卷尺 + HEX→DEC |
| L4 | mc 解析正确性 | stm32_uwb printf 距离 = 串口助手原始值 | stm32_uwb debug UART |
| L5 | 三边定位精度 | 静止 RMS < 20cm | 已知坐标点验证 |
| L6 | NMEA → FC | Mission Planner: GPS Status 3D Fix | Mission Planner |
| L7 | 端到端飞行 | Loiter 模式下稳定悬停 | 实飞 |

### 5.2 常见问题排查

| 症状 | 可能原因 | 解决 |
|------|---------|------|
| mc 只有 1~2 个有效距离 | Tag 和某些 Anchor 信道不匹配 / 天线朝向 | 确认所有 Anchor DIP 信道一致; 天线朝向 Tag |
| SPI 返回 0xFFFFFFFF | DW1000 焊接不良 / RSTn 未释放 | 检查 SPI + RSTn 引脚焊接; 量测供电 |
| parse_mc 解析失败 | baud 不匹配 | 确认 USART2 baud = 115200 |
| 定位抖动 > ±50cm | 单锚点野值 / 滤波不够 | 调低 DIST_ALPHA; 检查 MAX_DIST_JUMP |
| FC 无 GPS lock | baud 不一致 / GPS_ORIGIN 不对 | 确认 FC SERIALx_BAUD=57600; 检查 origin |
| 38.4MHz 不起振 | XTAL 匹配电容不对 | 按原理图确认负载电容; 考虑换 TCXO |
| RF 不通 / 距离极短 | Balun / 匹配网络 / SMA 虚焊 | 0402 RF 元件检查; 天线拧紧; SMA 量通断 |
| 编译失败 (Keil) | 缺少 ARMCC / 库不匹配 | 装 Keil MDK 社区版; 或迁移到 CubeMX GCC |

### 5.3 可调滤波参数

| 参数 | 默认 | 调大效果 | 调小效果 |
|------|------|---------|---------|
| DIST_ALPHA | 0.35 | 距离更平滑 | 响应更快 |
| POS_ALPHA | 0.30 | 位置响应更快 | 更平滑 |
| POS_BETA | 0.05 | 速度响应更快 | 速度更稳 |
| VEL_ALPHA | 0.30 | NMEA 速度响应快 | NMEA 速度更平滑 |
| DIST_WINDOW_SIZE | 8 | 更平滑(延迟增加) | 响应更快 |
| POS_WINDOW_SIZE | 5 | 更平滑 | 响应更快 |
| MAX_DIST_JUMP | 0.8m | 容忍更大突变 | 更严格剔除野值 |
