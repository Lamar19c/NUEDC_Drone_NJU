# TREK1000 迁移完整指南

> 基于 Schematic2 网表解析文档 (`docs/Schematic2-Netlist-解析.md`)
> 目标: 将 JZM01 黑盒 UWB 模块替换为 TREK1000 开源方案

---

## 一、迁移总览

```
迁移前:  JZM01 模块 ──$DIST──→ stm32_uwb ──NMEA──→ FC GPS
迁移后:  TREK1000 Tag ──mc──→ stm32_uwb ──NMEA──→ FC GPS
```

| 组件 | 迁移前 | 迁移后 | 改动 |
|------|--------|--------|------|
| UWB 测距 | JZM01 (黑盒, $DIST协议) | TREK1000 Tag (开源, mc协议) | 硬件替换 |
| 地面锚点 | JZM01 Anchor ×4 | TREK1000 Station ×4 | 硬件替换 |
| 定位解算 | stm32_uwb (C8T6) | stm32_uwb (不变) | 仅加 mc 解析 |
| NMEA 输出 | stm32_uwb → FC | 不变 | 无 |

---

## 二、硬件方案对照

### 方案 A: 裸 DW1000 + Balun（对应 Schematic2 网表）

基于 `C:\Users\cheng\Desktop\product\UWB\Netlist_Schematic2_2026-07-13.enet` 的设计:

| 元件 | 型号 | 封装 |
|------|------|------|
| MCU | STM32F103CBU6 | UFQFPN-48 |
| UWB | DW1000 裸片 | QFN-48 |
| Balun | HHM1595A1 | 0805 |
| 天线 | ACS5200HFAUWB | 芯片天线 |
| USB-UART | CH340X | MSOP-10 |
| EEPROM | AT24C02D | SOT-23-5 |
| LDO | ME6206A33M3G | SOT-23-3 |
| 晶振(DW) | 38.4MHz | SMD2016-4P |
| 晶振(MCU) | 16MHz | SMD2016-4P |

> 此方案 RF 链路需自调试，适合有 RF 经验者。

### 方案 B: DWM1000/BU01 模块（推荐）

使用 DWM1000 模块替换裸 DW1000 + Balun + RF 匹配网络 + 38.4MHz 晶振:

| 移除 (方案A有) | 替代 (模块内含) |
|---------------|---------------|
| DW1000 QFN48 裸片 | — |
| HHM1595A1 Balun | — |
| 0402 RF 匹配电容 (C13/C14/C17/C34/C35/C90) | — |
| 38.4MHz 晶振 (U12) + C5/C6 | — |
| 芯片天线 ANT1 | DWM1000 PCB 天线 |

> 方案 B 的 MCU 建议从 CBU6(UQFN48) 升级为 RCT6(LQFP64) — 256KB Flash, 手焊友好, TREK1000 Station 原生支持。

---

## 三、软件改动清单

### 3.1 已完成的改动 ✅

| 文件 | 改动 | 状态 |
|------|------|------|
| `stm32_uwb/Core/Src/uwb_solver.c` | 新增 `uwb_parser_parse_mc()` + `uwb_parser_feed()` 路由 | ✅ 已包含 |
| `stm32_uwb/Core/Src/usart.c` | USART2 BaudRate = 115200 | ✅ 已修改 |

验证方法:
```bash
grep "uwb_parser_parse_mc" stm32_uwb/Core/Src/uwb_solver.c  # 应找到函数定义
grep "BaudRate = 115200" stm32_uwb/Core/Src/usart.c           # 应匹配 USART2
```

### 3.2 待完成的改动 ⚠️

#### A. CubeMX .ioc 同步 (重要!)

`stm32_uwb/stm32_uwb.ioc` 第 148 行需手动修改:

```
# 修改前:
USART2.BaudRate=19200

# 修改后:
USART2.BaudRate=115200
```

> 不修改会导致 CubeMX 重新生成代码时将 usart.c 恢复为 19200。

#### B. 滤波参数适配 (建议)

TREK1000 mc 输出约 **3.57 Hz** (110K 模式), 比 JZM01 (~5 Hz) 慢。在 `uwb_config.h` 中建议调整:

```c
// 当前值          建议值         原因
DIST_TIMEOUT_MS  150    →   400    // mc 帧间隔 ~280ms, 150ms 太短会频繁超时
DIST_WINDOW_SIZE 8      →   5      // 8帧 @3.57Hz = 2.24s 滞后, 5帧 = 1.4s
```

> 这些是**建议值**，可根据实际飞行表现调整。如果保持现有值也能工作，但响应会偏慢。

---

## 四、固件部署

### 4.1 TREK1000 固件编译

| 固件 | 路径 | 编译工具 | MCU |
|------|------|---------|-----|
| Station | `Download/UWB_TDOA_STATION_V1_1-master/Code/gataway20181020/test_rx11/` | Keil MDK | STM32F103RC |
| Tag | `Download/UWB_TDOA_TAG_V1_1-master/` | Keil MDK | STM32F103 |

**Station 配置:**
- `ANCTOANCTWR=1` — Anchor-to-Anchor TWR 时钟同步
- `DEEP_SLEEP=1` — 低功耗
- DIP 开关: S4=ON (Anchor模式), S5~S7=地址 (A0=000, A1=001, A2=010, A3=011)
- 模式3 (S2=OFF 110K, S3=ON CH5) — 推荐

**Tag 配置:**
- FreeRTOS 调度
- 自动输出 mc 帧 @ ~3.57Hz
- UART 115200

### 4.2 烧录与验证

```
步骤 1: PCB 打样 → 焊接 5 块 (4 Anchor + 1 Tag)
步骤 2: ST-Link 烧录固件 → 设 DIP 角色
步骤 3: A0 UART → USB-TTL → 串口助手 → 确认 mc 输出
步骤 4: Tag 板上电 → A0 串口确认 4 距离有效
步骤 5: Tag UART TX → stm32_uwb USART2 RX → printf 确认解析
步骤 6: stm32_uwb USART3 → FC GPS 口 → Mission Planner 3D Fix
步骤 7: 实飞验证 Loiter 悬停
```

---

## 五、连接拓扑

### 5.1 无人机端

```
Tag 板 (新PCB)                    stm32_uwb (C8T6, 已有)
┌──────────────────┐           ┌──────────────────────────┐
│ PA9  USART1_TX   ├──杜邦线──→│ PA3  USART2_RX (115200)   │
│ GND              ├──杜邦线──→│ GND                       │
│ 5V ← 无人机电源   │           │ PB10 USART3_TX ──→ FC GPS│
│ DWM1000 ←→ 天线   │           │ PA9  USART1_TX → debug   │
└──────────────────┘           └──────────────────────────┘
```

### 5.2 地面 Anchor

```
Anchor ×4 (相同 PCB, 不同固件角色)
┌─────────────────────┐
│ Micro USB ← 充电宝   │
│ DWM1000 ←→ 天线      │
│ DIP 开关 设角色       │
│ A0: UART → USB-TTL   │  (调试)
│ A1/A2/A3: UART 悬空  │
└─────────────────────┘

Anchor 之间: 无连线, 通过 UWB 无线电通信
```

---

## 六、协议处理流程

```
TREK1000 Tag UART TX (115200)
  │  mc 0f 0000064a 000005f2 00000680 00000610 000a 01 000001f4 t0:0\r\n
  │  │   │         │         │         │
  │  │   S1(mm)    S2        S3        S4        (HEX)
  │  └─ valid_mask: 0x0f = S1~S4 全部有效
  └── 帧头
  ▼
stm32_uwb USART2 RX (PA3) — DMA 循环缓冲
  ▼
uwb_parser_feed() → 检测 'm'+'c' → uwb_parser_parse_mc()
  ▼  mm → m, 仅存储 valid_mask 标记的有效锚点
distances[4] (float, 米)
  ▼
DistanceFilter — 滑动窗口中值 + EMA 低通
  ▼
UWBSolver — WLSQ 三边测量 (4锚点3D / 3锚点2D)
  ▼
PositionFilter — 中值滤波 → α-β 预测-校正
  ▼
local_to_gps() — NED → WGS84 (E7 整数)
  ▼
NMEA_Generator — $GPGGA + $GPRMC + $GPVTG
  ▼
USART3 DMA → FC GPS 口 (57600)
```

---

## 七、测试验证

### 7.1 逐级测试

| 层级 | 测试项 | 通过标准 | 工具 |
|------|--------|---------|------|
| L1 | DW1000 SPI | `dwt_readdevid()` = 0xDECA0130 | ST-Link 断点 |
| L2 | mc 输出 | 串口助手看到 mc 帧 | USB-TTL |
| L3 | TWR 精度 | 1m 距离误差 < ±10cm | 卷尺 |
| L4 | mc 解析 | stm32_uwb printf = 串口助手原始值 | Debug UART |
| L5 | 三边定位 | 静止 RMS < 20cm | 已知坐标验证 |
| L6 | NMEA → FC | Mission Planner GPS 3D Fix | Mission Planner |
| L7 | 端到端飞行 | Loiter 稳定悬停 | 实飞 |

### 7.2 常见问题

| 症状 | 可能原因 | 排查 |
|------|---------|------|
| mc 无输出 | Tag 固件未烧录 / DIP 不对 | 串口助手直接接 Tag 的 PA9 |
| mc 只有 1~2 距离 | 信道不匹配 / 天线朝向 | 确认 DIP 信道一致 |
| parse_mc 失败 | baud 不匹配 | 确认 USART2 = 115200 |
| 定位抖动 > 50cm | 锚点野值 / 滤波太慢 | 调低 DIST_ALPHA |
| FC 无 GPS lock | baud 不一致 | 确认 FC SERIALx_BAUD = 57600 |

---

## 八、文件清单

### 当前迁移包 (`trek1000_migration/`)

| 文件 | 用途 |
|------|------|
| `README.md` | 快速集成指南 |
| `MIGRATION_COMPLETE.md` | 本文件 — 完整迁移指南 |
| `HARDWARE.md` | 硬件连接方案 (方案A/B/C) |
| `NETLIST.md` | 方案B 结构化网表 |
| `SCHEMATIC_GUIDE.md` | EasyEDA 手工绘制指南 |
| `uwb_solver.c` | 已修改的求解器 (含 mc 解析) |
| `uwb_solver_original.c` | 原始求解器 (备份) |
| `uwb_solver.patch` | Git diff 补丁 |
| `trek1000_sch_dwm1000.json` | 方案B EasyEDA 原理图 JSON |
| `generate_schematic.py` | EasyEDA API 自动生成脚本 |
| `place_components.js` | EasyEDA 元件布局脚本 |

### 参考文档 (外部)

| 文件 | 用途 |
|------|------|
| `../docs/superpowers/specs/2026-07-13-trek1000-migration-design.md` | 迁移设计规格 |
| `../../product/UWB/docs/Schematic2-Netlist-解析.md` | 方案A Schematic2 网表解析 |

---

## 九、迁移完成判定

- [ ] TREK1000 Station ×4 + Tag ×1 固件编译通过
- [ ] PCB 打样焊接 5 块
- [ ] Tag 板 mc 输出验证 (串口助手)
- [ ] stm32_uwb `.ioc` USART2 baud = 115200
- [ ] stm32_uwb `uwb_config.h` DIST_TIMEOUT_MS = 400
- [ ] 4 锚点定位 RMS < 20cm (静止测试)
- [ ] Mission Planner GPS 3D Fix
- [ ] 实飞 Loiter 悬停稳定
