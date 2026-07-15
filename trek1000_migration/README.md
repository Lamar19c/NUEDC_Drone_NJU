# TREK1000 Migration — stm32_uwb 软件补丁

## 概述

将 JZM01 模块替换为 TREK1000 Tag 板后，stm32_uwb 板需要支持 `mc` 协议（替代原来的 `$DIST` 协议）。

## 改动文件

| 文件 | 改动说明 |
|------|---------|
| `uwb_solver.c` | 新增 `uwb_parser_parse_mc()` 函数 + `uwb_parser_feed()` 加路由 |

其余文件 (`uwb_solver.h`, `uwb_config.h`, `uwb_nmea.c/h`, `main.c`) **无需改动**。

## 集成步骤

### 1. 替换文件

```bash
# 备份原文件
cp stm32_uwb/Core/Src/uwb_solver.c stm32_uwb/Core/Src/uwb_solver.c.bak

# 覆盖为新版
cp trek1000_migration/uwb_solver.c stm32_uwb/Core/Src/uwb_solver.c
```

### 2. 修改 USART2 波特率

在 `stm32_uwb/Core/Src/usart.c` 或 CubeMX `.ioc` 中：
- **USART2** baud 从 **19200** 改为 **115200**（匹配 TREK1000 UART 输出）

### 3. 重新编译

STM32CubeIDE: Project → Build All

### 4. 烧录

ST-Link 烧录 `stm32_uwb.elf` 到 stm32_uwb 板（C8T6）

## 协议兼容

`uwb_parser_feed()` 同时兼容三种协议输入：

| 帧头 | 协议 | 来源 |
|------|------|------|
| `mc` | TREK1000 mc (HEX, mm) | TREK1000 Tag 板 |
| `$DIST` | JZM01 $DIST (ASCII, m) | JZM01 模块 (fallback) |

无需配置，自动识别。两种 Tag 可以随时切换。

## 数据流

```
Tag板 UART TX (115200)
    │ mc 0f 0000064a 000005f2 00000680 00000610 ...
    ▼
stm32_uwb USART2 RX (PA3)
    │ uwb_parser_feed() → uwb_parser_parse_mc()
    ▼
distances[4] (m) → DistanceFilter → UWBSolver → NMEA → FC
```

## 测试验证

1. Tag 板上电，stm32_uwb debug printf 应显示距离数组
2. 距离值与 A0 串口助手原始 mc 数据对比（HEX → 十进制 ÷ 1000）
3. 定位输出 (x,y,z) 与 Tag 实际位置对比（静止 RMS < 20cm）
4. Mission Planner 上 GPS 3D Fix

## 回滚

```bash
cp stm32_uwb/Core/Src/uwb_solver.c.bak stm32_uwb/Core/Src/uwb_solver.c
```
