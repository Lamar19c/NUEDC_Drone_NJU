# TREK1000 适配项目 — 基于 Schematic2 网表

## 目录结构

```
trek1000_adapted/
├── tag/          ← TREK1000 Tag 固件 (无人机端, FreeRTOS)
├── station/      ← TREK1000 Station 固件 (地面锚点, 裸机)
├── setup.sh      ← 初始化脚本 (已执行)
└── README.md     ← 本文件
```

## 适配内容

| 项目 | 原始值 | 适配值 | 原因 |
|------|--------|--------|------|
| RSTn | PA0 | **PB4** | 匹配 Schematic2 网表 |
| IRQ | PB5 | **PB0** | 匹配 Schematic2 网表 |
| HSE | 12MHz | **16MHz** | Schematic2 晶振 |

SPI(PA4-7)、UART(PA9/PA10) 与原始 TREK1000 一致，无需改动。

## 编译

### Tag (无人机)

```
1. Keil MDK 打开 tag/ 目录下的工程文件 (.uvproj)
2. 工程设置: Options → C/C++ → Define: HSE_VALUE=16000000
3. 确认 STM32F103RC/RCT6 为目标 MCU
4. Build → 生成 .hex
5. ST-Link 烧录
```

### Station ×4 (地面锚点)

```
1. Keil MDK 打开 station/ 目录下的工程文件
2. 同上设置 HSE_VALUE=16000000
3. Build → 同一 .hex 烧录 4 块板
4. DIP 开关设置角色:
   - S4=ON (Anchor 模式)
   - S5-S7: 000=A0, 001=A1, 010=A2, 011=A3
   - 模式3: S2=OFF(110K) S3=ON(CH5)
```

## 连接方式

```
Tag 板                      stm32_uwb (已有 C8T6)
PA9 (USART1_TX) ──杜邦线──→ PA3 (USART2_RX)
GND             ──────────→ GND

Baud: 115200, 协议: mc (HEX, mm)
```

## 重新初始化

如需重新生成（比如更新了 port_adapted/ 下的文件）：

```bash
cd trek1000_adapted
bash setup.sh
```

## 相关文档

- `../trek1000_migration/MIGRATION_COMPLETE.md` — 完整迁移指南
- `../trek1000_migration/PINMAP_ANALYSIS.md` — 引脚对照分析
- `../../product/UWB/docs/Schematic2-Netlist-解析.md` — 网表解析
