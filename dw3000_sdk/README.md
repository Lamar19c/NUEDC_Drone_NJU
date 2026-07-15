# DW3000 UWB Tag 定位程序 — NJU 板

## 硬件

- MCU: STM32F103CBT6 (LQFP-48, 8MHz HSE → 72MHz)
- UWB: Qorvo DWM3000TR13 (DW3110 内核)
- 飞控接口: UART1 PA9/PA10 (115200)
- Debug: UART2 PA2/PA3 (115200)
- EEPROM: AT24C64 I2C PB6/PB7

## 目录结构

```
dw3000_sdk/
├── driver/          ← dw3000-decadriver-source (Qorvo 08.02.02)
├── libdeca/         ← libdeca (TWR 库)
├── app/
│   ├── main.c       ← 主程序入口
│   ├── board.c/h    ← NJU 板硬件驱动 (SPI/UART/I2C/LED)
│   ├── ranging.c/h  ← DW3000 SS-TWR 测距状态机
│   └── protocol.c/h ← mc 协议输出 (可选)
└── README.md
```

## 编译

### Keil MDK

1. 新建工程，选 STM32F103CBT6
2. 添加源文件:
   - `app/main.c`, `app/board.c`, `app/ranging.c`
   - `driver/dwt_uwb_driver/**/*.c` (除 dw3720/)
   - `driver/platform/deca_compat.c`
   - STM32F1xx HAL 库
3. Include paths:
   - `app/`
   - `driver/dwt_uwb_driver/`
   - `driver/dwt_uwb_driver/dw3000/`
   - `driver/platform/`
4. Define: `HSE_VALUE=8000000`, `USE_HAL_DRIVER`, `STM32F103xB`
5. Build → ST-Link 烧录

### STM32CubeIDE

导入源码同上，CubeMX 配置:
- HSE: 8MHz Crystal
- SPI1: Full-Duplex Master, NSS=Software
- USART1: 115200 (FC)
- USART2: 115200 (Debug)
- I2C1: 100kHz

## 数据流

```
NJU 板 (DW3000 Tag)
  │  DS-TWR 与 4 Anchor 测距
  │  mc 帧输出 UART1 PA9 @115200
  ▼
stm32_uwb (C8T6) PA3
  │  三边定位 + NMEA → FC
  ▼
FC GPS 口
```

## 待校准

| 参数 | 默认值 | 说明 |
|------|--------|------|
| ANT_DELAY_TX | 16450 | 需用已知距离校准 |
| ANT_DELAY_RX | 16450 | 同上 |
| Anchor 坐标 | — | 在 stm32_uwb 的 uwb_config.h 中配置 |
