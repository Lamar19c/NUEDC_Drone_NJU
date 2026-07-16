# STM32 UWB 固件烧录指南

> 此文档供 Claude Code 会话使用。每次烧录时参考此文确保流程一致。

## 硬件准备

### 串口烧录 (UART bootloader)

| STM32 Blue Pill | USB-TTL 模块 | 说明 |
|----------------|-------------|------|
| **PA9** (USART1_TX) | RX | STM32 发送 → PC 接收 |
| **PA10** (USART1_RX) | TX | PC 发送 → STM32 接收 |
| **GND** | GND | 共地 |

- STM32 通过 USB 口或 3.3V 供电
- **必须**：BOOT0 跳线帽插到 **`1`**，然后按 **RESET** 按钮

### ST-Link 烧录 (SWD)

| ST-Link | STM32 Blue Pill |
|---------|-----------------|
| SWDIO | **PA13** |
| SWCLK | **PA14** |
| GND | GND |
| 3.3V | 3V3 |

## 烧录命令

### 工具路径

```
"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
```

### 固件路径

```
C:\Users\cheng\Desktop\homework\无人机\main\stm32_uwb\Debug\stm32_uwb.hex
```

### UART 串口烧录 (推荐)

```bash
"/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe" \
  -c port=COMx br=115200 P=EVEN \
  -d "C:/Users/cheng/Desktop/homework/无人机/main/stm32_uwb/Debug/stm32_uwb.hex" \
  -v
```

> 将 `COMx` 替换为实际串口号（如 COM10、COM11）

### ST-Link SWD 烧录

```bash
"/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe" \
  -c port=SWD mode=UR \
  -d "C:/Users/cheng/Desktop/homework/无人机/main/stm32_uwb/Debug/stm32_uwb.hex" \
  -v
```

## Claude Code 烧录速查

```
# 串口烧录 (替换 COMx)
"/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe" -c port=COMx br=115200 P=EVEN -d "C:/Users/cheng/Desktop/homework/无人机/main/stm32_uwb/Debug/stm32_uwb.hex" -v
```

## 常见问题

| 症状 | 原因 | 解决 |
|------|------|------|
| `Timeout error` | BOOT0≠1 或未按 RESET | BOOT0→1，按 RESET，重试 |
| `Cannot open port` | 串口被占用 | 关闭所有串口终端/CubeIDE Serial Monitor |
| `No device found` (ST-Link) | BOOT0=1 残留或 SWD 锁死 | 先用串口全片擦除恢复 SWD |
| Flash 后程序不运行 | BOOT0 未恢复 | BOOT0→0，按 RESET |
| 串口号不确定 | USB-TTL 换了口 | `powershell -Command "[System.IO.Ports.SerialPort]::GetPortNames()"` |

## 烧录后运行

1. **BOOT0 跳线帽 → `0`**
2. 按 **RESET**
3. PC 端串口助手打开对应 COM 口 @ **115200 baud**
