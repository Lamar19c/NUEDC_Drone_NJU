# STM32F103C8T6 UWB 定位模块 — 设计文档

## 1. 背景与目标

当前 UWB 室内定位系统 (`UWB/uwb.py`) 运行在机载计算机（Raspberry Pi / 旭日X3）上，通过 USB-TTL 连接 JZM01 UWB 基座读取 `$DIST` 测距数据，经 EKF + 三边测量解算位置，输出 NMEA 伪 GPS 信号到飞控。

**目标**：将定位解算功能从机载计算机迁移到 STM32F103C8T6 单片机，取消机载计算机对 UWB 定位的依赖，降低系统成本（~¥12）和重量（~5g）。

已有参考实现：`UWB/uwb_arduino/` 目录下的 Arduino C++ 移植版（双重中值滤波替代 EKF），已验证通过 Nano 33 BLE 和 ESP32 平台。

## 2. 关键决策

| 决策项 | 结论 | 理由 |
|--------|------|------|
| 开发框架 | STM32CubeIDE + HAL 裸机 | 用户选择，最精简、性能最优 |
| 锚点坐标 | 固定值，硬编码 `uwb_config.h` | 简化部署，无需现场标定 |
| 滤波算法 | 双重中值滤波（距离 + 位置），无 EKF | STM32F103 无 FPU，中值滤波整数运算更适合 M3 内核 |
| 代码复用 | `uwb_solver.h` + `uwb_nmea.h` 算法模块零改动 | 已验证正确，纯 C++ 标准库无 Arduino 依赖 |
| 标定流程 | 不移植，坐标手动填入头文件 | 用户决定用硬编码固定值 |

## 3. 文件结构

```
stm32_uwb/
├── Core/Src/main.c              # HAL 初始化 + 主循环 (~150行)
├── Core/Inc/uwb_config.h        # 配置参数 — 锚点坐标、串口、滤波参数 (~80行)
├── Core/Inc/uwb_solver.h        # UWB_Parser + DistanceFilter + PositionFilter + UWBSolver (~400行)
├── Core/Inc/uwb_nmea.h          # NMEA_Generator — 坐标转换 + $GPGGA/$GPRMC 生成 (~150行)
└── stm32_uwb.ioc                # STM32CubeMX 项目文件 (引脚/时钟/外设配置)
```

## 4. 硬件接线

### 串口分配

| USART | 引脚 | 方向 | 连接目标 | 波特率 | 用途 |
|-------|------|------|---------|--------|------|
| USART1 | PA9 (TX) | → | USB-TTL → PC | 115200 | 调试 printf 输出 |
| USART1 | PA10 (RX) | ← | USB-TTL ← PC | 115200 | 预留 |
| USART2 | PA3 (RX) | ← | JZM01 基座 TX | 19200 | `$DIST` 测距数据接收 |
| USART2 | PA2 (TX) | → | JZM01 基座 RX | 19200 | 预留 (配置/查询) |
| USART3 | PB10 (TX) | → | 飞控 GPS RX (Telem2) | 57600 | NMEA 伪 GPS 输出 |
| USART3 | PB11 (RX) | ← | 飞控 GPS TX | 57600 | 预留 |

### 供电与调试

| 引脚 | 连接 |
|------|------|
| 5V | ← 飞控 BEC 5V 输出（或独立 BEC） |
| GND | ↔ JZM01 基座 GND + 飞控 GND + USB-TTL GND |
| PA13 (SWDIO) | ↔ ST-Link V2 |
| PA14 (SWCLK) | ← ST-Link V2 |
| PC13 | → 板载 LED（状态指示） |

### 关键注意事项

- **电平匹配**：STM32F103 是 3.3V 逻辑，JZM01 基座需确认 TX 电平。若 JZM01 为 5V TTL，需串 1kΩ 电阻或使用电平转换模块
- **共地**：所有设备 GND 必须连通
- **烧录**：使用 ST-Link V2 (四线: 3.3V, GND, SWDIO, SWCLK)

## 5. 算法与数据流

```
$DIST,M1,S<id>,<m>\n  (JZM01 → USART2 RX, DMA + 空闲中断)
         │
         ▼
  UWB_Parser.feed(char c)     ← 逐字符接收，行缓冲，解析锚点ID和距离
         │
         ▼
  DistanceFilter.filter()     ← 逐锚点 8帧滑动窗口中位数 + 跳变限幅 (0.8m)
         │
         ▼
  UWBSolver.solve()           ← 加权最小二乘三边测量
         │                      4+ 锚点 → 3D 解算
         │                      3 锚点 → 2D 解算 (Z 固定 DEFAULT_HEIGHT)
         │                      Cramer 法则解 3×3 / 2×2 线性方程组
         │
         ▼
  PositionFilter.filter()     ← x, y, z 8帧滑动窗口中位数
         │
         ▼
  有限差分 → vx, vy, vz       ← (当前滤波位置 - 上一帧滤波位置) / dt
         │
         ├─→ USART1 printf    ← 调试终端: [时间] x=... y=... z=... vx=... vy=...
         │
         ▼
  NMEA_Generator.generate()   ← UWB(x,y,z) → GPS(E7) → NMEA ddmm.mmmm
         │                       速度 m/s → knots
         │                       航向 atan2(vx, vy) → 真北角度
         │
         ▼
  USART3 TX (DMA)             ← $GPGGA + $GPRMC, 5Hz
         │
         ▼
  飞控 GPS 口                 ← SERIALx_PROTOCOL=5
```

### 与 Python 版差异

| 特性 | Python 版 (uwb.py) | STM32 版 |
|------|-------------------|----------|
| 滤波 | 6 状态 EKF (自适应噪声) | 双重中值滤波 |
| 标定 | 交互式 3 模式菜单 | 硬编码坐标 |
| 输出 | 终端 + UDP JSON + NMEA | printf 调试 + NMEA |
| 串口接收 | pyserial 阻塞读取 | DMA + 空闲中断环形缓冲 |
| NMEA 发送 | pyserial 阻塞写入 | DMA 非阻塞发送 |

## 6. Arduino → STM32 HAL API 映射

| Arduino API | STM32 HAL 等价调用 |
|-------------|-------------------|
| `Serial.available()` | `__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)` |
| `Serial.read()` | `HAL_UART_Receive(&huart2, &c, 1, 0)` |
| `millis()` | `HAL_GetTick()` |
| `Serial.begin(baud)` | `HAL_UART_Init()` (由 CubeMX 生成) |
| `Serial.print(str)` | `printf(str)` (重定向到 USART1) |
| `Serial.println(str)` | `printf("%s\r\n", str)` |
| `delay(ms)` | `HAL_Delay(ms)` |
| `constrain(v, lo, hi)` | 自定义宏 `#define CONSTRAIN(v,l,h) ((v)<(l)?(l):((v)>(h)?(h):(v)))` |

### `uwb_solver.h` / `uwb_nmea.h` 头文件替换

删除 `#include <Arduino.h>`，替换为：

```c
#include <math.h>      // fabsf, sqrtf, atan2f, cosf, sinf
#include <string.h>    // strlen, strncmp, strchr
#include <stdio.h>     // snprintf
#include <stdint.h>    // int32_t, uint8_t
```

### printf 重定向

`printf` 通过重写 `__io_putchar()` 或 `_write()` 重定向到 USART1 (PA9 TX)：

```c
int _write(int fd, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
```

不需要半主机模式，纯硬件 UART 输出，调试时 USB-TTL 接 PC 即可。

### USART2 DMA 接收配置

- DMA1 Channel6 (USART2_RX)，循环模式
- 环形缓冲区大小 `RX2_BUF_SIZE = 256` 字节
- 空闲中断 (`UART_IT_IDLE`) 触发帧结束，记录 `rx2_len` 并置 `rx2_ready`

## 7. 配置参数 (`uwb_config.h`)

```c
// ---- 锚点 ----
#define ANCHOR_COUNT        4
const float ANCHOR_POSITIONS[ANCHOR_COUNT][3] = {
    {0.0f, 0.0f, 1.5f},   // S1 — 场地原点
    {5.0f, 0.0f, 1.5f},   // S2 — X轴方向
    {0.0f, 5.0f, 1.5f},   // S3 — Y轴方向
    {5.0f, 5.0f, 1.5f},   // S4 — 对角
};

// ---- 求解器 ----
#define DEFAULT_HEIGHT      1.0f     // 3锚点降级模式 Z 固定值
#define POS_CLAMP_MIN      -10.0f
#define POS_CLAMP_MAX       10.0f
#define RESIDUAL_MAX_3D     2.0f
#define RESIDUAL_MAX_2D     1.5f

// ---- 滤波 ----
#define DIST_WINDOW_SIZE    8
#define MAX_DIST_JUMP       0.8f

// ---- NMEA ----
#define GPS_ORIGIN_LAT      321148408   // 场地参考原点纬度 (E7)
#define GPS_ORIGIN_LON      1189590664  // 场地参考原点经度 (E7)
#define GPS_ORIGIN_ALT      1200        // 场地参考原点海拔 (cm)
#define NMEA_RATE_HZ        5.0f

// ---- 主循环 ----
#define LOOP_INTERVAL_MS    20
```

## 8. 飞控参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `SERIALx_PROTOCOL` | `5` | GPS 协议 |
| `SERIALx_BAUD` | `57` (57600) | 匹配 STM32 NMEA 输出 |
| `GPS_TYPE` | `1` (AUTO) | 自动识别 NMEA |

## 9. 部署步骤

1. **量测锚点坐标**：卷尺测量场地中 4 个锚点的 (X, Y, Z) 位置
2. **编辑 `uwb_config.h`**：填入 `ANCHOR_POSITIONS` 和场地 `GPS_ORIGIN_*`
3. **CubeMX 配置**：打开 `stm32_uwb.ioc`，确认引脚分配和时钟 (72MHz HSE)
4. **编译**：STM32CubeIDE → Build (Ctrl+B)
5. **烧录**：ST-Link V2 连接 PA13/PA14/GND/3.3V → Run (F11)
6. **验证**：
   - PC 串口工具连接 USART1 (115200 baud)，应看到 `x=... y=... z=...` 定位输出
   - 飞控地面站应显示 GPS 定位（来自 NMEA 串口）
   - 移动 UWB 标签，地面站位置跟随变化

## 10. 改动范围

| 文件 | 操作 | 改动量 |
|------|------|--------|
| `Core/Src/main.c` | 新建 | ~150 行 |
| `Core/Inc/uwb_config.h` | 从 `uwb_arduino/` 复制并改值 | ~5 行 |
| `Core/Inc/uwb_solver.h` | 从 `uwb_arduino/` 复制，替换 `#include <Arduino.h>` | ~5 行 |
| `Core/Inc/uwb_nmea.h` | 从 `uwb_arduino/` 复制，替换 `#include <Arduino.h>` | ~5 行 |
| `stm32_uwb.ioc` | CubeMX 新建 | 图形化配置 |

总新增代码量约 150 行 (`main.c`)，其余 650 行算法代码直接从 Arduino 版复用。
