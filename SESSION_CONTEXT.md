# STM32 UWB 项目状态 — 新对话快速恢复上下文

> **使用后请删除此文件。** 飞控必须在 NMEA 4 位小数分下才能 fix_type=3。

## 当前状态

| 项目 | 路径 | 版本 | 状态 |
|------|------|------|------|
| **stm32_uwb** (活跃) | `stm32_uwb/` | EKF + WLSQ NMEA 4位 | 编译后待烧录验证 fix_type=3 |
| stm32_uwb_ekf (独立) | `stm32_uwb_ekf/` | `3afd2a5` | 独立 EKF 项目快照 |
| stm32_uwb_v4 (备份) | `stm32_uwb_v4/` | v4 | UWB代码 文件备份 |
| UWBcode (源) | `C:\Users\cheng\Desktop\homework\无人机\UWBcode\` | v5 | 4文件: main,ekf,nmea.c/h |

## 当前文件构成 (`stm32_uwb/Core/`)

### Src/
| 文件 | 作用 | 来源 |
|------|------|------|
| `main.c` | EKF 主循环 + WLSQ NMEA(9参数调用) | `cf73887` |
| `uwb_ekf.c` | CV模型EKF, q_acc_xy=1.5, set_fixed_z | UWBcode |
| `uwb_nmea.c` | **4位小数分 NMEA**（float32转换, 无GSA） | `cf73887` |
| `uwb_solver.c` | 仅提供 UWB_Parser（$DIST解析） | WLSQ |
| `uwb_nmea.c/h` → 注意: nmea_gen_generate 是9参数, 无fix_quality/sats/hdop | | |
| `stm32f1xx_it.c` | 含 USART3_IRQHandler | |
| `usart.c` | USART2=115200, USART3=57600, USART3 IRQ使能 | |

### Inc/
| 文件 | 作用 |
|------|------|
| `uwb_ekf.h` | EKF 声明, 含 uwb_ekf_set_fixed_z |
| `uwb_solver.h` | UWB_Parser 结构体 + 函数声明 |
| `uwb_nmea.h` | NMEA_Generator(ggpa[100],rmc[110],vtg[100]), local_to_gps |
| `uwb_config.h` | ANCHOR_POSITIONS, DEFAULT_HEIGHT=1.0, 旧滤波参数（EKF不用） |

## 编译前检查清单

1. main.c 中 `#include "uwb_ekf.h"` ✅
2. main.c 中 `nmea_gen_generate` 是 **9 参数** (无 fix_quality/sats/hdop)
3. main.c 中 `nmea_start_dma_send` 是 **3 参数** (ggpa, rmc, vtg——无 GSA)
4. 编译列表含 `uwb_solver.c` + `uwb_ekf.c`
5. uwb_nmea.c 用 **4 位小数分** (`%04d` 格式, float32 转换)

## 关键发现

- **fix_type=3** 需要 NMEA 小数分 **4 位** (ddmm.mmmm)
- 5 位和 6 位小数分 → fix_type=1 (NO_FIX)
- WLSQ 版(4位)验证通过, EKF 版用 v3 int64(6位)失败, 改用5位也失败

## 锚点坐标

```
S1(0,0,1.0)  S2(5,0,1.5)
S3(0,5,1.5)  S4(5,5,1.0)
```
UWB 模块 115200 baud → PA3(USART2_RX)

## 烧录命令

```bash
"/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe" -c port=COM11 br=115200 P=EVEN -d "C:/Users/cheng/Desktop/homework/无人机/main/stm32_uwb/Debug/stm32_uwb.hex" -v
```

## 引脚

| 引脚 | 功能 | 波特率 |
|------|------|--------|
| PA9 | USART1_TX → 串口调试 | 115200 |
| PA3 | USART2_RX ← UWB模块 | 115200 |
| PB10 | USART3_TX → 飞控GPS口 | 57600 |

## GPS 验证

飞控参数: `SERIALx_BAUD=57`, `GPS_TYPE=5`
MAVLink Inspector → `GPS_RAW_INT.fix_type` → 目标值 **3**

## Git 关键提交

```
2cc2eaa  UWBcode v5 (5位+GSA, 待验证)
cf73887  WLSQ恢复版 (4位NMEA, fix_type=3 ✅)
84ea77f  WLSQ最后稳定版
5990b60  α-β准确版(轨迹正常)
28302b0  TDMA优化版(距离直通)
```
