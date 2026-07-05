/**
 * uwb_config.h — UWB 室内定位 Arduino 配置
 *
 * 使用前请根据实际场地修改以下参数：
 *   1. ANCHOR_POSITIONS — 锚点坐标 (运行 Python 版 --calibrate 标定后填入)
 *   2. ANCHOR_COUNT — 锚点数量
 *   3. GPS_ORIGIN_LAT/LON/ALT — GPS 参考原点 (NMEA 模式需要)
 *   4. DEFAULT_HEIGHT — 3 锚点降级模式下的固定高度
 */

#ifndef UWB_CONFIG_H
#define UWB_CONFIG_H

// ============================================================================
// 1. 串口配置
// ============================================================================

// ---- 平台自动检测 ----
// Nano 33 BLE Sense (nRF52840): 仅 1 个硬件 UART → UWB 用 SoftwareSerial
// ESP32 / ESP32-S3 (Nano ESP32): 多硬件 UART → UWB 用 Serial1, GPS 用 Serial0
// 注意: ESP32-S3 的 Serial2 默认引脚 (GPIO19/20) 与 USB 冲突，必须避开
#if defined(ARDUINO_ARCH_NRF52840)
  #define UWB_USE_SOFTWARE_SERIAL
  #define UWB_RX_PIN         2       // D2 ← UWB 模块 TX
  #define UWB_TX_PIN         3       // D3 → UWB 模块 RX
  #define UWB_SERIAL_BAUD    19200
#elif defined(ESP32)
  #define UWB_SERIAL_PORT    Serial1
  #define UWB_RX_PIN         5       // D2 (GPIO5) ← UWB 模块 TX
  #define UWB_TX_PIN         6       // D3 (GPIO6) → UWB 模块 RX
  #define UWB_SERIAL_BAUD    19200
#else
  #define UWB_SERIAL_PORT    Serial2
  #define UWB_SERIAL_BAUD    19200
#endif

// 调试输出串口 (USB)
#define DEBUG_SERIAL_PORT  Serial
#define DEBUG_SERIAL_BAUD  115200

// ============================================================================
// 2. 锚点位置 (关键参数!)
// ============================================================================

// 锚点数量 (3 个 = 2D 降级模式, 4+ = 全 3D 模式)
#define ANCHOR_COUNT 4

// 锚点坐标 (单位: 米) — 按 S1, S2, S3, S4 顺序
// 坐标系: X=东, Y=北, Z=上
// 运行 Python 版 uwb.py --calibrate 标定后把结果填入这里
const float ANCHOR_POSITIONS[ANCHOR_COUNT][3] = {
  {0.0f, 0.0f, 1.0f},   // S1
  {5.0f, 0.0f, 1.0f},   // S2
  {0.0f, 5.0f, 1.0f},   // S3
  {5.0f, 5.0f, 1.0f},   // S4
};

// ============================================================================
// 3. 求解器参数
// ============================================================================

// 3 锚点降级模式下的固定 Z 高度 (米)
#define DEFAULT_HEIGHT    1.0f

// 坐标范围约束 (米) — 防止解算飞出场地
#define POS_CLAMP_MIN     -10.0f
#define POS_CLAMP_MAX      10.0f

// 残差阈值 — 超过则丢弃当前解 (3D模式)
#define RESIDUAL_MAX_3D   2.0f

// 残差阈值 — 2D模式
#define RESIDUAL_MAX_2D   1.5f

// ============================================================================
// 4. 距离滤波参数
// ============================================================================

// 滑动窗口大小
#define DIST_WINDOW_SIZE  8

// 单帧测距最大允许跳变 (米) — 超过判定为野值
#define MAX_DIST_JUMP     0.8f

// ============================================================================
// 5. 输出配置
// ============================================================================

// 终端输出开关 (通过主串口打印位置)
#define OUTPUT_TERMINAL   true

// NMEA GPS 模拟输出
// 启用后会在 GPS_SERIAL_PORT 上发送 $GPGGA + $GPRMC
#define NMEA_ENABLED      true

// NMEA 输出串口 (连接到飞控 GPS 端口)
// Nano ESP32: Serial0 = D1(TX=GPIO43) | Nano 33 BLE: Serial1 = D1(TX)
#if defined(ESP32)
  #define GPS_SERIAL_PORT   Serial0
#else
  #define GPS_SERIAL_PORT   Serial1
#endif
#define GPS_SERIAL_BAUD   57600

// NMEA 输出频率 (Hz)
#define NMEA_RATE_HZ      5.0f

// ============================================================================
// 6. GPS 参考原点 (E7 格式, NMEA 模式需要)
// ============================================================================

// 南京大学仙林校区 (示例, 请替换为实际场地坐标)
#define GPS_ORIGIN_LAT    321148408   // 32.1148408°
#define GPS_ORIGIN_LON    1189590664  // 118.9590664°
#define GPS_ORIGIN_ALT    1200        // 120.0m (厘米)

// ============================================================================
// 7. 主循环频率
// ============================================================================

// 主循环目标周期 (毫秒)
#define LOOP_INTERVAL_MS  20   // 50Hz

// UWB 读取超时 (毫秒)
#define UWB_READ_TIMEOUT_MS  2000

#endif // UWB_CONFIG_H
