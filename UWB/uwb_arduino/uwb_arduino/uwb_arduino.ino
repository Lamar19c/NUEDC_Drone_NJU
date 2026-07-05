/**
 * uwb_arduino.ino — UWB 室内 3D 定位系统 (Arduino 移植版)
 *
 * ===== 硬件连线 =====
 *   Nano 33 BLE Sense (ABX00083):
 *     D2 ← UWB 模块 TX    (SoftwareSerial RX, 接收 $DIST 数据)
 *     D1 → 飞控 GPS RX   (Serial1 TX, NMEA 输出)
 *     GND ↔ 飞控 GND + UWB GND
 *     ⚠️ 3.3V 逻辑电平! 5V 设备需电平转换
 *
 *   ESP32:
 *     GPIO16 (RX2) ← UWB 模块 TX
 *     GPIO17 (TX2) → UWB 模块 RX
 *     GPIO18 (TX1) → 飞控 GPS RX
 *
 * ===== 串口映射 =====
 *   Serial  (USB)   — 调试输出 (115200 baud)
 *   Serial1 (UART)  — NMEA GPS 模拟 → 飞控 GPS 口 (57600 baud)
 *   SoftwareSerial / Serial2 — UWB 模块数据接收 (19200 baud)
 *
 * ===== 滤波器架构 =====
 *   1. 距离中值滤波 (DistanceFilter)  — 逐锚点滑动窗口中位数 + 跳变限幅
 *   2. 位置中值滤波 (PositionFilter) — 三边测量解算后对 x,y,z 再做中值平滑
 *   3. 速度 = 相邻帧有限差分
 *
 * ===== 使用步骤 =====
 *   1. 编辑 uwb_config.h: 设置 ANCHOR_POSITIONS, GPS_ORIGIN_*
 *   2. 编译上传 (开发板选 Arduino Nano 33 BLE 或 ESP32)
 *   3. 打开串口监视器 (115200 baud) 查看定位输出
 *   4. 飞控端设置 SERIALx_PROTOCOL=5 (GPS) 即可接收 NMEA
 *
 * 移植自 uwb.py v2.0
 */

#include <Arduino.h>
#include "uwb_config.h"        // ← 必须在 SoftwareSerial 判断之前!

// ---- SoftwareSerial (仅 nRF52840 平台, 由 uwb_config.h 中的 UWB_USE_SOFTWARE_SERIAL 控制) ----
#ifdef UWB_USE_SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial uwbSoftSerial(UWB_RX_PIN, UWB_TX_PIN);
  #define UWB_SERIAL  uwbSoftSerial
#else
  #define UWB_SERIAL  UWB_SERIAL_PORT
#endif

#include "uwb_solver.h"
#include "uwb_nmea.h"

// ============================================================================
// 全局对象
// ============================================================================

UWBSolver* solver = nullptr;
UWB_Parser uwbParser;
DistanceFilter distFilter;
NMEA_Generator nmea;

unsigned long lastLoopTime = 0;
unsigned long lastNmeaTime = 0;
unsigned long loopCount = 0;
bool solverInitialized = false;

// ============================================================================
// 初始化
// ============================================================================

void setup() {
  // 1. 调试串口
  DEBUG_SERIAL_PORT.begin(DEBUG_SERIAL_BAUD);
  delay(500);
  DEBUG_SERIAL_PORT.println();
  DEBUG_SERIAL_PORT.println(F("========================================"));
  DEBUG_SERIAL_PORT.println(F("  UWB 室内 3D 定位 — Arduino 版"));
  DEBUG_SERIAL_PORT.println(F("  (中值滤波)"));
  DEBUG_SERIAL_PORT.println(F("========================================"));

  // 2. UWB 模块串口
#ifdef UWB_USE_SOFTWARE_SERIAL
  DEBUG_SERIAL_PORT.print(F("UWB 串口: SoftwareSerial (D"));
  DEBUG_SERIAL_PORT.print(UWB_RX_PIN);
  DEBUG_SERIAL_PORT.print(F("←UWB TX) @ "));
  DEBUG_SERIAL_PORT.println(UWB_SERIAL_BAUD);
#elif defined(ESP32)
  DEBUG_SERIAL_PORT.print(F("UWB 串口: Serial1 (D"));
  DEBUG_SERIAL_PORT.print(UWB_RX_PIN);
  DEBUG_SERIAL_PORT.print(F("←UWB TX) @ "));
  DEBUG_SERIAL_PORT.println(UWB_SERIAL_BAUD);
  UWB_SERIAL.begin(UWB_SERIAL_BAUD, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);
#else
  DEBUG_SERIAL_PORT.print(F("UWB 串口: Serial2 @ "));
  DEBUG_SERIAL_PORT.println(UWB_SERIAL_BAUD);
  UWB_SERIAL.begin(UWB_SERIAL_BAUD);
#endif

  // 3. NMEA GPS 模拟串口
#if NMEA_ENABLED
  DEBUG_SERIAL_PORT.print(F("NMEA 输出: "));
#if defined(ESP32)
  DEBUG_SERIAL_PORT.print(F("Serial0 (D1→FC RX) @ "));
#else
  DEBUG_SERIAL_PORT.print(F("Serial1 (D1→FC RX) @ "));
#endif
  DEBUG_SERIAL_PORT.println(GPS_SERIAL_BAUD);
  GPS_SERIAL_PORT.begin(GPS_SERIAL_BAUD);
#endif

  // 4. 初始化求解器 (纯三边测量 + 中值滤波, 无 EKF)
  solver = new UWBSolver(ANCHOR_POSITIONS, ANCHOR_COUNT);
  solverInitialized = true;

  // 5. 打印配置
  DEBUG_SERIAL_PORT.print(F("锚点数量: "));
  DEBUG_SERIAL_PORT.println(ANCHOR_COUNT);
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    DEBUG_SERIAL_PORT.print(F("  S"));
    DEBUG_SERIAL_PORT.print(i + 1);
    DEBUG_SERIAL_PORT.print(F(": ("));
    DEBUG_SERIAL_PORT.print(ANCHOR_POSITIONS[i][0], 2);
    DEBUG_SERIAL_PORT.print(F(", "));
    DEBUG_SERIAL_PORT.print(ANCHOR_POSITIONS[i][1], 2);
    DEBUG_SERIAL_PORT.print(F(", "));
    DEBUG_SERIAL_PORT.print(ANCHOR_POSITIONS[i][2], 2);
    DEBUG_SERIAL_PORT.println(F(")"));
  }

  if (ANCHOR_COUNT == 3) {
    DEBUG_SERIAL_PORT.print(F("3 锚点模式: Z 固定 = "));
    DEBUG_SERIAL_PORT.print(DEFAULT_HEIGHT);
    DEBUG_SERIAL_PORT.println(F("m"));
  }

  DEBUG_SERIAL_PORT.print(F("GPS 原点 (E7): lat="));
  DEBUG_SERIAL_PORT.print(GPS_ORIGIN_LAT);
  DEBUG_SERIAL_PORT.print(F(" lon="));
  DEBUG_SERIAL_PORT.print(GPS_ORIGIN_LON);
  DEBUG_SERIAL_PORT.print(F(" alt="));
  DEBUG_SERIAL_PORT.println(GPS_ORIGIN_ALT);

#ifdef UWB_USE_SOFTWARE_SERIAL
  DEBUG_SERIAL_PORT.println(F("⚠️ 3.3V 电平 — 5V 飞控需电平转换"));
#endif

  DEBUG_SERIAL_PORT.println(F("等待 UWB 数据..."));
  DEBUG_SERIAL_PORT.println();

  lastLoopTime = millis();
}

// ============================================================================
// 主循环
// ============================================================================

void loop() {
  unsigned long now = millis();

  // ---- 缓存的最后位置 (无 UWB 时 NMEA 仍能输出) ----
  static float gx = 0, gy = 0, gz = 1.0f, gvx = 0, gvy = 0;
  static bool hasPos = false;

  // ---- Step 1: 读取 UWB 数据 ----
  while (UWB_SERIAL.available() > 0) {
    char c = UWB_SERIAL.read();
    uwbParser.feed(c);
  }

  // ---- Step 2: 有数据则求解 ----
  if (uwbParser.validCount() >= 3) {
    float distances[ANCHOR_COUNT];
    uwbParser.getDistances(distances, ANCHOR_COUNT);
    uwbParser.clearDistances();

    // 距离中值滤波
    distFilter.filter(distances, ANCHOR_COUNT);

    float x, y, z, vx, vy, vz;
    if (solverInitialized) {
      // 三边测量 → 位置中值滤波 → 有限差分速度
      bool ok = solver->solve(distances, now, x, y, z, vx, vy, vz);
      if (ok) {
        gx = x; gy = y; gz = z; gvx = vx; gvy = vy;
        hasPos = true;
        loopCount++;

#if OUTPUT_TERMINAL
        int32_t lat_e7, lon_e7, alt_mm;
        NMEA_Generator::localToGPS(x, y, z,
            GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
            lat_e7, lon_e7, alt_mm);

        DEBUG_SERIAL_PORT.print(F("["));
        DEBUG_SERIAL_PORT.print(millis() / 1000.0f, 1);
        DEBUG_SERIAL_PORT.print(F("] x="));
        DEBUG_SERIAL_PORT.print(x, 2);
        DEBUG_SERIAL_PORT.print(F(" y="));
        DEBUG_SERIAL_PORT.print(y, 2);
        DEBUG_SERIAL_PORT.print(F(" z="));
        DEBUG_SERIAL_PORT.print(z, 2);
        DEBUG_SERIAL_PORT.print(F("  vx="));
        DEBUG_SERIAL_PORT.print(vx, 2);
        DEBUG_SERIAL_PORT.print(F(" vy="));
        DEBUG_SERIAL_PORT.print(vy, 2);
        DEBUG_SERIAL_PORT.print(F(" vz="));
        DEBUG_SERIAL_PORT.print(vz, 2);
        DEBUG_SERIAL_PORT.print(F("  GPS: "));
        DEBUG_SERIAL_PORT.print(lat_e7 / 1e7f, 7);
        DEBUG_SERIAL_PORT.print(F(","));
        DEBUG_SERIAL_PORT.print(lon_e7 / 1e7f, 7);
        DEBUG_SERIAL_PORT.print(F(","));
        DEBUG_SERIAL_PORT.print(alt_mm / 1000.0f, 2);
        DEBUG_SERIAL_PORT.println(F("m"));
#endif
      }
    }
  }

  // ---- Step 3: NMEA GPS 模拟 (始终运行, 无 UWB 时用原点) ----
  static bool nmeaFirst = true;
#if NMEA_ENABLED
  float nmeaInterval = 1000.0f / NMEA_RATE_HZ;
  if (now - lastNmeaTime >= (unsigned long)nmeaInterval) {
    lastNmeaTime = now;

    nmea.generate(gx, gy, gz, gvx, gvy,
                  GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT);
    nmea.sendTo(GPS_SERIAL_PORT);

    static int nmeaCount = 0;
    nmeaCount++;
    if (nmeaFirst && nmeaCount == 1) {
      nmeaFirst = false;
      DEBUG_SERIAL_PORT.println(F("\n  NMEA 首条已发送 (D1):"));
      DEBUG_SERIAL_PORT.print(F("    "));
      DEBUG_SERIAL_PORT.println(nmea.getGGPA());
      DEBUG_SERIAL_PORT.print(F("    "));
      DEBUG_SERIAL_PORT.println(nmea.getRMC());
      DEBUG_SERIAL_PORT.println();
    }
    if (nmeaCount % (int(NMEA_RATE_HZ) * 5) == 0) {
      DEBUG_SERIAL_PORT.print(F("  NMEA "));
      DEBUG_SERIAL_PORT.print(nmeaCount);
      DEBUG_SERIAL_PORT.print(F(" 组 → D1 "));
      DEBUG_SERIAL_PORT.print(hasPos ? F("[UWB]") : F("[原点-无UWB]"));
      DEBUG_SERIAL_PORT.println();
    }
  }
#endif

  delay(LOOP_INTERVAL_MS);
}
