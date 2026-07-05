/**
 * uwb_nmea.h — NMEA 0183 GPS 语句生成器
 *
 * 功能:
 *   1. 将 UWB 局部坐标 (x,y,z) 转换为 GPS 经纬度
 *   2. 生成 $GPGGA + $GPRMC 标准 NMEA 语句
 *   3. 可选: 通过串口发送到飞控 GPS 端口
 *
 * 移植自 Python 版 uwb.py 的 local_to_gps + _deg_to_nmea + _nmea_checksum
 */

#ifndef UWB_NMEA_H
#define UWB_NMEA_H

#include <Arduino.h>
#include "uwb_config.h"

class NMEA_Generator {
public:
  NMEA_Generator() {
    _ggpa[0] = '\0';
    _rmc[0] = '\0';
  }

  /**
   * UWB 局部坐标 → GPS 经纬度 (E7 格式)
   *
   * @param x, y, z      UWB 局部坐标 (米): X=东, Y=北, Z=上
   * @param origin_lat   GPS 参考原点纬度 (E7 格式)
   * @param origin_lon   GPS 参考原点经度 (E7 格式)
   * @param origin_alt   GPS 参考原点海拔 (厘米)
   * @param out_lat      输出纬度 (E7)
   * @param out_lon      输出经度 (E7)
   * @param out_alt_mm   输出海拔 (毫米)
   */
  static void localToGPS(float x, float y, float z,
                         int32_t origin_lat, int32_t origin_lon, int32_t origin_alt,
                         int32_t &out_lat, int32_t &out_lon, int32_t &out_alt_mm)
  {
    float lat_deg = origin_lat / 1e7f;
    float lat_rad = lat_deg * DEG_TO_RAD;

    const float m_per_deg_lat = 111319.9f;
    float m_per_deg_lon = 111319.9f * cosf(lat_rad);

    // 限幅
    y = constrain(y, -20.0f, 20.0f);
    x = constrain(x, -20.0f, 20.0f);
    z = constrain(z, 0.0f, 10.0f);

    out_lat = origin_lat + (int32_t)(y * 1e7f / m_per_deg_lat);
    out_lon = origin_lon + (int32_t)(x * 1e7f / m_per_deg_lon);
    out_alt_mm = origin_alt * 10 + (int32_t)(z * 1000.0f);
  }

  /**
   * 生成一组 $GPGGA + $GPRMC 语句
   *
   * @param x, y, z    UWB 局部坐标 (米)
   * @param vx, vy     UWB 速度 (m/s)
   * @param origin_lat/lon/alt  GPS 参考原点
   */
  void generate(float x, float y, float z,
                float vx, float vy,
                int32_t origin_lat, int32_t origin_lon, int32_t origin_alt)
  {
    // Step 1: UWB → GPS
    int32_t lat_e7, lon_e7, alt_mm;
    localToGPS(x, y, z, origin_lat, origin_lon, origin_alt, lat_e7, lon_e7, alt_mm);
    float alt_m = alt_mm / 1000.0f;

    // Step 2: E7 → NMEA ddmm.mmmm
    float lat_deg = lat_e7 / 1e7f;
    float lon_deg = lon_e7 / 1e7f;

    char lat_str[16], lon_str[16];
    degToNMEA_Lat(lat_deg, lat_str);
    degToNMEA_Lon(lon_deg, lon_str);

    // Step 3: 时间 (millis 模拟, 实际应用可接 RTC)
    unsigned long now = millis() / 1000UL;
    int h = (now / 3600) % 24;
    int m = (now / 60) % 60;
    int s = now % 60;

    char time_str[10];
    sprintf(time_str, "%02d%02d%02d", h, m, s);

    // 日期 (编译日期)
    int day = (__DATE__[4] == ' ' ? 0 : (__DATE__[4]-'0')*10) + (__DATE__[5]-'0');
    int month = monthNum(__DATE__);
    int year = (__DATE__[9]-'0')*10 + (__DATE__[10]-'0');
    char date_str[8];
    sprintf(date_str, "%02d%02d%02d", day, month, year);

    // Step 4: 速度 + 航向
    float speed_kn = sqrtf(vx*vx + vy*vy) * 1.94384f;
    float course = atan2f(vx, vy) * RAD_TO_DEG;
    if (course < 0) course += 360.0f;

    // Step 5: 组装 $GPGGA
    // $GPGGA,time,lat,N,lon,E,qual,sats,hdop,alt,M,geoid,M,,*CK
    snprintf(_ggpa, sizeof(_ggpa),
             "$GPGGA,%s.00,%s,%s,1,08,1.0,%.1f,M,0.0,M,,*",
             time_str, lat_str, lon_str, alt_m);
    checksumAppend(_ggpa);

    // Step 6: 组装 $GPRMC
    // $GPRMC,time,status,lat,N,lon,E,speed,course,date,,,mode*CK
    snprintf(_rmc, sizeof(_rmc),
             "$GPRMC,%s.00,A,%s,%s,%.2f,%.1f,%s,,,A*",
             time_str, lat_str, lon_str, speed_kn, course, date_str);
    checksumAppend(_rmc);
  }

  const char* getGGPA() const { return _ggpa; }
  const char* getRMC()  const { return _rmc; }

  void sendTo(HardwareSerial &ser) {
    ser.print(_ggpa);
    ser.print(_rmc);
  }

private:
  char _ggpa[100];
  char _rmc[100];

  // ---- NMEA 格式转换 ----

  /**
   * 纬度: 十进制度 → "DDMM.MMMM,N/S"
   */
  static void degToNMEA_Lat(float deg, char* out) {
    float a = fabsf(deg);
    int d = (int)a;
    float mm = (a - d) * 60.0f;
    char dir = (deg >= 0) ? 'N' : 'S';
    snprintf(out, 16, "%02d%07.4f,%c", d, mm, dir);
  }

  /**
   * 经度: 十进制度 → "DDDMM.MMMM,E/W"
   */
  static void degToNMEA_Lon(float deg, char* out) {
    float a = fabsf(deg);
    int d = (int)a;
    float mm = (a - d) * 60.0f;
    char dir = (deg >= 0) ? 'E' : 'W';
    snprintf(out, 16, "%03d%07.4f,%c", d, mm, dir);
  }

  /**
   * NMEA 异或校验和
   */
  static uint8_t xorChecksum(const char* s) {
    uint8_t ck = 0;
    for (int i = 1; s[i] && s[i] != '*'; i++) ck ^= (uint8_t)s[i];
    return ck;
  }

  /**
   * 在校验和占位符 * 之后追加 CK\r\n
   */
  static void checksumAppend(char* buf) {
    uint8_t ck = xorChecksum(buf);
    int len = strlen(buf);
    snprintf(buf + len, 6, "%02X\r\n", ck);
  }

  static int monthNum(const char* dateStr) {
    if (strncmp(dateStr, "Jan", 3) == 0) return 1;
    if (strncmp(dateStr, "Feb", 3) == 0) return 2;
    if (strncmp(dateStr, "Mar", 3) == 0) return 3;
    if (strncmp(dateStr, "Apr", 3) == 0) return 4;
    if (strncmp(dateStr, "May", 3) == 0) return 5;
    if (strncmp(dateStr, "Jun", 3) == 0) return 6;
    if (strncmp(dateStr, "Jul", 3) == 0) return 7;
    if (strncmp(dateStr, "Aug", 3) == 0) return 8;
    if (strncmp(dateStr, "Sep", 3) == 0) return 9;
    if (strncmp(dateStr, "Oct", 3) == 0) return 10;
    if (strncmp(dateStr, "Nov", 3) == 0) return 11;
    if (strncmp(dateStr, "Dec", 3) == 0) return 12;
    return 1;
  }
};

#endif // UWB_NMEA_H
