/**
 * uwb_nmea.h — NMEA 0183 GPS 语句生成器
 *
 * UWB 局部坐标 → GPS(E7) → NMEA ddmm.mmmm → $GPGGA + $GPRMC + $GPVTG
 * 从 uwb_arduino/uwb_nmea.h 改写 — C++ class → C struct + .c
 *
 * v2: 所有 snprintf 使用 %d 整数格式，不依赖 newlib-nano 浮点 printf 支持。
 */
#ifndef UWB_NMEA_H
#define UWB_NMEA_H

#include "uwb_config.h"
#include <stdint.h>    /* int32_t, uint32_t */

/* ========================================================================
 * NMEA_Generator
 * ======================================================================== */

struct NMEA_Generator {
    char ggpa[100];
    char rmc[110];   /* GCC 估计最坏 103 字节，留余量 */
    char vtg[100];
};

void nmea_gen_init(struct NMEA_Generator *n);

void nmea_gen_generate(struct NMEA_Generator *n,
                       float x, float y, float z,
                       float vx, float vy,
                       int32_t origin_lat, int32_t origin_lon, int32_t origin_alt,
                       uint32_t now_sec);

const char* nmea_gen_ggpa(const struct NMEA_Generator *n);
const char* nmea_gen_rmc(const struct NMEA_Generator *n);
const char* nmea_gen_vtg(const struct NMEA_Generator *n);

/* ========================================================================
 * local_to_gps — UWB 坐标 → GPS (E7)
 * ======================================================================== */

void local_to_gps(float x, float y, float z,
                  int32_t origin_lat, int32_t origin_lon, int32_t origin_alt,
                  int32_t *out_lat, int32_t *out_lon, int32_t *out_alt_mm);

#endif /* UWB_NMEA_H */
