/**
 * uwb_nmea.h — NMEA 0183 GPS 语句生成器 (纯 C)
 *
 * UWB 局部坐标 → GPS(E7) → NMEA ddmm.mmmm → $GPGGA + $GPRMC
 * 从 uwb_arduino/uwb_nmea.h 改写 — C++ class → C struct
 */
#ifndef UWB_NMEA_H
#define UWB_NMEA_H

#include "uwb_config.h"
#include <math.h>      /* fabsf, sqrtf, atan2f, cosf */
#include <string.h>    /* strlen, strncmp */
#include <stdio.h>     /* snprintf */
#include <stdint.h>    /* int32_t, uint8_t */

/* ========================================================================
 * 1. 格式转换 helpers
 * ======================================================================== */

static void deg_to_nmea_lat(float deg, char *out) {
    float a = fabsf(deg);
    int d = (int)a;
    float mm = (a - (float)d) * 60.0f;
    char dir = (deg >= 0.0f) ? 'N' : 'S';
    snprintf(out, 16, "%02d%07.4f,%c", d, mm, dir);
}

static void deg_to_nmea_lon(float deg, char *out) {
    float a = fabsf(deg);
    int d = (int)a;
    float mm = (a - (float)d) * 60.0f;
    char dir = (deg >= 0.0f) ? 'E' : 'W';
    snprintf(out, 16, "%03d%07.4f,%c", d, mm, dir);
}

static uint8_t nmea_xor_checksum(const char *s) {
    uint8_t ck = 0;
    for (int i = 1; s[i] && s[i] != '*'; i++) ck ^= (uint8_t)s[i];
    return ck;
}

static void nmea_checksum_append(char *buf) {
    uint8_t ck = nmea_xor_checksum(buf);
    int len = (int)strlen(buf);
    snprintf(buf + len, 6, "%02X\r\n", ck);
}

static int month_num(const char *date_str) {
    if      (strncmp(date_str, "Jan", 3) == 0) return 1;
    else if (strncmp(date_str, "Feb", 3) == 0) return 2;
    else if (strncmp(date_str, "Mar", 3) == 0) return 3;
    else if (strncmp(date_str, "Apr", 3) == 0) return 4;
    else if (strncmp(date_str, "May", 3) == 0) return 5;
    else if (strncmp(date_str, "Jun", 3) == 0) return 6;
    else if (strncmp(date_str, "Jul", 3) == 0) return 7;
    else if (strncmp(date_str, "Aug", 3) == 0) return 8;
    else if (strncmp(date_str, "Sep", 3) == 0) return 9;
    else if (strncmp(date_str, "Oct", 3) == 0) return 10;
    else if (strncmp(date_str, "Nov", 3) == 0) return 11;
    else if (strncmp(date_str, "Dec", 3) == 0) return 12;
    return 1;
}

/* ========================================================================
 * 2. localToGPS — UWB 坐标 → GPS (E7)
 * ======================================================================== */

static void local_to_gps(float x, float y, float z,
                         int32_t origin_lat, int32_t origin_lon, int32_t origin_alt,
                         int32_t *out_lat, int32_t *out_lon, int32_t *out_alt_mm) {
    float lat_deg = (float)origin_lat / 1e7f;
    float lat_rad = lat_deg * DEG_TO_RAD;

    const float m_per_deg_lat = 111319.9f;
    float m_per_deg_lon = 111319.9f * cosf(lat_rad);

    /* clamp */
    if (y < -20.0f) y = -20.0f; else if (y > 20.0f) y = 20.0f;
    if (x < -20.0f) x = -20.0f; else if (x > 20.0f) x = 20.0f;
    if (z <   0.0f) z =   0.0f; else if (z > 10.0f) z = 10.0f;

    *out_lat = origin_lat + (int32_t)(y * 1e7f / m_per_deg_lat);
    *out_lon = origin_lon + (int32_t)(x * 1e7f / m_per_deg_lon);
    *out_alt_mm = origin_alt * 10 + (int32_t)(z * 1000.0f);
}

/* ========================================================================
 * 3. NMEA_Generator struct + generate
 * ======================================================================== */

struct NMEA_Generator {
    char ggpa[100];
    char rmc[100];
};

static void nmea_gen_init(struct NMEA_Generator *n) {
    n->ggpa[0] = '\0';
    n->rmc[0]  = '\0';
}

/**
 * Generate $GPGGA + $GPRMC sentences from UWB position.
 * @param now_sec  current time in seconds (from HAL_GetTick() / 1000)
 *                  passed by caller to keep this header HAL-free.
 */
static void nmea_gen_generate(struct NMEA_Generator *n,
                              float x, float y, float z,
                              float vx, float vy,
                              int32_t origin_lat, int32_t origin_lon, int32_t origin_alt,
                              uint32_t now_sec) {
    /* Step 1: UWB → GPS (E7) */
    int32_t lat_e7, lon_e7, alt_mm;
    local_to_gps(x, y, z, origin_lat, origin_lon, origin_alt, &lat_e7, &lon_e7, &alt_mm);
    float alt_m = (float)alt_mm / 1000.0f;

    /* Step 2: E7 → NMEA ddmm.mmmm */
    float lat_deg = (float)lat_e7 / 1e7f;
    float lon_deg = (float)lon_e7 / 1e7f;

    char lat_str[16], lon_str[16];
    deg_to_nmea_lat(lat_deg, lat_str);
    deg_to_nmea_lon(lon_deg, lon_str);

    /* Step 3: Timestamp (seconds → HHMMSS) */
    unsigned long now = now_sec;
    int h = (int)((now / 3600UL) % 24UL);
    int m = (int)((now / 60UL) % 60UL);
    int s = (int)(now % 60UL);
    char time_str[10];
    snprintf(time_str, sizeof(time_str), "%02d%02d%02d", h, m, s);

    /* Step 4: Date from compile-time __DATE__ */
    int day = (__DATE__[4] == ' ' ? 0 : (__DATE__[4] - '0') * 10) + (__DATE__[5] - '0');
    int month = month_num(__DATE__);
    int year = (__DATE__[9] - '0') * 10 + (__DATE__[10] - '0');
    char date_str[8];
    snprintf(date_str, sizeof(date_str), "%02d%02d%02d", day, month, year);

    /* Step 5: Speed + course */
    float speed_kn = sqrtf(vx * vx + vy * vy) * 1.94384f;
    float course = atan2f(vx, vy) * RAD_TO_DEG;
    if (course < 0.0f) course += 360.0f;

    /* Step 6: $GPGGA */
    snprintf(n->ggpa, sizeof(n->ggpa),
             "$GPGGA,%s.00,%s,%s,1,08,1.0,%.1f,M,0.0,M,,*",
             time_str, lat_str, lon_str, alt_m);
    nmea_checksum_append(n->ggpa);

    /* Step 7: $GPRMC */
    snprintf(n->rmc, sizeof(n->rmc),
             "$GPRMC,%s.00,A,%s,%s,%.2f,%.1f,%s,,,A*",
             time_str, lat_str, lon_str, speed_kn, course, date_str);
    nmea_checksum_append(n->rmc);
}

static const char* nmea_gen_ggpa(const struct NMEA_Generator *n) { return n->ggpa; }
static const char* nmea_gen_rmc(const struct NMEA_Generator *n)  { return n->rmc; }

#endif /* UWB_NMEA_H */
