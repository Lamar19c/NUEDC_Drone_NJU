/**
 * uwb_nmea.c — NMEA 0183 GPS 语句生成器 (实现)
 */
#include "uwb_nmea.h"
#include "uwb_config.h"
#include <math.h>      /* fabsf, sqrtf, atan2f, cosf, roundf */
#include <string.h>    /* strlen, strncmp */
#include <stdio.h>     /* snprintf */

/* ========================================================================
 * 1. 整数格式化 helpers (替代 %f，STM32 newlib-nano 兼容)
 * ======================================================================== */

static int fmt_1dp(char *buf, float val) {
    int sign = (val < 0.0f) ? -1 : 1;
    float a = fabsf(val);
    int ip = (int)a;
    int fp = (int)roundf((a - (float)ip) * 10.0f);
    if (fp >= 10) { ip++; fp = 0; }
    int off = 0;
    if (sign < 0) buf[off++] = '-';
    off += snprintf(buf + off, 8, "%d.%01d", ip, fp);
    return off;
}

static int fmt_2dp(char *buf, float val) {
    int sign = (val < 0.0f) ? -1 : 1;
    float a = fabsf(val);
    int ip = (int)a;
    int fp = (int)roundf((a - (float)ip) * 100.0f);
    if (fp >= 100) { ip++; fp = 0; }
    int off = 0;
    if (sign < 0) buf[off++] = '-';
    off += snprintf(buf + off, 8, "%d.%02d", ip, fp);
    return off;
}

/* ========================================================================
 * 2. 坐标格式转换
 * ======================================================================== */

static void deg_to_nmea_lat(float deg, char *out) {
    float a = fabsf(deg);
    int d = (int)a;
    float mm = (a - (float)d) * 60.0f;
    int mm_int = (int)mm;
    int mm_frac_raw = (int)roundf((mm - (float)mm_int) * 10000.0f);
    unsigned int mm_frac = (mm_frac_raw < 0) ? 0 : ((mm_frac_raw > 9999) ? 9999 : (unsigned int)mm_frac_raw);
    if (mm_frac >= 10000) { mm_int++; mm_frac = 0; }
    if (mm_int >= 60) { d++; mm_int = 0; }  /* 分钟满 60 向度进位 */
    char dir = (deg >= 0.0f) ? 'N' : 'S';
    snprintf(out, 24, "%02d%02d.%04u,%c", d, mm_int, mm_frac, dir);
}

static void deg_to_nmea_lon(float deg, char *out) {
    float a = fabsf(deg);
    int d = (int)a;
    float mm = (a - (float)d) * 60.0f;
    int mm_int = (int)mm;
    int mm_frac_raw = (int)roundf((mm - (float)mm_int) * 10000.0f);
    unsigned int mm_frac = (mm_frac_raw < 0) ? 0 : ((mm_frac_raw > 9999) ? 9999 : (unsigned int)mm_frac_raw);
    if (mm_frac >= 10000) { mm_int++; mm_frac = 0; }
    if (mm_int >= 60) { d++; mm_int = 0; }  /* 分钟满 60 向度进位 */
    char dir = (deg >= 0.0f) ? 'E' : 'W';
    snprintf(out, 24, "%03d%02d.%04u,%c", d, mm_int, mm_frac, dir);
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
 * 3. localToGPS — UWB 坐标 → GPS (E7)
 *
 *    坐标映射:
 *      X → Longitude (经度, 东西方向)
 *      Y → Latitude  (纬度, 南北方向)
 *      Z → Altitude  (海拔, 上下方向)
 *
 *    锚点布局 (v3): S1=(0,0,1.5) S2=(5,0,1.5) S3=(0,-5,0) S4=(5,5,1.5)
 *    S2 在 S1 正东 (X+), S3 在 S1 正南 (Y-)
 *
 *    统一使用 POS_CLAMP_MIN/MAX 限幅
 * ======================================================================== */

/* ---- 预计算常量 (避免每次 local_to_gps 重复计算 cosf) ---- */

static float g_m_per_deg_lon = 0.0f;   /* 0 = 未初始化 */

void local_to_gps(float x, float y, float z,
                  int32_t origin_lat, int32_t origin_lon, int32_t origin_alt,
                  int32_t *out_lat, int32_t *out_lon, int32_t *out_alt_mm) {
    const float m_per_deg_lat = 111319.9f;

    /* cos(lat) 基本不变，仅首次调用时计算一次 */
    if (g_m_per_deg_lon == 0.0f) {
        float lat_deg = (float)origin_lat / 1e7f;
        float lat_rad = lat_deg * DEG_TO_RAD;
        g_m_per_deg_lon = 111319.9f * cosf(lat_rad);
    }
    const float m_per_deg_lon = g_m_per_deg_lon;

    /* 统一使用配置文件中的限幅宏 */
    if (y < POS_CLAMP_MIN) y = POS_CLAMP_MIN; else if (y > POS_CLAMP_MAX) y = POS_CLAMP_MAX;
    if (x < POS_CLAMP_MIN) x = POS_CLAMP_MIN; else if (x > POS_CLAMP_MAX) x = POS_CLAMP_MAX;
    if (z < POS_CLAMP_MIN) z = POS_CLAMP_MIN; else if (z > POS_CLAMP_MAX) z = POS_CLAMP_MAX;

    *out_lat = origin_lat + (int32_t)(y * 1e7f / m_per_deg_lat);
    *out_lon = origin_lon + (int32_t)(x * 1e7f / m_per_deg_lon);
    *out_alt_mm = origin_alt * 10 + (int32_t)(z * 1000.0f);
}

/* ========================================================================
 * 4. NMEA_Generator
 * ======================================================================== */

void nmea_gen_init(struct NMEA_Generator *n) {
    n->ggpa[0] = '\0';
    n->rmc[0]  = '\0';
    n->vtg[0]  = '\0';
}

void nmea_gen_generate(struct NMEA_Generator *n,
                       float x, float y, float z,
                       float vx, float vy,
                       int32_t origin_lat, int32_t origin_lon, int32_t origin_alt,
                       uint32_t now_sec) {
    /* Step 1: UWB → GPS (E7) */
    int32_t lat_e7, lon_e7, alt_mm;
    local_to_gps(x, y, z, origin_lat, origin_lon, origin_alt, &lat_e7, &lon_e7, &alt_mm);

    /* Step 2: E7 → NMEA ddmm.mmmm */
    float lat_deg = (float)lat_e7 / 1e7f;
    float lon_deg = (float)lon_e7 / 1e7f;

    char lat_str[24], lon_str[24];
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
    char date_str[16];
    snprintf(date_str, sizeof(date_str), "%02d%02d%02d", day, month, year);

    /* Step 5: Speed + course */
    float speed_kn = sqrtf(vx * vx + vy * vy) * 1.94384f;
    float speed_kmh = speed_kn * 1.852f;
    float course = atan2f(vx, vy) * RAD_TO_DEG;
    if (course < 0.0f) course += 360.0f;

    /* Format float values with integer method */
    float alt_m = (float)alt_mm / 1000.0f;
    char alt_str[12], hdop_str[12];
    char spd_str[12], crs_str[12], spdkm_str[12];
    fmt_1dp(alt_str,  alt_m);
    fmt_1dp(hdop_str, NMEA_HDOP);
    fmt_2dp(spd_str,  speed_kn);
    fmt_1dp(crs_str,  course);
    fmt_2dp(spdkm_str, speed_kmh);

    /* Step 6: $GPGGA — GPS Fix Data
     * Format: time,lat,N,lon,E,quality,sats,hdop,alt,M,geoid,M,age,refid */
    snprintf(n->ggpa, sizeof(n->ggpa),
             "$GPGGA,%s.00,%s,%s,1,%02d,%s,%s,M,0.0,M,,*",
             time_str, lat_str, lon_str, NMEA_SATELLITES, hdop_str, alt_str);
    nmea_checksum_append(n->ggpa);

    /* Step 7: $GPRMC — Recommended Minimum Navigation Information
     * Format: time,status,lat,N,lon,E,speed,course,date,magvar,magdir,mode */
    snprintf(n->rmc, sizeof(n->rmc),
             "$GPRMC,%s.00,A,%s,%s,%s,%s,%s,0.0,E,A*",
             time_str, lat_str, lon_str, spd_str, crs_str, date_str);
    nmea_checksum_append(n->rmc);

    /* Step 8: $GPVTG — Course Over Ground and Ground Speed
     * Format: cogt,T,cogm,M,sog,N,sogk,K,mode */
    snprintf(n->vtg, sizeof(n->vtg),
             "$GPVTG,%s,T,0.0,M,%s,N,%s,K,A*",
             crs_str, spd_str, spdkm_str);
    nmea_checksum_append(n->vtg);
}

const char* nmea_gen_ggpa(const struct NMEA_Generator *n) { return n->ggpa; }
const char* nmea_gen_rmc(const struct NMEA_Generator *n)  { return n->rmc; }
const char* nmea_gen_vtg(const struct NMEA_Generator *n)  { return n->vtg; }
