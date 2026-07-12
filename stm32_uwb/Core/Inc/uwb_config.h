/**
 * uwb_config.h — UWB 室内定位 STM32 配置 (纯 C)
 *
 * 使用前请根据实际场地修改:
 *   1. ANCHOR_POSITIONS — 锚点坐标 (卷尺量测)
 *   2. ANCHOR_COUNT — 锚点数量 (3 或 4)
 *   3. GPS_ORIGIN_LAT/LON/ALT — 场地 GPS 参考原点
 */
#ifndef UWB_CONFIG_H
#define UWB_CONFIG_H

#include <stdint.h>   /* int32_t */

/* ========================================================================
 * 1. 锚点
 * ======================================================================== */

#define ANCHOR_COUNT 4

static const float ANCHOR_POSITIONS[ANCHOR_COUNT][3] = {
    {0.0f, 0.0f, 1.5f},   /* S1 — 场地原点 */
    {5.0f, 0.0f, 1.5f},   /* S2 — X轴方向 */
    {0.0f, 5.0f, 1.5f},   /* S3 — Y轴方向 */
    {5.0f, 5.0f, 1.5f},   /* S4 — 对角 */
};

/* ========================================================================
 * 2. 求解器参数
 * ======================================================================== */

#define DEFAULT_HEIGHT   1.0f
#define POS_CLAMP_MIN   -10.0f
#define POS_CLAMP_MAX    10.0f
#define RESIDUAL_MAX_3D  2.0f
#define RESIDUAL_MAX_2D  1.5f

/* ========================================================================
 * 3. 滤波参数
 * ======================================================================== */

#define DIST_WINDOW_SIZE  8
#define MAX_DIST_JUMP     0.8f
#define POS_WINDOW_SIZE   5         /* 减小窗口，降低阶梯效应 */
#define DIST_ALPHA        0.35f     /* 距离端 EMA 低通系数 (抑制高频毛刺) */
#define POS_ALPHA         0.30f     /* α-β 滤波 α 系数 (位置更新权重) */
#define POS_BETA          0.05f     /* α-β 滤波 β 系数 (速度更新权重) */
#define DIST_TIMEOUT_MS   150       /* 锚点距离超时 (ms)，超过此值视为失效 */
#define VEL_ALPHA         0.3f      /* NMEA 速度 EMA 低通系数 */

/* ========================================================================
 * 4. 数学常量 (Arduino 兼容)
 * ======================================================================== */

#ifndef RAD_TO_DEG
#define RAD_TO_DEG  57.29577951308232f
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD  0.017453292519943295f
#endif

/* ========================================================================
 * 5. NMEA / GPS
 * ======================================================================== */

#define GPS_ORIGIN_LAT   321148408   /* 32.1148408° (E7) */
#define GPS_ORIGIN_LON   1189590664  /* 118.9590664° (E7) */
#define GPS_ORIGIN_ALT   1200        /* 12.0m (cm) */
#define NMEA_RATE_HZ     5.0f
#define NMEA_HDOP        1.5f        /* HDOP 水平精度因子 (u-blox 典型 1.0~2.0) */
#define NMEA_SATELLITES  10          /* 可见卫星数 (u-blox 典型 8~16) */

/* ========================================================================
 * 6. 主循环
 * ======================================================================== */

#define LOOP_INTERVAL_MS  20

#endif /* UWB_CONFIG_H */
