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
    {0.0f, 0.0f, 1.0f},   /* S1 — 场地原点 */
    {5.0f, 0.0f, 1.5f},   /* S2 — X轴方向 */
    {0.0f, 5.0f, 1.5f},   /* S3 — Y轴方向 */
    {5.0f, 5.0f, 1.0f},   /* S4 — 对角 */
};

/* ========================================================================
 * 2. 求解器参数
 * ======================================================================== */

#define DEFAULT_HEIGHT   1.0f
#define POS_CLAMP_MIN   -10.0f
#define POS_CLAMP_MAX    10.0f
#define RESIDUAL_MAX_3D  2.5f      /* 放宽残差，提升低速解算成功率 */
#define RESIDUAL_MAX_2D  2.0f      /* 放宽残差，提升低速解算成功率 */

/* ========================================================================
 * 3. 滤波参数
 * ======================================================================== */

#define DIST_WINDOW_SIZE  1         /* TDMA优化: 直通，不引入逐锚点相位滞后 */
#define MAX_DIST_JUMP     99.0f     /* TDMA优化: 直通，不跳变限幅 */
#define POS_WINDOW_SIZE   1         /* TDMA优化: 直通, 位置中值已关闭 */
#define DIST_ALPHA        1.0f      /* TDMA优化: 直通, 逐锚点EMA放大时间偏斜 */
#define POS_ALPHA         0.55f     /* 全局 EMA 输出平滑，不影响时偏 */
#define DIST_TIMEOUT_MS   150       /* TDMA优化: 短超时压缩时间窗 v*T≈15cm@1m/s */
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
