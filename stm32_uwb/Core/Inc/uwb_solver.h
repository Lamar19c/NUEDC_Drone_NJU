/**
 * uwb_solver.h — UWB 三边测量求解器 + 双中值滤波 + 协议解析
 *
 * 从 uwb_arduino/uwb_solver.h 改写 — C++ class → C struct + .c
 */
#ifndef UWB_SOLVER_H
#define UWB_SOLVER_H

#include "uwb_config.h"
#include <stdint.h>    /* int32_t, uint32_t */

/* ========================================================================
 * 1. UWB_Parser — $DIST,M1,S<id>,<distance> 协议解析
 *    v2: per-anchor timestamps + timeout-based validity
 * ======================================================================== */

struct UWB_Parser {
    char     buffer[64];
    int      buffer_idx;
    float    distances[8];      /* S1..S8, -1.0f = 未收到 */
    uint32_t last_update_ms[8]; /* per-anchor last update timestamp (ms) */
};

void uwb_parser_init(struct UWB_Parser *p);
int  uwb_parser_feed(struct UWB_Parser *p, char c, uint32_t now_ms);
int  uwb_parser_valid_count(struct UWB_Parser *p, uint32_t now_ms);
int  uwb_parser_get_valid_distances(struct UWB_Parser *p, float out[],
                                     int max_anchors, uint32_t now_ms);

/* ========================================================================
 * 2. DistanceFilter — 逐锚点滑动窗口中位数 + 跳变限幅
 * ======================================================================== */

struct DistanceFilter {
    float window[DIST_WINDOW_SIZE][8];
    int   window_count;
};

void dist_filter_init(struct DistanceFilter *df);
void dist_filter_apply(struct DistanceFilter *df, float raw[], int count);

/* ========================================================================
 * 3. PositionFilter — 解算后 x,y,z 中值平滑
 * ======================================================================== */

struct PositionFilter {
    float wx[POS_WINDOW_SIZE], wy[POS_WINDOW_SIZE], wz[POS_WINDOW_SIZE];
    int   count;
};

void pos_filter_init(struct PositionFilter *pf);
void pos_filter_apply(struct PositionFilter *pf, float *x, float *y, float *z);

/* ========================================================================
 * 3b. VelocityFilter — 一阶低通，独立于位置滤波降低相位滞后
 * ======================================================================== */

struct VelocityFilter {
    float vx, vy, vz;
    int   has_last;
};

void vel_filter_init(struct VelocityFilter *vf);
void vel_filter_apply(struct VelocityFilter *vf, float *vx, float *vy, float *vz);

/* ========================================================================
 * 4. UWBSolver — 加权最小二乘三边测量 (中值滤波替代 EKF)
 *
 *    v2 修复:
 *    - 加权最小二乘权重使用 sqrt(w) (修正权重平方 bug)
 *    - 3D 求解选距离最近锚点为参考 (提升鲁棒性)
 *    - 解算后残差校验，超过阈值则丢弃
 * ======================================================================== */

struct UWBSolver {
    const float (*anchors)[3];
    int           anchor_count;
    unsigned long last_t;
    struct PositionFilter pos_filter;
    float         last_x, last_y, last_z;      /* filtered position */
    float         last_raw_x, last_raw_y, last_raw_z;  /* raw for velocity */
    int           has_last_pos;
};

void uwb_solver_init(struct UWBSolver *s, const float anchors[][3], int count);
int  uwb_solver_solve(struct UWBSolver *s, const float distances[],
                       unsigned long now_ms,
                       float *x, float *y, float *z,
                       float *vx, float *vy, float *vz);

/* ---- diagnostic globals (read from main loop, written by parser ISR) ---- */
extern char     g_uwb_last_raw_line[64];
extern int      g_uwb_has_raw_line;
extern uint32_t g_parser_line_ok;
extern uint32_t g_parser_line_bad;

#endif /* UWB_SOLVER_H */
