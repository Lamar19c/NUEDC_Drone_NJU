/**
 * uwb_solver.h — UWB 三边测量求解器 + 双中值滤波 + 协议解析 (纯 C)
 *
 * 从 uwb_arduino/uwb_solver.h 改写 — C++ class → C struct
 */
#ifndef UWB_SOLVER_H
#define UWB_SOLVER_H

#include "uwb_config.h"
#include <math.h>      /* fabsf, sqrtf, atan2f, cosf, sinf */
#include <stdlib.h>    /* atof */
#include <string.h>    /* strlen, strncmp, strchr */
#include <stdio.h>     /* snprintf */
#include <stdint.h>    /* int32_t, uint8_t */

/* ---- 中位数计算 (DistanceFilter + PositionFilter 共用) ---- */

static float median_float(float arr[], int n) {
    float sorted[16];  /* max(DIST_WINDOW_SIZE, POS_WINDOW_SIZE) = 8 */
    for (int i = 0; i < n; i++) sorted[i] = arr[i];
    /* bubble sort (n ≤ 8, fast enough) */
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float tmp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = tmp;
            }
        }
    }
    if (n % 2 == 1) return sorted[n / 2];
    return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0f;
}

/* ========================================================================
 * 1. UWB_Parser — $DIST,M1,S<id>,<distance> 协议解析
 * ======================================================================== */

struct UWB_Parser {
    char buffer[64];
    int  buffer_idx;
    float distances[8];  /* S1..S8, -1.0f = 未收到 */
};

static void uwb_parser_init(struct UWB_Parser *p) {
    p->buffer_idx = 0;
    p->buffer[0] = '\0';
    for (int i = 0; i < 8; i++) p->distances[i] = -1.0f;
}

/**
 * 解析 $DIST,M1,S<id>,<distance_meters>
 * 返回 1 = 完成一行解析, 0 = 继续接收
 */
static int uwb_parser_parse_line(struct UWB_Parser *p, const char *line) {
    if (line[0] != '$' || line[1] != 'D' || line[2] != 'I' ||
        line[3] != 'S' || line[4] != 'T' || line[5] != ',') {
        return 0;
    }

    const char *ptr = line + 6;  /* skip "$DIST," */

    /* field 1: M1 (skip) */
    const char *comma = strchr(ptr, ',');
    if (!comma) return 0;
    ptr = comma + 1;

    /* field 2: S<id> */
    if (*ptr != 'S') return 0;
    ptr++;
    int slave_id = 0;
    while (*ptr >= '0' && *ptr <= '9') {
        slave_id = slave_id * 10 + (*ptr - '0');
        ptr++;
    }
    if (slave_id < 1 || slave_id > 8) return 0;
    if (*ptr != ',') return 0;
    ptr++;

    /* field 3: distance */
    float dist = (float)atof(ptr);
    if (dist <= 0.0f || dist >= 50.0f) {
        p->distances[slave_id - 1] = -1.0f;
        return 1;
    }

    p->distances[slave_id - 1] = dist;
    return 1;
}

/**
 * 喂入新字符。返回 1 表示解析到完整一行。
 */
static int uwb_parser_feed(struct UWB_Parser *p, char c) {
    if (c == '\n' || c == '\r') {
        if (p->buffer_idx > 0) {
            p->buffer[p->buffer_idx] = '\0';
            p->buffer_idx = 0;
            return uwb_parser_parse_line(p, p->buffer);
        }
        return 0;
    }
    if (p->buffer_idx < (int)(sizeof(p->buffer) - 1)) {
        p->buffer[p->buffer_idx++] = c;
    }
    return 0;
}

static void uwb_parser_get_distances(struct UWB_Parser *p, float out[], int max_anchors) {
    for (int i = 0; i < max_anchors; i++) {
        out[i] = p->distances[i];
    }
}

static int uwb_parser_valid_count(struct UWB_Parser *p) {
    int cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (p->distances[i] >= 0.0f && p->distances[i] < 50.0f) cnt++;
    }
    return cnt;
}

static void uwb_parser_clear_distances(struct UWB_Parser *p) {
    for (int i = 0; i < 8; i++) p->distances[i] = -1.0f;
}

/* ========================================================================
 * 2. DistanceFilter — 逐锚点滑动窗口中位数 + 跳变限幅
 * ======================================================================== */

struct DistanceFilter {
    float window[DIST_WINDOW_SIZE][8];
    int   window_count;
};

static void dist_filter_init(struct DistanceFilter *df) {
    df->window_count = 0;
}

static void dist_filter_apply(struct DistanceFilter *df, float raw[], int count) {
    /* push current frame into sliding window */
    if (df->window_count < DIST_WINDOW_SIZE) {
        for (int i = 0; i < count; i++) {
            df->window[df->window_count][i] = raw[i];
        }
        df->window_count++;
        if (df->window_count < 3) return;
    } else {
        /* shift left, drop oldest */
        for (int w = 0; w < DIST_WINDOW_SIZE - 1; w++) {
            for (int i = 0; i < count; i++) {
                df->window[w][i] = df->window[w + 1][i];
            }
        }
        for (int i = 0; i < count; i++) {
            df->window[DIST_WINDOW_SIZE - 1][i] = raw[i];
        }
    }

    /* per-anchor median + jump-limit */
    for (int i = 0; i < count; i++) {
        if (raw[i] <= 0.0f || raw[i] >= 50.0f) continue;

        float hist[DIST_WINDOW_SIZE];
        int hist_cnt = 0;
        for (int w = 0; w < df->window_count; w++) {
            float v = df->window[w][i];
            if (v > 0.0f && v < 50.0f) {
                hist[hist_cnt++] = v;
            }
        }
        if (hist_cnt < 2) continue;

        float med = median_float(hist, hist_cnt);
        if (fabsf(raw[i] - med) > MAX_DIST_JUMP) {
            raw[i] = med;
        }
    }
}

/* ========================================================================
 * 3. PositionFilter — 解算后 x,y,z 中值平滑
 * ======================================================================== */

struct PositionFilter {
    float wx[POS_WINDOW_SIZE], wy[POS_WINDOW_SIZE], wz[POS_WINDOW_SIZE];
    int   count;
};

static void pos_filter_init(struct PositionFilter *pf) {
    pf->count = 0;
}

static void pos_filter_apply(struct PositionFilter *pf, float *x, float *y, float *z) {
    if (pf->count < POS_WINDOW_SIZE) {
        pf->wx[pf->count] = *x;
        pf->wy[pf->count] = *y;
        pf->wz[pf->count] = *z;
        pf->count++;
    } else {
        for (int i = 0; i < POS_WINDOW_SIZE - 1; i++) {
            pf->wx[i] = pf->wx[i + 1];
            pf->wy[i] = pf->wy[i + 1];
            pf->wz[i] = pf->wz[i + 1];
        }
        pf->wx[POS_WINDOW_SIZE - 1] = *x;
        pf->wy[POS_WINDOW_SIZE - 1] = *y;
        pf->wz[POS_WINDOW_SIZE - 1] = *z;
    }

    if (pf->count >= 3) {
        *x = median_float(pf->wx, pf->count);
        *y = median_float(pf->wy, pf->count);
        *z = median_float(pf->wz, pf->count);
    }
}

/* ========================================================================
 * 4. UWBSolver — 加权最小二乘三边测量 (中值滤波替代 EKF)
 * ======================================================================== */

struct UWBSolver {
    const float (*anchors)[3];
    int           anchor_count;
    int           mode_2d;
    unsigned long last_t;
    struct PositionFilter pos_filter;
    float         last_x, last_y, last_z;
    int           has_last_pos;
};

/* ---- 4a. 3x3 线性方程组 (Cramer 法则) ---- */

static int solve3x3(float out[3], const float A[9], const float b[3]) {
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);

    if (fabsf(det) < 1e-10f) return 0;

    float inv_det = 1.0f / det;

    out[0] = (b[0] * (A[4] * A[8] - A[5] * A[7])
           -  A[1] * (b[1] * A[8] - A[5] * b[2])
           +  A[2] * (b[1] * A[7] - A[4] * b[2])) * inv_det;

    out[1] = (A[0] * (b[1] * A[8] - A[5] * b[2])
           -  b[0] * (A[3] * A[8] - A[5] * A[6])
           +  A[2] * (A[3] * b[2] - b[1] * A[6])) * inv_det;

    out[2] = (A[0] * (A[4] * b[2] - b[1] * A[7])
           -  A[1] * (A[3] * b[2] - b[1] * A[6])
           +  b[0] * (A[3] * A[7] - A[4] * A[6])) * inv_det;

    return 1;
}

static void uwb_solver_init(struct UWBSolver *s, const float anchors[][3], int count) {
    s->anchors = anchors;
    s->anchor_count = count;
    s->last_t = 0;
    s->mode_2d = 0;
    s->last_x = s->last_y = s->last_z = 0.0f;
    s->has_last_pos = 0;
    pos_filter_init(&s->pos_filter);
}

/* ---- 4b. 2D 求解 (3 锚点降级, Z 固定 DEFAULT_HEIGHT) ---- */

static int solve_2d(struct UWBSolver *s, const int idx[], const float dist[],
                    float *out_x, float *out_y, float *out_z) {
    float fixed_z = DEFAULT_HEIGHT;

    int i0 = idx[0], i1 = idx[1], i2 = idx[2];
    float p0x = s->anchors[i0][0], p0y = s->anchors[i0][1], p0z = s->anchors[i0][2];
    float p1x = s->anchors[i1][0], p1y = s->anchors[i1][1], p1z = s->anchors[i1][2];
    float p2x = s->anchors[i2][0], p2y = s->anchors[i2][1], p2z = s->anchors[i2][2];

    float d0 = dist[i0], d1 = dist[i1], d2 = dist[i2];
    float p0_norm_sq = p0x * p0x + p0y * p0y + p0z * p0z;

    float w1 = 1.0f / (d1 + 0.1f);
    float w2 = 1.0f / (d2 + 0.1f);

    float A00 = 2.0f * (p1x - p0x) * w1;
    float A01 = 2.0f * (p1y - p0y) * w1;
    float A10 = 2.0f * (p2x - p0x) * w2;
    float A11 = 2.0f * (p2y - p0y) * w2;

    float p1_norm_sq = p1x * p1x + p1y * p1y + p1z * p1z;
    float p2_norm_sq = p2x * p2x + p2y * p2y + p2z * p2z;

    float b0 = (d0 * d0 - d1 * d1 - p0_norm_sq + p1_norm_sq
                - 2.0f * (p1z - p0z) * fixed_z) * w1;
    float b1 = (d0 * d0 - d2 * d2 - p0_norm_sq + p2_norm_sq
                - 2.0f * (p2z - p0z) * fixed_z) * w2;

    float ATA0 = A00 * A00 + A10 * A10;
    float ATA1 = A00 * A01 + A10 * A11;
    float ATA3 = A01 * A01 + A11 * A11;

    float det = ATA0 * ATA3 - ATA1 * ATA1;
    if (fabsf(det) < 1e-10f) return 0;

    float ATb0 = A00 * b0 + A10 * b1;
    float ATb1 = A01 * b0 + A11 * b1;

    float inv_det = 1.0f / det;
    float rx = ( ATA3 * ATb0 - ATA1 * ATb1) * inv_det;
    float ry = (-ATA1 * ATb0 + ATA0 * ATb1) * inv_det;

    if (fabsf(rx) > 100.0f || fabsf(ry) > 100.0f) return 0;

    *out_x = (rx < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((rx > POS_CLAMP_MAX) ? POS_CLAMP_MAX : rx);
    *out_y = (ry < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((ry > POS_CLAMP_MAX) ? POS_CLAMP_MAX : ry);
    *out_z = fixed_z;
    return 1;
}

/* ---- 4c. 3D 求解 (4+ 锚点, 加权正规方程 + Cramer) ---- */

static int solve_3d(struct UWBSolver *s, const int idx[], const float dist[], int n,
                    float *out_x, float *out_y, float *out_z) {
    int i0 = idx[0];
    float p0x = s->anchors[i0][0], p0y = s->anchors[i0][1], p0z = s->anchors[i0][2];
    float d0 = dist[i0];
    float p0_norm_sq = p0x * p0x + p0y * p0y + p0z * p0z;

    int m = n - 1;  /* number of equations */
    float ATA[9] = {0};
    float ATb[3] = {0};

    for (int j = 0; j < m; j++) {
        int ij = idx[j + 1];
        float px = s->anchors[ij][0], py = s->anchors[ij][1], pz = s->anchors[ij][2];
        float dj = dist[ij];

        float w = 1.0f / (dj + 0.1f);

        float a0 = 2.0f * (px - p0x) * w;
        float a1 = 2.0f * (py - p0y) * w;
        float a2 = 2.0f * (pz - p0z) * w;

        float pj_norm_sq = px * px + py * py + pz * pz;
        float bj = (d0 * d0 - dj * dj - p0_norm_sq + pj_norm_sq) * w;

        ATA[0] += a0 * a0;  ATA[1] += a0 * a1;  ATA[2] += a0 * a2;
        ATA[3] += a1 * a0;  ATA[4] += a1 * a1;  ATA[5] += a1 * a2;
        ATA[6] += a2 * a0;  ATA[7] += a2 * a1;  ATA[8] += a2 * a2;

        ATb[0] += a0 * bj;
        ATb[1] += a1 * bj;
        ATb[2] += a2 * bj;
    }

    float result[3];
    if (!solve3x3(result, ATA, ATb)) return 0;

    for (int i = 0; i < 3; i++) {
        if (fabsf(result[i]) > 100.0f) return 0;
    }

    *out_x = (result[0] < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((result[0] > POS_CLAMP_MAX) ? POS_CLAMP_MAX : result[0]);
    *out_y = (result[1] < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((result[1] > POS_CLAMP_MAX) ? POS_CLAMP_MAX : result[1]);
    *out_z = (result[2] < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((result[2] > POS_CLAMP_MAX) ? POS_CLAMP_MAX : result[2]);
    return 1;
}

/* ---- 4d. 求解入口 — 收集有效锚点 → 三边测量 → 中值滤波 → 速度 ---- */

static int uwb_solver_solve(struct UWBSolver *s, const float distances[],
                            unsigned long now_ms,
                            float *x, float *y, float *z,
                            float *vx, float *vy, float *vz) {
    /* collect valid anchors */
    int valid_idx[8];
    int valid_cnt = 0;
    for (int i = 0; i < s->anchor_count; i++) {
        if (distances[i] > 0.0f && distances[i] < 50.0f) {
            valid_idx[valid_cnt++] = i;
        }
    }
    if (valid_cnt < 3) return 0;

    /* compute dt */
    float dt = 0.1f;
    if (s->last_t > 0) {
        dt = (float)(now_ms - s->last_t) / 1000.0f;
    }
    s->last_t = now_ms;
    if (dt < 0.02f) dt = 0.02f;
    if (dt > 2.0f)  dt = 2.0f;

    /* save previous position for velocity */
    float prev_x = s->last_x, prev_y = s->last_y, prev_z = s->last_z;
    int had_prev = s->has_last_pos;

    /* trilateration */
    if (valid_cnt == 3) {
        if (!solve_2d(s, valid_idx, distances, x, y, z)) return 0;
    } else {
        if (!solve_3d(s, valid_idx, distances, valid_cnt, x, y, z)) return 0;
    }

    /* position median filter */
    pos_filter_apply(&s->pos_filter, x, y, z);

    /* store filtered position */
    s->last_x = *x; s->last_y = *y; s->last_z = *z;
    s->has_last_pos = 1;

    /* finite-difference velocity */
    if (had_prev && dt > 0.001f) {
        *vx = (*x - prev_x) / dt;
        *vy = (*y - prev_y) / dt;
        *vz = (*z - prev_z) / dt;
    } else {
        *vx = *vy = *vz = 0.0f;
    }

    return 1;
}

#endif /* UWB_SOLVER_H */
