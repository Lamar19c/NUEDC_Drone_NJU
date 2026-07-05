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

#endif /* UWB_SOLVER_H */
