/**
 * uwb_solver.c — UWB 三边测量求解器 + 双中值滤波 + 协议解析 (实现)
 *
 * TREK1000 迁移版 — 新增 mc 协议解析，兼容 $DIST fallback
 * 直接替换 stm32_uwb/Core/Src/uwb_solver.c 即可
 */
#include "uwb_solver.h"
#include "uwb_config.h"
#include <math.h>      /* fabsf, sqrtf, atan2f, cosf, sinf */
#include <stdlib.h>    /* atof */
#include <string.h>    /* strlen, strncmp, strchr */
#include <stdio.h>     /* snprintf */

/* ---- 内部 helper: 中位数计算 ---- */

char     g_uwb_last_raw_line[64] = {0};
int      g_uwb_has_raw_line     = 0;
uint32_t g_parser_line_ok       = 0;
uint32_t g_parser_line_bad      = 0;

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
 * 1. UWB_Parser
 * ======================================================================== */

void uwb_parser_init(struct UWB_Parser *p) {
    p->buffer_idx = 0;
    p->buffer[0] = '\0';
    for (int i = 0; i < 8; i++) {
        p->distances[i]      = -1.0f;
        p->last_update_ms[i] = 0;
    }
}

/**
 * 解析 $DIST,M1,S<id>,<distance_meters>
 * 返回 1 = 完成一行解析, 0 = 继续接收
 */
static int uwb_parser_parse_line(struct UWB_Parser *p, const char *line,
                                  uint32_t now_ms) {
    if (line[0] != '$' || line[1] != 'D' || line[2] != 'I' ||
        line[3] != 'S' || line[4] != 'T' || line[5] != ',') {
        /* capture raw line for diagnostic */
        if (line[0] != '\0') {
            int i;
            for (i = 0; line[i] && i < 63; i++)
                g_uwb_last_raw_line[i] = line[i];
            g_uwb_last_raw_line[i] = '\0';
            g_uwb_has_raw_line = 1;
        }
        g_parser_line_bad++;
        return 0;
    }
    g_parser_line_ok++;

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

    p->distances[slave_id - 1]      = dist;
    p->last_update_ms[slave_id - 1] = now_ms;
    return 1;
}

/* ========================================================================
 * 1b. TREK1000 mc 协议解析 (新增)
 *
 * mc 帧格式:
 *   mc <valid_hex> <S1_mm_hex> <S2_mm_hex> <S3_mm_hex> <S4_mm_hex>
 *      <lcount_hex> <rnum_hex> <time_hex> t<tag>:a<anchor>\r\n
 *
 * 示例:
 *   mc 0f 0000064a 000005f2 00000680 00000610 000a 01 000001f4 t0:0
 *
 * 所有数值为 HEX 编码，距离单位 mm。
 * ======================================================================== */

static int uwb_parser_parse_mc(struct UWB_Parser *p, const char *line,
                                uint32_t now_ms) {
    /* validate frame header "mc " */
    if (line[0] != 'm' || line[1] != 'c' || line[2] != ' ') return 0;

    const char *ptr = line + 3;
    char *end;

    /* field 1: valid_mask (1 hex char, bitmask: bit0=S1 .. bit3=S4) */
    int mask = (int)strtol(ptr, &end, 16);
    if (ptr == end) return 0;
    ptr = end;

    /* fields 2-5: 4 distance values (HEX 8-digit, unit: mm) */
    uint32_t dists[4] = {0};
    for (int i = 0; i < 4; i++) {
        while (*ptr == ' ') ptr++;
        if (*ptr < '0') return 0;
        dists[i] = (uint32_t)strtol(ptr, &end, 16);
        if (ptr == end) return 0;
        ptr = end;
    }

    /* store valid distances: mm → m */
    for (int i = 0; i < 4; i++) {
        if (mask & (1 << i)) {
            float d = (float)dists[i] / 1000.0f;
            if (d > 0.0f && d < 50.0f) {
                p->distances[i]      = d;
                p->last_update_ms[i] = now_ms;
            }
        }
    }

    g_parser_line_ok++;
    return 1;
}

int uwb_parser_feed(struct UWB_Parser *p, char c, uint32_t now_ms) {
    if (c == '\n' || c == '\r') {
        if (p->buffer_idx > 0) {
            p->buffer[p->buffer_idx] = '\0';
            p->buffer_idx = 0;

            /* ─── TREK1000 migration: route mc frames ─── */
            if (p->buffer[0] == 'm' && p->buffer[1] == 'c')
                return uwb_parser_parse_mc(p, p->buffer, now_ms);

            /* fallback: JZM01 $DIST */
            if (p->buffer[0] == '$')
                return uwb_parser_parse_line(p, p->buffer, now_ms);

            return 0;
        }
        return 0;
    }
    if (p->buffer_idx < (int)(sizeof(p->buffer) - 1)) {
        p->buffer[p->buffer_idx++] = c;
    }
    return 0;
}

int uwb_parser_valid_count(struct UWB_Parser *p, uint32_t now_ms) {
    int cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (p->distances[i] >= 0.0f && p->distances[i] < 50.0f
            && (now_ms - p->last_update_ms[i]) < DIST_TIMEOUT_MS) {
            cnt++;
        }
    }
    return cnt;
}

int uwb_parser_get_valid_distances(struct UWB_Parser *p, float out[],
                                    int max_anchors, uint32_t now_ms) {
    int cnt = 0;
    for (int i = 0; i < max_anchors; i++) {
        if (p->distances[i] >= 0.0f && p->distances[i] < 50.0f
            && (now_ms - p->last_update_ms[i]) < DIST_TIMEOUT_MS) {
            out[i] = p->distances[i];
            cnt++;
        } else {
            out[i] = -1.0f;
        }
    }
    return cnt;
}

/* ========================================================================
 * 2. DistanceFilter
 * ======================================================================== */

void dist_filter_init(struct DistanceFilter *df) {
    df->window_count = 0;
    for (int i = 0; i < 8; i++) {
        df->ema[i]       = 0.0f;
        df->ema_ready[i] = 0;
    }
}

void dist_filter_apply(struct DistanceFilter *df, float raw[], int count) {
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

        /* per-anchor distance EMA — suppress high-frequency noise */
        if (!df->ema_ready[i]) {
            df->ema[i]       = raw[i];
            df->ema_ready[i] = 1;
        } else {
            df->ema[i] += DIST_ALPHA * (raw[i] - df->ema[i]);
        }
        raw[i] = df->ema[i];
    }
}

/* ========================================================================
 * 3. PositionFilter
 * ======================================================================== */

void pos_filter_init(struct PositionFilter *pf) {
    pf->count = 0;
}

void pos_filter_apply(struct PositionFilter *pf, float *x, float *y, float *z) {
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
 * 3b. VelocityFilter — 一阶低通，降低相位滞后
 * ======================================================================== */

void vel_filter_init(struct VelocityFilter *vf) {
    vf->vx = vf->vy = vf->vz = 0.0f;
    vf->has_last = 0;
}

void vel_filter_apply(struct VelocityFilter *vf, float *vx, float *vy, float *vz) {
    if (!vf->has_last) {
        vf->vx = *vx; vf->vy = *vy; vf->vz = *vz;
        vf->has_last = 1;
        return;
    }
    vf->vx += VEL_ALPHA * (*vx - vf->vx);
    vf->vy += VEL_ALPHA * (*vy - vf->vy);
    vf->vz += VEL_ALPHA * (*vz - vf->vz);
    *vx = vf->vx;
    *vy = vf->vy;
    *vz = vf->vz;
}

/* ========================================================================
 * 4. UWBSolver
 * ======================================================================== */

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

void uwb_solver_init(struct UWBSolver *s, const float anchors[][3], int count) {
    s->anchors = anchors;
    s->anchor_count = count;
    s->last_t = 0;
    s->last_x = s->last_y = s->last_z = 0.0f;
    s->last_raw_x = s->last_raw_y = s->last_raw_z = 0.0f;
    s->has_last_pos = 0;
    s->has_ab_state = 0;
    pos_filter_init(&s->pos_filter);
}

/* ---- 4b. 残差计算 ---- */

static float compute_mean_residual(float x, float y, float z,
                                    const int idx[], const float dist[], int n,
                                    const float anchors[][3]) {
    float sum = 0.0f;
    int cnt = 0;
    for (int i = 0; i < n; i++) {
        int ai = idx[i];
        float dx = x - anchors[ai][0];
        float dy = y - anchors[ai][1];
        float dz = z - anchors[ai][2];
        float est_dist = sqrtf(dx * dx + dy * dy + dz * dz);
        sum += fabsf(est_dist - dist[ai]);
        cnt++;
    }
    return (cnt > 0) ? (sum / (float)cnt) : 0.0f;
}

/* ---- 4c. 2D 求解 (3 锚点降级, Z 固定 DEFAULT_HEIGHT) ---- */

static int solve_2d(struct UWBSolver *s, const int idx[], const float dist[],
                    float *out_x, float *out_y, float *out_z) {
    float fixed_z = DEFAULT_HEIGHT;

    int i0 = idx[0], i1 = idx[1], i2 = idx[2];
    float p0x = s->anchors[i0][0], p0y = s->anchors[i0][1], p0z = s->anchors[i0][2];
    float p1x = s->anchors[i1][0], p1y = s->anchors[i1][1], p1z = s->anchors[i1][2];
    float p2x = s->anchors[i2][0], p2y = s->anchors[i2][1], p2z = s->anchors[i2][2];

    float d0 = dist[i0], d1 = dist[i1], d2 = dist[i2];
    float p0_norm_sq = p0x * p0x + p0y * p0y + p0z * p0z;

    /* 加权最小二乘：系数乘以 √w，正规方程权重 = w (而非 w²) */
    float w1 = sqrtf(1.0f / (d1 + 0.1f));
    float w2 = sqrtf(1.0f / (d2 + 0.1f));

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

    /* 残差校验：超出阈值则丢弃 */
    float residual = compute_mean_residual(rx, ry, fixed_z, idx, dist, 3, s->anchors);
    if (residual > RESIDUAL_MAX_2D) return 0;

    *out_x = (rx < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((rx > POS_CLAMP_MAX) ? POS_CLAMP_MAX : rx);
    *out_y = (ry < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((ry > POS_CLAMP_MAX) ? POS_CLAMP_MAX : ry);
    *out_z = fixed_z;
    return 1;
}

/* ---- 4d. 3D 求解 (4+ 锚点, 加权正规方程 + Cramer) ---- */

static int solve_3d(struct UWBSolver *s, int idx[], const float dist[], int n,
                    float *out_x, float *out_y, float *out_z) {
    /* 选择距离最近的锚点作为参考 (提升鲁棒性) */
    int i0 = idx[0];
    float d0 = dist[i0];
    for (int i = 1; i < n; i++) {
        if (dist[idx[i]] < d0) {
            /* swap idx[0] with idx[i] */
            int tmp = idx[0];
            idx[0] = idx[i];
            idx[i] = tmp;
            i0 = idx[0];
            d0 = dist[i0];
        }
    }

    float p0x = s->anchors[i0][0], p0y = s->anchors[i0][1], p0z = s->anchors[i0][2];
    float p0_norm_sq = p0x * p0x + p0y * p0y + p0z * p0z;

    int m = n - 1;  /* number of equations */
    float ATA[9] = {0};
    float ATb[3] = {0};

    for (int j = 0; j < m; j++) {
        int ij = idx[j + 1];
        float px = s->anchors[ij][0], py = s->anchors[ij][1], pz = s->anchors[ij][2];
        float dj = dist[ij];

        /* 加权最小二乘：系数乘以 √w */
        float w = sqrtf(1.0f / (dj + 0.1f));

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

    /* 残差校验：超出阈值则丢弃 */
    float residual = compute_mean_residual(
        result[0], result[1], result[2], idx, dist, n, s->anchors);
    if (residual > RESIDUAL_MAX_3D) return 0;

    *out_x = (result[0] < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((result[0] > POS_CLAMP_MAX) ? POS_CLAMP_MAX : result[0]);
    *out_y = (result[1] < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((result[1] > POS_CLAMP_MAX) ? POS_CLAMP_MAX : result[1]);
    *out_z = (result[2] < POS_CLAMP_MIN) ? POS_CLAMP_MIN : ((result[2] > POS_CLAMP_MAX) ? POS_CLAMP_MAX : result[2]);
    return 1;
}

/* ---- 4e. 求解入口 — 收集有效锚点 → 三边测量 → 中值滤波 → 速度 ---- */

int uwb_solver_solve(struct UWBSolver *s, const float distances[],
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

    /* save previous raw position for velocity */
    float prev_rx = s->last_raw_x, prev_ry = s->last_raw_y, prev_rz = s->last_raw_z;
    int had_prev = s->has_last_pos;

    /* trilateration */
    if (valid_cnt == 3) {
        if (!solve_2d(s, valid_idx, distances, x, y, z)) return 0;
    } else {
        if (!solve_3d(s, valid_idx, distances, valid_cnt, x, y, z)) return 0;
    }

    /* NaN/Inf guard — prevent contamination of filters.
     * Use direct float comparison: NaN != NaN, Inf/NaN * 0 → NaN */
    if (!(*x == *x && *x * 0.0f == 0.0f) ||
        !(*y == *y && *y * 0.0f == 0.0f) ||
        !(*z == *z && *z * 0.0f == 0.0f)) return 0;

    /* save raw position for low-lag velocity computation */
    float raw_x = *x, raw_y = *y, raw_z = *z;
    s->last_raw_x = raw_x; s->last_raw_y = raw_y; s->last_raw_z = raw_z;

    /* raw velocity from unfiltered position delta */
    if (had_prev && dt > 0.001f) {
        *vx = (raw_x - prev_rx) / dt;
        *vy = (raw_y - prev_ry) / dt;
        *vz = (raw_z - prev_rz) / dt;
    } else {
        *vx = *vy = *vz = 0.0f;
    }

    /* position median filter (remove outliers) */
    pos_filter_apply(&s->pos_filter, x, y, z);

    /* α-β filter: predict with velocity, correct with measurement
       x̅ = x̂₋₁ + v̂₋₁·dt      (prediction)
       r  = z - x̅              (residual)
       x̂ = x̅ + α·r            (position update)
       v̂ = v̂₋₁ + (β/dt)·r     (velocity update) */
    if (!s->has_ab_state) {
        s->ax_x = *x;  s->ay_y = *y;  s->az_z = *z;
        s->ax_vx = *vx; s->ay_vy = *vy; s->az_vz = *vz;
        s->has_ab_state = 1;
    } else {
        /* predict */
        float px = s->ax_x + s->ax_vx * dt;
        float py = s->ay_y + s->ay_vy * dt;
        float pz = s->az_z + s->az_vz * dt;

        /* residual */
        float rx = *x - px;
        float ry = *y - py;
        float rz = *z - pz;

        /* update */
        s->ax_x   += POS_ALPHA * rx;
        s->ay_y   += POS_ALPHA * ry;
        s->az_z   += POS_ALPHA * rz;
        s->ax_vx  += (POS_BETA / dt) * rx;
        s->ay_vy  += (POS_BETA / dt) * ry;
        s->az_vz  += (POS_BETA / dt) * rz;
    }
    *x = s->ax_x;
    *y = s->ay_y;
    *z = s->az_z;

    /* store filtered position */
    s->last_x = *x; s->last_y = *y; s->last_z = *z;
    s->has_last_pos = 1;

    return 1;
}
