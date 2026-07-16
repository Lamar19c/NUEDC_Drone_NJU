/**
 * uwb_ekf.c — 常速模型 EKF，测距逐锚点顺序更新 (实现)
 *
 * CPU 预算 (F103@72MHz, soft-float 粗估):
 *   predict 利用 CV 结构 ~ 数百次浮点乘；每锚点标量 update ~ 百余次。
 *   每 20ms tick 最多 4 条更新，合计约 1~2k 次浮点运算，占用个位数百分比 CPU。
 *   若吃紧可降为 4 状态(px,py,vx,vy)、z 交给气压计/测距仪(见文末说明)。
 */
#include "uwb_ekf.h"
#include <math.h>     /* sqrtf */
#include <string.h>   /* memset */

/* ---- 默认调参 (可在 init 后按场地覆盖) ---- */
#define EKF_Q_ACC_XY_DEFAULT      8.0f    /* 水平机动 ~2m/s^2 量级 */
#define EKF_Q_ACC_Z_DEFAULT       0.5f    /* 垂直几何差，压小 */
#define EKF_R_RANGE_DEFAULT       0.04f   /* σ_range≈0.2m -> 0.04 m^2 */
#define EKF_GATE_SQ_DEFAULT       16.0f   /* 4σ 门限，兼顾 NLOS 剔除与鲁棒 */
#define EKF_HDOP_REF_SIGMA        0.15f   /* HDOP=σ_pos/0.15m */
#define EKF_MEAS_TIMEOUT_MS       200u
#define EKF_FIX_TIMEOUT_MS        300u
#define EKF_HEALTH_ANCHOR_MS      300u    /* 判"近期在线锚点"的窗口 */
#define EKF_POS_SIGMA_MAX         1.5f    /* 水平 σ 超过则判 fix 不可信 */
#define EKF_INIT_POS_VAR          25.0f   /* 初始位置方差 (5m)^2 */
#define EKF_INIT_VEL_VAR          4.0f    /* 初始速度方差 (2m/s)^2 */

void uwb_ekf_init(struct UWB_EKF *e, const float anchors[][3], int count) {
    memset(e, 0, sizeof(*e));
    e->anchors      = anchors;
    e->anchor_count = count;
    e->q_acc_xy       = EKF_Q_ACC_XY_DEFAULT;
    e->q_acc_z        = EKF_Q_ACC_Z_DEFAULT;
    e->r_range        = EKF_R_RANGE_DEFAULT;
    e->gate_sq        = EKF_GATE_SQ_DEFAULT;
    e->hdop_ref_sigma = EKF_HDOP_REF_SIGMA;
    e->meas_timeout_ms= EKF_MEAS_TIMEOUT_MS;
    e->fix_timeout_ms = EKF_FIX_TIMEOUT_MS;

    /* 初始状态：锚点质心 + 默认高度，速度 0，大协方差让量测拉进来 */
    float cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < count; i++) { cx += anchors[i][0]; cy += anchors[i][1]; }
    if (count > 0) { cx /= count; cy /= count; }
    e->x[0] = cx; e->x[1] = cy; e->x[2] = DEFAULT_HEIGHT;
    e->x[3] = e->x[4] = e->x[5] = 0.0f;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) e->P[i][j] = 0.0f;
    e->P[0][0] = e->P[1][1] = e->P[2][2] = EKF_INIT_POS_VAR;
    e->P[3][3] = e->P[4][4] = e->P[5][5] = EKF_INIT_VEL_VAR;
    e->initialized  = 0;
    e->last_pred_ms = 0;
}

/* ---- 预测：常速模型，P = F P F^T + Q，利用 F=[[I,dtI],[0,I]] 结构 ---- */
static void ekf_predict(struct UWB_EKF *e, float dt) {
    if (dt <= 0.0f) return;

    /* 状态推进 */
    e->x[0] += e->x[3] * dt;
    e->x[1] += e->x[4] * dt;
    e->x[2] += e->x[5] * dt;

    float (*P)[6] = e->P;

    /* Step1: 行 0..2 += dt * 行 3..5   (A = F P) */
    for (int j = 0; j < 6; j++) {
        P[0][j] += dt * P[3][j];
        P[1][j] += dt * P[4][j];
        P[2][j] += dt * P[5][j];
    }
    /* Step2: 列 0..2 += dt * 列 3..5   (P = A F^T) */
    for (int i = 0; i < 6; i++) {
        P[i][0] += dt * P[i][3];
        P[i][1] += dt * P[i][4];
        P[i][2] += dt * P[i][5];
    }

    /* 过程噪声 Q：分轴 piecewise-white-accel，(pos,vel) 对为 (0,3)(1,4)(2,5) */
    float dt2 = dt * dt, dt3 = dt2 * dt;
    const int pv[3][2] = { {0,3}, {1,4}, {2,5} };
    for (int a = 0; a < 3; a++) {
        float q = (a < 2) ? e->q_acc_xy : e->q_acc_z;
        int p = pv[a][0], v = pv[a][1];
        P[p][p] += q * dt3 / 3.0f;
        P[p][v] += q * dt2 / 2.0f;
        P[v][p] += q * dt2 / 2.0f;
        P[v][v] += q * dt;
    }
}

/* ---- 标量量测更新：单锚点距离 ----
 * 返回 1 = 已应用, 0 = 被门限剔除 */
static int ekf_update_range(struct UWB_EKF *e, int ai, float meas) {
    float dx = e->x[0] - e->anchors[ai][0];
    float dy = e->x[1] - e->anchors[ai][1];
    float dz = e->x[2] - e->anchors[ai][2];
    float r  = sqrtf(dx*dx + dy*dy + dz*dz);
    if (r < 1e-3f) return 0;          /* 退化，跳过 */

    float Hx = dx / r, Hy = dy / r, Hz = dz / r;   /* H 仅前三项非零 */
    float (*P)[6] = e->P;

    /* PHt (6x1) = P * H^T，H 只有前三列有效 */
    float PHt[6];
    for (int i = 0; i < 6; i++)
        PHt[i] = P[i][0]*Hx + P[i][1]*Hy + P[i][2]*Hz;

    float S = Hx*PHt[0] + Hy*PHt[1] + Hz*PHt[2] + e->r_range;
    if (S < 1e-9f) return 0;

    float innov = meas - r;

    /* innovation 门限：拒绝 NLOS/野值 */
    if (innov * innov > e->gate_sq * S) return 0;

    /* 增益 K = PHt / S，状态修正 */
    float invS = 1.0f / S;
    float K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = PHt[i] * invS;
        e->x[i] += K[i] * innov;
    }

    /* P = (I - K H) P。HP (1x6) = H * P，H 只前三行 */
    float HP[6];
    for (int j = 0; j < 6; j++)
        HP[j] = Hx*P[0][j] + Hy*P[1][j] + Hz*P[2][j];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            P[i][j] -= K[i] * HP[j];

    /* 对称化，抑制 soft-float 累积误差破坏对称性 */
    for (int i = 0; i < 6; i++)
        for (int j = i + 1; j < 6; j++) {
            float m = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = m;
        }
    return 1;
}

int uwb_ekf_step(struct UWB_EKF *e, const float distances[],
                 const uint32_t last_update_ms[], uint32_t now_ms) {
    /* 收集"新且未超时"的锚点，按时间戳升序应用 */
    int   order[8], n = 0;
    for (int i = 0; i < e->anchor_count && i < 8; i++) {
        if (distances[i] <= 0.0f || distances[i] >= 50.0f) continue;
        uint32_t ts = last_update_ms[i];
        if (ts == e->last_meas_ms[i]) continue;               /* 已消费过 */
        if ((now_ms - ts) >= e->meas_timeout_ms) continue;     /* 太旧 */
        order[n++] = i;
    }
    /* 插入排序 (n<=8) 按 last_update_ms 升序 */
    for (int a = 1; a < n; a++) {
        int k = order[a], b = a - 1;
        while (b >= 0 && last_update_ms[order[b]] > last_update_ms[k]) {
            order[b + 1] = order[b]; b--;
        }
        order[b + 1] = k;
    }

    if (!e->initialized) {
        if (n > 0) { e->initialized = 1; e->last_pred_ms = last_update_ms[order[0]]; }
        else return 0;
    }

    int applied = 0;
    for (int a = 0; a < n; a++) {
        int i = order[a];
        uint32_t ts = last_update_ms[i];
        float dt = 0.0f;
        if (ts > e->last_pred_ms) dt = (float)(ts - e->last_pred_ms) / 1000.0f;
        if (dt > 0.5f) dt = 0.5f;        /* 长间隔限幅，防协方差爆 */
        ekf_predict(e, dt);
        e->last_pred_ms = ts;

        if (ekf_update_range(e, i, distances[i])) {
            applied++;
            e->consec_reject = 0;
            e->last_good_update_ms = ts;
        } else {
            e->consec_reject++;
        }
        e->last_meas_ms[i] = ts;
    }
    e->last_step_anchor_cnt = n;

    /* 状态推进到 now，供随时读取 */
    if (now_ms > e->last_pred_ms) {
        float dt = (float)(now_ms - e->last_pred_ms) / 1000.0f;
        if (dt > 0.5f) dt = 0.5f;
        ekf_predict(e, dt);
        e->last_pred_ms = now_ms;
    }
    return applied;
}

void uwb_ekf_get(struct UWB_EKF *e, uint32_t now_ms,
                 float *x, float *y, float *z,
                 float *vx, float *vy, float *vz) {
    /* 若外部读取时刻晚于内部时刻，用速度外推一小步(不改内部状态) */
    float dt = 0.0f;
    if (now_ms > e->last_pred_ms) dt = (float)(now_ms - e->last_pred_ms) / 1000.0f;
    if (dt > 0.5f) dt = 0.5f;
    *x = e->x[0] + e->x[3] * dt;
    *y = e->x[1] + e->x[4] * dt;
    *z = e->x[2] + e->x[5] * dt;
    *vx = e->x[3]; *vy = e->x[4]; *vz = e->x[5];
}

void uwb_ekf_health(const struct UWB_EKF *e, uint32_t now_ms,
                    int *fix_ok, int *sats, float *hdop) {
    int anchors_recent = 0;
    for (int i = 0; i < e->anchor_count && i < 8; i++)
        if (e->last_meas_ms[i] != 0 &&
            (now_ms - e->last_meas_ms[i]) < EKF_HEALTH_ANCHOR_MS)
            anchors_recent++;

    float pos_var = e->P[0][0] + e->P[1][1];
    float pos_sigma = (pos_var > 0.0f) ? sqrtf(pos_var) : 0.0f;

    float hd = pos_sigma / e->hdop_ref_sigma;
    if (hd < 0.5f) hd = 0.5f;
    if (hd > 99.9f) hd = 99.9f;

    int fresh = e->initialized &&
                (now_ms - e->last_good_update_ms) < e->fix_timeout_ms;
    int ok = fresh && (anchors_recent >= 3) &&
             (pos_sigma < EKF_POS_SIGMA_MAX) &&
             (e->consec_reject < 8);

    if (fix_ok) *fix_ok = ok ? 1 : 0;
    if (hdop)   *hdop   = hd;
    if (sats) {
        /* 健康时给出与在线锚点数挂钩的伪卫星数，让地面站的最小卫星/HDOP
         * 检查在退化时能如实报警；不健康时压到很低。 */
        *sats = ok ? (4 + anchors_recent * 2) : (anchors_recent > 0 ? 3 : 0);
        if (*sats > 19) *sats = 19;
    }
}
