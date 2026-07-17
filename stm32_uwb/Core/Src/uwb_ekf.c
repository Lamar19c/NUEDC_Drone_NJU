/**
 * uwb_ekf.c — 常速模型 EKF，测距逐锚点顺序更新 (实现)
 *
 * ============================ 现场标定清单 ============================
 * 上机时按顺序调这几个量，均在下方 "默认调参" 宏定义处修改。
 *
 * 1. 固定高度 FIXED_HEIGHT_M (在 main.c)
 *    量一下 tag 实际离地高度，填进去。本布站锚点垂直几乎共面，z 不可观测，
 *    必须固定。高度偏 0.5m 只带来 xy 2~5cm 误差，不必很精确。
 *
 * 2. r_range (测距量测方差, m^2) = σ_range^2
 *    让 tag 静止，读串口 raw_d，估四个距离各自的标准差 σ。
 *    典型 DW1000 LOS 下 σ≈0.05~0.10m，则 r_range 填 0.0025~0.01。
 *    偏大 -> 轨迹发钝/滞后；偏小 -> 抖、易被野值带偏。
 *
 * 3. q_acc_xy (水平加速度过程噪声 PSD, m^2/s^3)
 *    按预期最大机动定。实测：室内慢速/定点用 8 会让速度状态跟着测距噪声
 *    乱窜、静止反而抖(可达 15~40cm)，故默认改为 1.5。手持慢走/温和飞行
 *    1~2；常规机动 8；激进急转 16~40。偏小 -> 快速运动滞后甚至被门限拒;
 *    偏大 -> 静止发抖。判据：静止时若 vx/vy 明显非零，就是 q 偏大。
 *
 * 4. gate_sq (innovation 卡方门限, 无量纲)
 *    9≈3σ, 16≈4σ, 36≈6σ。NLOS/多径严重的场地调大(放宽)避免误拒好点;
 *    但太大则野值进得来。默认 16。若串口频繁看到 fix 掉、anc 数偏低，
 *    多半是门限太严把测量拒了，先放到 36 试。
 *
 * 5. hdop_ref_sigma (HDOP 归一化参考 σ, m)
 *    只影响报给飞控的 HDOP 数值大小，不影响定位。收敛后串口看 hdop,
 *    调这个让健康 HDOP 落到明显低于 ArduPilot GPS_HDOP_GOOD(默认1.4)、
 *    留出裕度(如 0.7~0.9)。默认 0.15。
 *
 * 6. meas_timeout_ms / fix_timeout_ms
 *    meas_timeout 略大于一个测距轮询周期(本系统约100ms)即可，默认200。
 *    fix_timeout 是多久没有效更新就判失锁，默认300ms。
 *
 * 上机自检：串口每帧会打印 fix/hdop/anc(见 main.c)。正常应是
 * fix=1、anc=4、hdop 稳定在小值；遮挡一个锚点 anc 掉到 3 仍 fix=1、
 * hdop 略升；遮挡到 <3 或全遮，应在 300ms 内 fix=0、hdop 飙升。
 * ====================================================================
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
#define EKF_Q_ACC_XY_DEFAULT      1.5f    /* 室内慢速默认；激进飞行再上调到 8~16 */
#define EKF_Q_ACC_Z_DEFAULT       0.5f    /* 垂直几何差，压小 */
#define EKF_R_RANGE_DEFAULT       0.04f   /* σ_range≈0.2m -> 0.04 m^2 */
#define EKF_GATE_SQ_DEFAULT       36.0f   /* 6σ 门限，放宽避免初始收敛时拒测 */
#define EKF_HDOP_REF_SIGMA        1.00f   /* HDOP=σ_pos/1.0m，收敛后<1，对齐 GPS_HDOP_GOOD=1.4 */
#define EKF_MEAS_TIMEOUT_MS       200u
#define EKF_FIX_TIMEOUT_MS        300u
#define EKF_HEALTH_ANCHOR_MS      300u    /* 判"近期在线锚点"的窗口 */
#define EKF_POS_SIGMA_MAX         1.5f    /* 水平 σ 超过则判 fix 不可信 */
#define EKF_INIT_POS_VAR          1.0f    /* 初始位置方差 (1m)^2，冷启动HDOP≈1.4 */
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

void uwb_ekf_set_fixed_z(struct UWB_EKF *e, float z) {
    e->z_fixed = 1;
    e->z_ext   = z;
    e->x[2]    = z;
    e->x[5]    = 0.0f;
    e->q_acc_z = 0.0f;              /* z 不再有过程噪声 */
    /* 断开 z、vz 与其它状态的协方差耦合，防止泄漏 */
    for (int j = 0; j < 6; j++) {
        e->P[2][j] = e->P[j][2] = 0.0f;
        e->P[5][j] = e->P[j][5] = 0.0f;
    }
}

void uwb_ekf_update_height(struct UWB_EKF *e, float z) {
    if (e->z_fixed) { e->z_ext = z; e->x[2] = z; e->x[5] = 0.0f; }
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
    float zc = e->z_fixed ? e->z_ext : e->x[2];   /* 固定模式用外部高度算残差 */
    float dx = e->x[0] - e->anchors[ai][0];
    float dy = e->x[1] - e->anchors[ai][1];
    float dz = zc      - e->anchors[ai][2];
    float r  = sqrtf(dx*dx + dy*dy + dz*dz);
    if (r < 1e-3f) return 0;          /* 退化，跳过 */

    float Hx = dx / r, Hy = dy / r;
    float Hz = e->z_fixed ? 0.0f : (dz / r);   /* 固定高度时不把残差摊到 z */
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
    for (int i = 0; i < 6; i++) K[i] = PHt[i] * invS;
    if (e->z_fixed) { K[2] = 0.0f; K[5] = 0.0f; }   /* 冻结 z、vz */
    for (int i = 0; i < 6; i++) e->x[i] += K[i] * innov;

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
    if (e->z_fixed) { *z = e->z_ext; *vz = 0.0f; }
    else            { *z = e->x[2] + e->x[5] * dt; *vz = e->x[5]; }
    *vx = e->x[3]; *vy = e->x[4];
}

int uwb_ekf_online_anchors(const struct UWB_EKF *e, uint32_t now_ms) {
    int n = 0;
    for (int i = 0; i < e->anchor_count && i < 8; i++)
        if (e->last_meas_ms[i] != 0 &&
            (now_ms - e->last_meas_ms[i]) < EKF_HEALTH_ANCHOR_MS)
            n++;
    return n;
}

void uwb_ekf_health(const struct UWB_EKF *e, uint32_t now_ms,
                    int *fix_ok, int *sats, float *hdop) {
    int anchors_recent = uwb_ekf_online_anchors(e, now_ms);

    float pos_var = e->P[0][0] + e->P[1][1];
    float pos_sigma = (pos_var > 0.0f) ? sqrtf(pos_var) : 0.0f;

    float hd = pos_sigma / e->hdop_ref_sigma;
    if (hd < 0.5f) hd = 0.5f;
    if (hd > 99.9f) hd = 99.9f;

    int fresh = e->initialized &&
                (now_ms - e->last_good_update_ms) < e->fix_timeout_ms;
    /* GGA 定位有效位：有新鲜、>=3 锚点的解就报有效(1)。不再用 pos_sigma/reject
     * 阈值去翻转它——那些瞬时波动会让 fix 反复掉 0、飞控卡在"未定位"。
     * 定位的好坏由 HDOP 表达，交给飞控加权/解锁判断。 */
    int ok = fresh && (anchors_recent >= 3);

    if (fix_ok) *fix_ok = ok ? 1 : 0;
    if (hdop)   *hdop   = hd;
    if (sats) {
        /* 健康时给出与在线锚点数挂钩的伪卫星数，让地面站的最小卫星/HDOP
         * 检查在退化时能如实报警；不健康时压到很低。 */
        *sats = ok ? (4 + anchors_recent * 2) : (anchors_recent > 0 ? 3 : 0);
        if (*sats > 19) *sats = 19;
    }
}
