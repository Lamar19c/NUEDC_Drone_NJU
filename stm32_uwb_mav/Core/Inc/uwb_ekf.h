/**
 * uwb_ekf.h — 常速(CV)模型 EKF，测距逐锚点顺序更新
 *
 * 替代原 snapshot 加权最小二乘。核心区别：
 *   - 状态 [px,py,pz,vx,vy,vz]，常速过程模型带过程噪声
 *   - 每条距离报文按自己的时间戳单独做一次标量更新(sequential update)，
 *     predict 先推进到该测量时刻，从根本上消除"顺序测距时间偏斜"造成的
 *     随速度增大的定位误差
 *   - innovation 门限自带外点(NLOS 跳变)剔除，取代距离中值/跳变限幅
 *   - 协方差可直接导出 HDOP 和健康标志，供 NMEA 如实反映定位质量
 *
 * 目标平台 STM32F103 (Cortex-M3, 无硬件 FPU)，故用标量更新避免矩阵求逆，
 * 6x6 预测利用 CV 结构手写，soft-float 下开销可控 (见 .c 内注释)。
 */
#ifndef UWB_EKF_H
#define UWB_EKF_H

#include "uwb_config.h"
#include <stdint.h>

struct UWB_EKF {
    const float (*anchors)[3];
    int   anchor_count;

    /* 状态与协方差 */
    float x[6];          /* px,py,pz,vx,vy,vz */
    float P[6][6];
    int   initialized;
    uint32_t last_pred_ms;          /* 状态当前所处时刻 */
    uint32_t last_meas_ms[8];       /* 每锚点已消费的测量时间戳 */

    /* 调参 */
    float q_acc_xy;      /* 水平加速度过程噪声功率谱密度 (m^2/s^3) */
    float q_acc_z;       /* 垂直加速度过程噪声 PSD，几何差建议调小 */
    float r_range;       /* 单次测距量测方差 (m^2)，≈ σ_range^2 */
    float gate_sq;       /* innovation 门限 (卡方，3σ≈9) */
    float hdop_ref_sigma;/* HDOP 归一化参考 σ (m)，用来对齐地面站阈值 */
    uint32_t meas_timeout_ms;   /* 距离超时，超过则不喂入 */
    uint32_t fix_timeout_ms;    /* 多久没有有效更新就判为 fix 丢失 */

    /* 健康度 */
    uint32_t last_good_update_ms;
    int      consec_reject;
    int      last_step_anchor_cnt;

    /* 固定高度模式：z 不由测距估计(垂直几何差、几乎不可观测)，
     * 改由外部(气压计/测距仪/常值)给定。启用后 x/y 仍正常估计。 */
    int   z_fixed;
    float z_ext;
};

void uwb_ekf_init(struct UWB_EKF *e, const float anchors[][3], int count);

/* 启用固定高度模式并设定初始高度 (强烈建议本布站使用) */
void uwb_ekf_set_fixed_z(struct UWB_EKF *e, float z);

/* 固定高度模式下，每周期用外部高度源(气压计/测距仪)刷新 z */
void uwb_ekf_update_height(struct UWB_EKF *e, float z);

/**
 * 喂入当前每锚点的距离快照 + 各自时间戳。函数内部只消费"比上次更新"的
 * 测量，并按时间戳升序逐条做 predict+update，因此非同时测距被正确处理。
 * distances[i] <=0 或 >=50 视为无效；last_update_ms[i] 为该锚点最近一次
 * 距离的时间戳(HAL_GetTick)。返回本次实际应用的测量条数。
 */
int  uwb_ekf_step(struct UWB_EKF *e, const float distances[],
                  const uint32_t last_update_ms[], uint32_t now_ms);

/* 读取推进到 now_ms 的状态(不改动内部状态) */
void uwb_ekf_get(struct UWB_EKF *e, uint32_t now_ms,
                 float *x, float *y, float *z,
                 float *vx, float *vy, float *vz);

/* 供 NMEA 使用的健康度：fix_ok(0/1)、伪卫星数、HDOP */
void uwb_ekf_health(const struct UWB_EKF *e, uint32_t now_ms,
                    int *fix_ok, int *sats, float *hdop);

/* 当前在线锚点数(近期有测量的锚点)，供调试打印观察健康度 */
int  uwb_ekf_online_anchors(const struct UWB_EKF *e, uint32_t now_ms);

#endif /* UWB_EKF_H */
