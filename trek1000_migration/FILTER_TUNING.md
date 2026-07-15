# TREK1000 滤波参数调优建议

## 背景

TREK1000 Tag 在 110K 模式下 mc 帧输出约 **3.57 Hz**（~280ms 间隔），
相比 JZM01 模块的 ~5 Hz（~200ms 间隔）更慢。

## 建议修改 (`stm32_uwb/Core/Inc/uwb_config.h`)

```c
// ── 修改 1: DIST_TIMEOUT_MS ──
// 原值: 150ms — 对 3.57Hz (280ms间隔) 太短，会导致锚点频繁超时失效
// 新值: 400ms — 容忍 1 帧丢帧 + 余量
#define DIST_TIMEOUT_MS   400    // 原: 150

// ── 修改 2: DIST_WINDOW_SIZE (可选) ──
// 原值: 8 — @3.57Hz 跨度 2.24s，可能感觉迟钝
// 新值: 5 — @3.57Hz 跨度 1.4s，折中平滑度和响应
#define DIST_WINDOW_SIZE  5      // 原: 8

// ── 以下参数保持不变 ──
#define MAX_DIST_JUMP     0.8f   // 距离跳变抑制 (m)
#define POS_WINDOW_SIZE   5      // 位置中值滤波窗口
#define DIST_ALPHA        0.35f  // 距离 EMA 低通系数
#define POS_ALPHA         0.30f  // α-β 位置更新权重
#define POS_BETA          0.05f  // α-β 速度更新权重
#define VEL_ALPHA         0.3f   // NMEA 速度 EMA 系数
```

## 调优原则

| 症状 | 调整方向 |
|------|---------|
| 位置更新慢、滞后 | 减小 DIST_WINDOW_SIZE, 增大 POS_ALPHA |
| 定位抖动大 | 增大 DIST_WINDOW_SIZE, 减小 POS_ALPHA |
| 锚点频繁失效 | 增大 DIST_TIMEOUT_MS |
| 野值干扰 | 减小 MAX_DIST_JUMP |

## 实飞调优流程

1. 先不改参数，飞一次看表现
2. 如果定位滞后明显 → 改 DIST_WINDOW_SIZE = 5
3. 如果锚点频繁失效 → 改 DIST_TIMEOUT_MS = 400
4. 如果抖动大 → 增大 DIST_ALPHA 到 0.5
5. 反复迭代直到满意
