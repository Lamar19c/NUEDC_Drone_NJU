/**
 * ranging.h — DW3000 UWB 测距 (Tag + Anchor 统一固件)
 *
 * 角色选择: 编译时定义 UWB_ROLE 为 "TAG" 或 "ANCHOR"
 *   #define UWB_ROLE_TAG     // → Tag (无人机)
 *   #define UWB_ROLE_ANCHOR  // → Anchor (地面)
 *
 * Tag:  轮询 4 锚点 SS-TWR → 输出 mc 帧 → stm32_uwb
 * Anchor: 监听 Poll → 回复 Resp (含时间戳)
 */
#ifndef RANGING_H
#define RANGING_H

#include <stdint.h>

/* ── 角色选择 (二选一) ── */
//#define UWB_ROLE_TAG
#define UWB_ROLE_ANCHOR

#ifndef UWB_ROLE_TAG
#ifndef UWB_ROLE_ANCHOR
#error "Must define UWB_ROLE_TAG or UWB_ROLE_ANCHOR"
#endif
#endif

#define NUM_ANCHORS     4

/* 帧类型 */
#define FRAME_POLL      0x01
#define FRAME_RESP      0x02
#define FRAME_FINAL     0x03

/* 地址 */
#define TAG_ADDR        0x00

/* ── 公共 API ── */
void ranging_init(void);
void ranging_loop(void);

#ifdef UWB_ROLE_TAG
void ranging_output_mc(void);
void ranging_get_distances(float d[4]);
#endif

#endif
