/**
 * ranging.c — DW3000 UWB 测距 (Tag + Anchor 统一)
 *
 *   UWB_ROLE_TAG:    轮询锚点 → 收 Resp → 算距离 → mc 输出
 *   UWB_ROLE_ANCHOR: 收 Poll → 回 Resp (含 TS)
 */
#include "ranging.h"
#include "hardware/board.h"
#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_interface.h"

/* ── 外部硬件函数 ── */
extern void uart_fc_send(const uint8_t *buf, uint16_t len);
extern void led_run(uint8_t on);
extern void led_uwb(uint8_t on);
extern void led_fix(uint8_t on);

/* ================================================================
 * 公共配置
 * ================================================================ */
static dwt_config_t g_cfg = {
    .chan = 5, .rxCode = 9, .txCode = 9,
    .rxPAC = DWT_PAC8, .txPAC = DWT_PAC8,
    .nsSFD = 1, .dataRate = DWT_BR_6M8,
    .phrMode = DWT_PHRMODE_STD,
    .sfdTO = 129 + 32 - 8,
    .stsMode = DWT_STS_MODE_OFF,
    .preambleLen = DWT_PLEN_128,
};

static uint8_t g_tx[128], g_rx[128];

/* ================================================================
 * 公共初始化
 * ================================================================ */
void ranging_init(void) {
    printf("[RNG] Reset DW3000...\r\n");
    dw3000_reset();

    int r = dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID
        | DWT_READ_OTP_BAT | DWT_READ_OTP_TMP
        | DWT_READ_OTP_LDO | DWT_SFDTOC_DISABLE);
    if (r != DWT_SUCCESS) {
        printf("[RNG] FAIL: dwt_initialise=%d\r\n", r);
        while (1) { led_fix(1); HAL_Delay(200); led_fix(0); HAL_Delay(200); }
    }

    uint32_t devid = dwt_readdevid();
    printf("[RNG] DEVID=0x%08lX  ver=%s\r\n", devid, dwt_getversionstr());

    dwt_configure(&g_cfg);
    dwt_setrxantennadelay(16450);
    dwt_settxantennadelay(16450);
    dwt_setleds(DWT_LEDS_ENABLE);
    dwt_setautorxreenable(1);
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_BEACON_EN);

    led_run(1);

#ifdef UWB_ROLE_TAG
    printf("[RNG] Role: TAG  (poll anchors)\r\n");
#else
    printf("[RNG] Role: ANCHOR  (listen + respond)\r\n");
#endif
    printf("[RNG] Ready.\r\n");
}

/* ================================================================
 * 帧构造工具
 * ================================================================ */
static uint16_t build_hdr(uint8_t *buf, uint8_t ftype, uint8_t addr) {
    uint8_t *p = buf;
    *p++ = 0x41; *p++ = 0x88;      /* Frame Control */
    *p++ = ftype & 7;               /* Sequence ← frame type */
    *p++ = 0xCA; *p++ = 0xDE;      /* PAN ID */
    *p++ = 0;    *p++ = 0;          /* Dst addr (broadcast) */
    *p++ = addr; *p++ = 0;          /* Src addr */
    *p++ = ftype;                   /* Payload: frame type marker */
    *p++ = addr;                    /* Payload: sender address */
    return (uint16_t)(p - buf);
}

/* ================================================================
 * TAG 模式
 * ================================================================ */
#ifdef UWB_ROLE_TAG

static float    g_dists[NUM_ANCHORS];
static uint32_t g_valid;

typedef enum { T_IDLE, T_TX_POLL, T_RX_RESP } t_state_t;
static t_state_t g_st = T_IDLE;
static uint8_t   g_anchor;

static void tag_tx_poll(uint8_t aid) {
    uint16_t len = build_hdr(g_tx, FRAME_POLL, TAG_ADDR);
    dwt_writetxdata(len, g_tx, 0);
    dwt_writetxfctrl(len, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    g_st = T_TX_POLL;
}

void ranging_loop(void) {
    switch (g_st) {

    case T_IDLE: {
        static uint32_t next;
        if (HAL_GetTick() < next) return;
        next = HAL_GetTick() + 20;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        HAL_Delay(1);
        tag_tx_poll(g_anchor);
        led_uwb(1);
        break;
    }

    case T_TX_POLL:
        if (dwt_read32bitreg(SYS_STATUS_ID) & DWT_INT_TXFRS_BIT_MASK) {
            dwt_write32bitreg(SYS_STATUS_ID, DWT_INT_TXFRS_BIT_MASK);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            g_st = T_RX_RESP;
        }
        break;

    case T_RX_RESP: {
        uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);
        if (st & DWT_INT_RXFCG_BIT_MASK) {
            dwt_write32bitreg(SYS_STATUS_ID, DWT_INT_RXFCG_BIT_MASK);
            uint16_t len = (uint16_t)dwt_read32bitreg(RX_FINFO_ID) & 0x3FF;
            uint8_t ts_resp[5], ts_anc[5];
            uint8_t ftype = 0, aid = 0;
            if (len > 9 && len <= sizeof(g_rx)) {
                dwt_readrxdata(g_rx, len, 0);
                ftype = g_rx[9];      /* frame type */
                aid   = g_rx[10];     /* anchor id */
                dwt_readrxtimestamp(ts_resp);
                memcpy(ts_anc, &g_rx[11], 5);
            }
            if (ftype == FRAME_RESP && aid == g_anchor) {
                double tr, ta;
                dwt_convertrxtimestamp(ts_resp, &tr);
                dwt_convertrxtimestamp(ts_anc, &ta);
                double tof = tr - ta;
                if (tof < 0) tof = 0;
                g_dists[aid] = (float)(tof * 149896229.0);  /* c/2 */
                g_valid |= (1 << aid);
                led_fix(1);
            }
            led_uwb(0);
            g_anchor = (g_anchor + 1) % NUM_ANCHORS;
            g_st = T_IDLE;
        } else if (st & (DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK)) {
            dwt_write32bitreg(SYS_STATUS_ID, st & (DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK));
            g_valid &= ~(1 << g_anchor);
            led_uwb(0);
            g_anchor = (g_anchor + 1) % NUM_ANCHORS;
            g_st = T_IDLE;
        }
        break;
    }
    }
}

void ranging_output_mc(void) {
    static uint32_t last;
    uint32_t now = HAL_GetTick();
    if (now - last < 280) return;
    last = now;
    uint32_t mm[4];
    for (int i = 0; i < NUM_ANCHORS; i++)
        mm[i] = (uint32_t)(g_dists[i] * 1000.0f);
    char buf[100];
    int len = snprintf(buf, sizeof(buf),
        "mc %01lx %08lx %08lx %08lx %08lx 0000 01 00000000 t0:0\r\n",
        g_valid & 0x0FUL, mm[0], mm[1], mm[2], mm[3]);
    uart_fc_send((uint8_t *)buf, (uint16_t)len);
    led_fix(0);
}

void ranging_get_distances(float d[4]) { memcpy(d, g_dists, sizeof(g_dists)); }

#endif /* UWB_ROLE_TAG */

/* ================================================================
 * ANCHOR 模式
 * ================================================================ */
#ifdef UWB_ROLE_ANCHOR

static uint8_t g_anchor_id = 1;   /* 修改此值区分 Anchor 1-4 */

void ranging_loop(void) {
    static uint8_t ts_poll[5], ts_resp[5];
    uint32_t st = dwt_read32bitreg(SYS_STATUS_ID);

    if (st & DWT_INT_RXFCG_BIT_MASK) {
        dwt_write32bitreg(SYS_STATUS_ID, DWT_INT_RXFCG_BIT_MASK);

        uint16_t len = (uint16_t)dwt_read32bitreg(RX_FINFO_ID) & 0x3FF;
        uint8_t  ftype = 0, src = 0;
        if (len > 9 && len <= sizeof(g_rx)) {
            dwt_readrxdata(g_rx, len, 0);
            ftype = g_rx[9];
            src   = g_rx[10];
        }
        dwt_readrxtimestamp(ts_poll);   /* Poll 到达时间 */
        led_uwb(1);

        if (ftype == FRAME_POLL && src == TAG_ADDR) {
            /* 延迟一小段时间 → 准备 Resp */
            HAL_Delay(1);
            dwt_readrxtimestamp(ts_poll);   /* 重新获取精确时间戳 */

            uint16_t hdr_len = build_hdr(g_tx, FRAME_RESP, g_anchor_id);
            /* 附加 Anchor 时间戳到 payload */
            uint8_t *pp = g_tx + hdr_len;
            memcpy(pp, ts_poll, 5);  pp += 5;
            dwt_readtxtimestamp(ts_resp);
            memcpy(pp, ts_resp, 5);  pp += 5;
            uint16_t total = (uint16_t)(pp - g_tx);

            dwt_writetxdata(total, g_tx, 0);
            dwt_writetxfctrl(total, 0, 0);
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

            /* 等 TX 完成 */
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & DWT_INT_TXFRS_BIT_MASK));
            dwt_write32bitreg(SYS_STATUS_ID, DWT_INT_TXFRS_BIT_MASK);
            led_uwb(0);
        }
        /* 重新使能接收 */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    /* RX 超时 → 重新使能 */
    if (st & (DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK))
        dwt_write32bitreg(SYS_STATUS_ID, st & (DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK));
}

#endif /* UWB_ROLE_ANCHOR */
