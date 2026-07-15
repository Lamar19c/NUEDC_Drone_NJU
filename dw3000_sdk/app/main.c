/**
 * main.c — NJU DW3000 UWB 定位程序 (Tag + Anchor 统一固件)
 *
 * 角色切换: ranging.h 中定义 UWB_ROLE_TAG 或 UWB_ROLE_ANCHOR
 *
 *   UWB_ROLE_TAG:    轮询锚点 → mc 输出 → stm32_uwb → FC
 *   UWB_ROLE_ANCHOR: 收 Poll → 回 Resp (改 g_anchor_id)
 */
#include "hardware/board.h"
#include "ranging.h"
#include <stdio.h>
#include <string.h>

#ifdef UWB_ROLE_TAG
/* ── EEPROM 配置 (仅 Tag 需要) ── */
#define EEP_OFFSET 0x00
typedef struct {
    char     magic[4];
    uint16_t tag_addr;
    uint8_t  ch, prf, rate, plen;
    uint16_t ant_dly_tx, ant_dly_rx;
    uint32_t crc;
} cfg_t;
static cfg_t g_cfg;

static void cfg_defaults(void) {
    memset(&g_cfg, 0, sizeof(g_cfg));
    memcpy(g_cfg.magic, "UWB", 3);
    g_cfg.ch = 5; g_cfg.prf = 2; g_cfg.rate = 1; g_cfg.plen = 3;
    g_cfg.ant_dly_tx = 16450; g_cfg.ant_dly_rx = 16450;
}
static void cfg_load(void) {
    extern int eep_read(uint16_t a, uint8_t *b, uint16_t n);
    extern int eep_write(uint16_t a, const uint8_t *b, uint16_t n);
    if (eep_read(EEP_OFFSET, (uint8_t *)&g_cfg, sizeof(g_cfg)) != HAL_OK
        || g_cfg.magic[0] != 'U') {
        printf("[CFG] EEPROM empty, defaults\r\n");
        cfg_defaults();
        eep_write(EEP_OFFSET, (uint8_t *)&g_cfg, sizeof(g_cfg));
    }
}
#endif

/* ================================================================
 * main
 * ================================================================ */
int main(void) {
    board_init();

    printf("\r\n========================================\r\n");
#ifdef UWB_ROLE_TAG
    printf("  NJU UWB TAG — DW3000 SS-TWR\r\n");
#else
    printf("  NJU UWB ANCHOR — DW3000 Resp\r\n");
#endif
    printf("  MCU: STM32F103CBT6 @ 72MHz\r\n");
    printf("========================================\r\n\r\n");

#ifdef UWB_ROLE_TAG
    cfg_load();
#endif

    ranging_init();

#ifdef UWB_ROLE_TAG
    printf("[APP] Ranging started. mc → UART1 115200\r\n\r\n");
    while (1) {
        ranging_loop();
        ranging_output_mc();
    }
#else
    printf("[APP] Listening for Poll frames...\r\n\r\n");
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    while (1) {
        ranging_loop();
    }
#endif
}

void EXTI0_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(DWM_IRQ_PIN);
}
