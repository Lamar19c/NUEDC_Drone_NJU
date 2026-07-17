/**
 * mavlink.c — Minimal handwritten MAVLink v1 protocol implementation
 *
 * STM32F103 — no heap, no C library dependency beyond what HAL provides.
 */
#include "mavlink.h"
#include "usart.h"        /* UART_HandleTypeDef */
#include "stm32f1xx_hal.h" /* HAL_UART_Transmit, HAL_GetTick */
#include <string.h>       /* memcpy */

/* ---- CRC16-CCITT (XMODEM) lookup table for MAVLink v1 ---- */
static const uint16_t crc16_table[256] = {
0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0 };

static uint16_t crc16_update(uint16_t crc, uint8_t byte) {
    return (uint16_t)((crc << 8) ^ crc16_table[((crc >> 8) ^ byte) & 0xFF]);
}

static uint16_t crc16_buffer(const uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) crc = crc16_update(crc, buf[i]);
    return crc;
}

/* ---- Packing helpers (little-endian, native on Cortex-M3) ---- */

static void _put_u8(uint8_t *buf, int *pos, uint8_t v)  { buf[(*pos)++] = v; }
static void _put_u16(uint8_t *buf, int *pos, uint16_t v) { memcpy(&buf[*pos], &v, 2); *pos += 2; }
static void _put_u32(uint8_t *buf, int *pos, uint32_t v) { memcpy(&buf[*pos], &v, 4); *pos += 4; }
static void _put_u64(uint8_t *buf, int *pos, uint64_t v) { memcpy(&buf[*pos], &v, 8); *pos += 8; }
static void _put_f32(uint8_t *buf, int *pos, float v)    { memcpy(&buf[*pos], &v, 4); *pos += 4; }

/* ---- Internal state ---- */
static UART_HandleTypeDef *g_mav_huart = NULL;
static uint8_t  g_seq  = 0;         /* rolling sequence number */
static uint8_t  g_txbuf[64];        /* max payload in this build is 32 bytes */
static uint32_t g_last_tx_ms = 0;

/* ---- Initialise ---- */
void mavlink_init(UART_HandleTypeDef *huart) {
    g_mav_huart = huart;
    g_seq = 0;
}

/* ---- Build and send a MAVLink v1 frame ---- */
static int mavlink_send_frame(uint8_t msgid, const uint8_t *payload, int payload_len) {
    if (!g_mav_huart) return -1;

    /* Build frame: magic + header + payload + CRC */
    uint8_t buf[MAVLINK_MAX_PAYLOAD + 9];  /* 1 magic + 6 header + 255 payload + 2 CRC */
    int pos = 0;

    /* Magic */
    buf[pos++] = MAVLINK_V1_MAGIC;

    /* Payload length */
    buf[pos++] = (uint8_t)(payload_len & 0xFF);

    /* Sequence */
    buf[pos++] = g_seq++;

    /* System ID + Component ID */
    buf[pos++] = MAVLINK_SYSID;
    buf[pos++] = MAVLINK_COMPID;

    /* Message ID */
    buf[pos++] = msgid;

    /* Payload */
    for (int i = 0; i < payload_len; i++) buf[pos++] = payload[i];

    /* CRC16 over header+payload (bytes 1..pos-1) */
    uint16_t crc = crc16_buffer(&buf[1], pos - 1);
    buf[pos++] = (uint8_t)(crc & 0xFF);
    buf[pos++] = (uint8_t)((crc >> 8) & 0xFF);

    /* Non-blocking: skip if UART DMA still busy; a dropped frame is better
       than corrupting the previous one or blocking EKF loop. */
    if (g_mav_huart->gState == HAL_UART_STATE_BUSY_TX) return -1;

    HAL_StatusTypeDef rc = HAL_UART_Transmit(
        g_mav_huart, buf, (uint16_t)pos, 5);   /* 5ms timeout = enough for 64 bytes @57600 */
    g_last_tx_ms = HAL_GetTick();
    return (rc == HAL_OK) ? 0 : -1;
}

/* ---- HEARTBEAT (msgid=0, payload=9) ---- */
int mavlink_send_heartbeat(uint32_t now_ms) {
    (void)now_ms;
    uint8_t p[9];
    int i = 0;
    _put_u8 (p, &i, MAV_TYPE_ONBOARD_CONTROLLER);         /* type */
    _put_u8 (p, &i, MAV_AUTOPILOT_INVALID);               /* autopilot */
    _put_u8 (p, &i, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);   /* base_mode */
    _put_u32(p, &i, 0);                                    /* custom_mode */
    _put_u8 (p, &i, MAV_STATE_ACTIVE);                     /* system_status */
    _put_u8 (p, &i, 3);                                    /* mavlink_version */
    return mavlink_send_frame(MAV_MSG_HEARTBEAT, p, i);
}

/* ---- VISION_POSITION_ESTIMATE (msgid=102, payload=32) ---- */
int mavlink_send_vision_position(float x, float y, float z,
                                 float vx, float vy, float vz,
                                 float heading_deg, uint64_t now_us) {
    uint8_t p[40];
    int i = 0;
    _put_u64(p, &i, now_us);       /* usec */
    _put_f32(p, &i, x);            /* x (m, NED) */
    _put_f32(p, &i, y);            /* y */
    _put_f32(p, &i, z);            /* z */
    _put_f32(p, &i, 0.0f);                             /* roll  — unknown */
    _put_f32(p, &i, 0.0f);                             /* pitch — unknown */
    _put_f32(p, &i, heading_deg * 0.01745329252f);    /* yaw   — rad from North */

    return mavlink_send_frame(MAV_MSG_VISION_POSITION_ESTIMATE, p, i);
}
