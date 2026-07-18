/**
 * mavlink.h — Minimal handwritten MAVLink v1 protocol for STM32F103
 *
 * Only implements HEARTBEAT (#0) + VISION_POSITION_ESTIMATE (#102).
 * No library dependency — ~300 lines, hand-checked field layouts.
 *
 * Usage:
 *   1. Call mavlink_init() once with your UART handle.
 *   2. Call mavlink_send_heartbeat() at 1 Hz.
 *   3. Call mavlink_send_vision_position() at desired update rate.
 */
#ifndef MAVLINK_H
#define MAVLINK_H

#include <stdint.h>

/* Forward-declare HAL UART handle (avoid pulling in full HAL in header) */
struct __UART_HandleTypeDef;
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

/* ---- System IDs ---- */
#define MAVLINK_SYSID          1
#define MAVLINK_COMPID         191    /* MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY */

/* ---- MAVLink v1 wire constants ---- */
#define MAVLINK_V1_MAGIC       0xFE
#define MAVLINK_V1_HEADER_LEN  6
#define MAVLINK_V1_CRC_LEN     2
#define MAVLINK_MAX_PAYLOAD    255

/* ---- Message IDs ---- */
#define MAV_MSG_HEARTBEAT                    0
#define MAV_MSG_VISION_POSITION_ESTIMATE   102

/* ---- Enums (values needed for HEARTBEAT) ---- */
#define MAV_TYPE_ONBOARD_CONTROLLER  18
#define MAV_AUTOPILOT_INVALID         8
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  0x01
#define MAV_STATE_ACTIVE              4

/* ---- API ---- */

/**
 * Initialise MAVLink output to use the given UART handle.
 * Call once at startup.
 */
void mavlink_init(UART_HandleTypeDef *huart);

/**
 * Pack and send a HEARTBEAT.  Call at 1 Hz.
 * Returns 0 on success, -1 if UART busy.
 */
int mavlink_send_heartbeat(uint32_t now_ms);

/**
 * Pack and send VISION_POSITION_ESTIMATE.
 *
 *   x, y, z       — position in NED frame (m)
 *   vx, vy, vz    — velocity in NED frame (m/s); pass 0 if unknown
 *   heading_deg   — yaw angle (deg, 0=North, 90=East); pass 0 if unknown
 *   now_us        — timestamp in microseconds
 *
 * Returns 0 on success, -1 if UART busy.
 */
int mavlink_send_vision_position(float x, float y, float z,
                                 float vx, float vy, float vz,
                                 float heading_deg, uint64_t now_us);

#endif /* MAVLINK_H */
