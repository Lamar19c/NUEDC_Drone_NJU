/**
 * main.c — UWB 室内 3D 定位 (STM32F103C8T6 + HAL)
 *
 * 串口:
 *   USART1 (PA9/PA10)  — printf 调试输出 (115200)
 *   USART2 (PA2/PA3)   — JZM01 $DIST 接收 (19200, DMA+空闲中断)
 *   USART3 (PB10/PB11) — NMEA 输出 → 飞控 GPS 口 (57600)
 */

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>

#include "uwb_config.h"
#include "uwb_solver.h"
#include "uwb_nmea.h"

/* ---- printf 重定向到 USART1 ---- */
int _write(int fd, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ---- USART2 DMA 环形接收 ---- */
#define RX2_BUF_SIZE  256
static uint8_t  rx2_buf[RX2_BUF_SIZE];
static volatile uint16_t rx2_len = 0;
static volatile uint8_t  rx2_ready = 0;

/* ---- 全局对象 (栈分配, 无堆) ---- */
static struct UWB_Parser      parser;
static struct DistanceFilter  dist_filt;
static struct UWBSolver       solver;
static struct NMEA_Generator  nmea;

/* ---- 位置缓存 ---- */
static float gx = 0.0f, gy = 0.0f, gz = 1.0f;
static float gvx = 0.0f, gvy = 0.0f;
static int   has_pos = 0;

/* ---- USART2 空闲中断回调 ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart->Instance == USART2) {
        rx2_len = size;
        rx2_ready = 1;
    }
}

/* ---- 外设初始化 (由 CubeMX 生成独立 .c 文件时删除以下 stub) ---- */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    /* ---- 初始化所有模块 ---- */
    uwb_parser_init(&parser);
    dist_filter_init(&dist_filt);
    uwb_solver_init(&solver, ANCHOR_POSITIONS, ANCHOR_COUNT);
    nmea_gen_init(&nmea);

    /* ---- 启动 USART2 DMA + 空闲中断接收 ---- */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, RX2_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    printf("\r\n========================================\r\n");
    printf("  UWB 室内 3D 定位 — STM32F103C8T6\r\n");
    printf("  (双中值滤波)\r\n");
    printf("========================================\r\n");
    printf("锚点数量: %d\r\n", ANCHOR_COUNT);
    for (int i = 0; i < ANCHOR_COUNT; i++) {
        printf("  S%d: (%.2f, %.2f, %.2f)\r\n",
               i + 1, ANCHOR_POSITIONS[i][0], ANCHOR_POSITIONS[i][1], ANCHOR_POSITIONS[i][2]);
    }
    if (ANCHOR_COUNT == 3) {
        printf("3 锚点模式: Z 固定 = %.2fm\r\n", DEFAULT_HEIGHT);
    }
    printf("GPS 原点: lat=%ld lon=%ld alt=%ld\r\n",
           (long)GPS_ORIGIN_LAT, (long)GPS_ORIGIN_LON, (long)GPS_ORIGIN_ALT);
    printf("等待 UWB 数据...\r\n\r\n");

    /* ---- 主循环 ---- */
    uint32_t last_nmea_time = 0;
    uint32_t nmea_count = 0;
    uint32_t loop_count = 0;
    const uint32_t nmea_interval_ms = (uint32_t)(1000.0f / NMEA_RATE_HZ);

    while (1) {
        /* ---- Step 1: 处理 DMA 接收到的 UWB 数据 ---- */
        if (rx2_ready) {
            rx2_ready = 0;
            for (uint16_t i = 0; i < rx2_len; i++) {
                uwb_parser_feed(&parser, (char)rx2_buf[i]);
            }
            /* 重新启动 DMA 接收 */
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, RX2_BUF_SIZE);
        }

        /* ---- Step 2: 有足够锚点 → 解算 ---- */
        if (uwb_parser_valid_count(&parser) >= 3) {
            float distances[ANCHOR_COUNT];
            uwb_parser_get_distances(&parser, distances, ANCHOR_COUNT);
            uwb_parser_clear_distances(&parser);

            /* 距离中值滤波 */
            dist_filter_apply(&dist_filt, distances, ANCHOR_COUNT);

            /* 三边求解 + 位置中值滤波 + 速度 */
            float x, y, z, vx, vy, vz;
            if (uwb_solver_solve(&solver, distances, HAL_GetTick(),
                                 &x, &y, &z, &vx, &vy, &vz)) {
                gx = x; gy = y; gz = z; gvx = vx; gvy = vy;
                has_pos = 1;
                loop_count++;

                /* 调试输出 */
                int32_t lat_e7, lon_e7, alt_mm;
                local_to_gps(x, y, z, GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
                             &lat_e7, &lon_e7, &alt_mm);

                printf("[%lu] x=%.2f y=%.2f z=%.2f  vx=%.2f vy=%.2f vz=%.2f  "
                       "GPS: %.7f,%.7f,%.2fm\r\n",
                       (unsigned long)(HAL_GetTick() / 1000UL),
                       x, y, z, vx, vy, vz,
                       (double)(lat_e7 / 1e7f), (double)(lon_e7 / 1e7f),
                       (double)(alt_mm / 1000.0f));
            }
        }

        /* ---- Step 3: NMEA 输出 (5Hz 定时) ---- */
        uint32_t now = HAL_GetTick();
        if (now - last_nmea_time >= nmea_interval_ms) {
            last_nmea_time = now;

            nmea_gen_generate(&nmea, gx, gy, gz, gvx, gvy,
                              GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
                              now / 1000UL);

            /* DMA 非阻塞发送 NMEA 到飞控 GPS 口 */
            const char *ggpa = nmea_gen_ggpa(&nmea);
            const char *rmc  = nmea_gen_rmc(&nmea);
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)ggpa, (uint16_t)strlen(ggpa));
            {
                uint32_t dma_timeout = HAL_GetTick() + 50;
                while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {
                    if (HAL_GetTick() > dma_timeout) break;  /* DMA hung, skip */
                }
            }
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)rmc, (uint16_t)strlen(rmc));

            nmea_count++;
            if (nmea_count == 1) {
                printf("\r\n  NMEA 首条已发送:\r\n");
                printf("    %s", ggpa);
                printf("    %s\r\n\r\n", rmc);
            }
            if (nmea_count % ((uint32_t)NMEA_RATE_HZ * 5) == 0) {
                printf("  NMEA %lu 组 → USART3 %s\r\n",
                       (unsigned long)nmea_count,
                       has_pos ? "[UWB]" : "[原点-无UWB]");
            }
        }

        HAL_Delay(LOOP_INTERVAL_MS);
    }
}

/* ========================================================================
 * 外设初始化 stub — CubeMX 生成独立 .c 文件后可删除
 * ======================================================================== */

static void SystemClock_Config(void) {
    /* CubeMX will generate this — 72MHz HSE + PLL */
}

static void MX_GPIO_Init(void) {
    /* CubeMX will generate this — PC13 LED output */
}

static void MX_DMA_Init(void) {
    /* CubeMX will generate this — DMA1 Channel6 for USART2_RX */
}

static void MX_USART1_UART_Init(void) {
    /* CubeMX will generate this — PA9 TX, PA10 RX, 115200-8-N-1 */
}

static void MX_USART2_UART_Init(void) {
    /* CubeMX will generate this — PA2 TX, PA3 RX, 19200-8-N-1 */
}

static void MX_USART3_UART_Init(void) {
    /* CubeMX will generate this — PB10 TX, PB11 RX, 57600-8-N-1 */
}
