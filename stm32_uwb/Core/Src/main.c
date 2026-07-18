/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "uwb_config.h"
#include "uwb_solver.h"   /* 解析器 UWB_Parser 仍在此 */
#include "uwb_ekf.h"
#include "uwb_nmea.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- printf redirect (non-blocking: debug output is optional) ---- */
int _write(int fd, char *ptr, int len) {
    /* 100ms timeout: enough for ~1KB at 115200, won't hang forever */
    if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100) != HAL_OK) {
        return len;  /* discard silently if timed out */
    }
    return len;
}

/* ---- USART2 interrupt receive ---- */
static uint8_t rx2_char;
static uint32_t uart2_rx_count = 0;     /* diagnostic: total bytes received */

/* ---- global objects ---- */
static struct UWB_Parser      parser;
static struct UWB_EKF         ekf;
static struct NMEA_Generator  nmea;

/* ---- 固定高度：UWB 只解水平，z 交给此常值(或下视测距仪) ---- */
#define FIXED_HEIGHT_M  1.6f

/* ---- position cache ---- */
static float gx = 0.0f, gy = 0.0f, gz = FIXED_HEIGHT_M;
static float gvx = 0.0f, gvy = 0.0f;
static int   has_pos = 0;

/* ---- NMEA counters ---- */
static uint32_t nmea_count = 0;
static uint32_t last_nmea_time = 0;
static uint32_t loop_count = 0;

/* ---- non-blocking loop ---- */
static uint32_t last_loop_time = 0;

/* ---- runtime stats ---- */
static uint32_t stat_parse_ok = 0, stat_parse_err = 0;
static uint32_t stat_solve_ok = 0, stat_solve_fail = 0;
static uint32_t stat_nmea_frames = 0;
static uint32_t last_stat_time = 0;

/* ---- NMEA non-blocking DMA chain ---- */
typedef enum {
    NMEA_IDLE = 0,
    NMEA_SENDING_GGPA,
    NMEA_SENDING_RMC,
    NMEA_SENDING_VTG,
    NMEA_SENDING_GSA
} nmea_tx_state_t;

static nmea_tx_state_t nmea_tx_state = NMEA_IDLE;
static const char *nmea_tx_rmc_ptr  = NULL;  /* saved for chaining */
static const char *nmea_tx_vtg_ptr  = NULL;
static const char *nmea_tx_gsa_ptr  = NULL;

/** Kick off NMEA DMA chain: GGA → RMC → VTG → GSA via TX complete callback.
 *  GSA 携带定位类型(3=3D)，飞控靠它确认三维定位，缺了会卡在"未定位/二维"。 */
static void nmea_start_dma_send(const char *ggpa, const char *rmc,
                                const char *vtg, const char *gsa) {
    if (nmea_tx_state != NMEA_IDLE) return;  /* still sending previous batch */
    nmea_tx_rmc_ptr = rmc;
    nmea_tx_vtg_ptr = vtg;
    nmea_tx_gsa_ptr = gsa;
    nmea_tx_state   = NMEA_SENDING_GGPA;
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)ggpa, (uint16_t)strlen(ggpa));
}

/* ---- USART2 receive complete callback ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart2_rx_count++;
        uwb_parser_feed(&parser, (char)rx2_char, HAL_GetTick());
        HAL_UART_Receive_IT(&huart2, &rx2_char, 1);
    }
}

/* ---- USART3 TX complete callback (DMA chain: GGA → RMC → VTG) ---- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        switch (nmea_tx_state) {
        case NMEA_SENDING_GGPA:
            nmea_tx_state = NMEA_SENDING_RMC;
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)nmea_tx_rmc_ptr,
                                  (uint16_t)strlen(nmea_tx_rmc_ptr));
            break;
        case NMEA_SENDING_RMC:
            nmea_tx_state = NMEA_SENDING_VTG;
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)nmea_tx_vtg_ptr,
                                  (uint16_t)strlen(nmea_tx_vtg_ptr));
            break;
        case NMEA_SENDING_VTG:
            nmea_tx_state = NMEA_SENDING_GSA;
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)nmea_tx_gsa_ptr,
                                  (uint16_t)strlen(nmea_tx_gsa_ptr));
            break;
        case NMEA_SENDING_GSA:
            nmea_tx_state = NMEA_IDLE;
            break;
        default:
            break;
        }
    }
}

/* ---- UART error recovery: prevent permanent receive stall ---- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        /* clear error flags and restart receive */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(&huart2, &rx2_char, 1);
    }
    if (huart->Instance == USART3) {
        /* DMA TX error: abort and reset state machine
           otherwise nmea_tx_state never returns to NMEA_IDLE */
        HAL_UART_AbortTransmit(&huart3);
        nmea_tx_state = NMEA_IDLE;
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  uwb_parser_init(&parser);
  uwb_ekf_init(&ekf, ANCHOR_POSITIONS, ANCHOR_COUNT);
  uwb_ekf_set_fixed_z(&ekf, FIXED_HEIGHT_M);   /* 固定高度模式 */
  nmea_gen_init(&nmea);

  HAL_UART_Receive_IT(&huart2, &rx2_char, 1);
  last_stat_time = HAL_GetTick();

  printf("\r\n========================================\r\n");
  printf("  UWB Indoor 3D Localization - STM32F103C8T6\r\n");
  printf("  (CV-EKF, sequential range update, fixed height)\r\n");
  printf("========================================\r\n");
  printf("Anchors: %d\r\n", ANCHOR_COUNT);
  for (int i = 0; i < ANCHOR_COUNT; i++) {
    int ax = (int)(ANCHOR_POSITIONS[i][0] * 100.0f + 0.5f);
    int ay = (int)(ANCHOR_POSITIONS[i][1] * 100.0f + 0.5f);
    int az = (int)(ANCHOR_POSITIONS[i][2] * 100.0f + 0.5f);
    printf("  S%d: (%d.%02d, %d.%02d, %d.%02d)\r\n",
           i + 1,
           ax / 100, (ax >= 0 ? ax : -ax) % 100,
           ay / 100, (ay >= 0 ? ay : -ay) % 100,
           az / 100, (az >= 0 ? az : -az) % 100);
  }
  if (ANCHOR_COUNT == 3) {
    int dz = (int)(DEFAULT_HEIGHT * 100.0f + 0.5f);
    printf("3-anchor mode: Z = %d.%02dm\r\n", dz / 100, (dz >= 0 ? dz : -dz) % 100);
  }
  printf("GPS origin: lat=%ld lon=%ld alt=%ld\r\n",
         (long)GPS_ORIGIN_LAT, (long)GPS_ORIGIN_LON, (long)GPS_ORIGIN_ALT);
  printf("Waiting for UWB data...\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- tick-based non-blocking scheduling ---- */
    uint32_t now = HAL_GetTick();
    if (now - last_loop_time < LOOP_INTERVAL_MS) continue;
    last_loop_time = now;

    /* ---- UWB solve (CV-EKF, 逐锚点时间戳顺序更新) ---- */
    if (uwb_parser_valid_count(&parser, now) >= 3) {
        float distances[ANCHOR_COUNT];
        int valid = uwb_parser_get_valid_distances(&parser, distances, ANCHOR_COUNT, now);
        if (valid == 0) stat_parse_err++; else stat_parse_ok++;

        /* 原始距离直接喂 EKF：内部按各锚点时间戳升序做 predict+update，
           从根本上消除顺序测距的时间偏斜；外点由 innovation 门限剔除，
           不再做距离中值/EMA(那是滞后与"越快框越大"的来源)。 */
        int applied = uwb_ekf_step(&ekf, distances, parser.last_update_ms, now);

        float x, y, z, vx, vy, vz;
        uwb_ekf_get(&ekf, now, &x, &y, &z, &vx, &vy, &vz);

        /* 发散防护：坏几何/soft-float 下万一出 NaN/Inf，复位重收敛，
           绝不让乱码污染 NMEA(否则飞控会收到 nan 经纬度)。 */
        if (isfinite(x) && isfinite(y) && isfinite(z)) {
            gx = x; gy = y; gz = z; gvx = vx; gvy = vy;
            has_pos = 1;
            if (applied > 0) { loop_count++; stat_solve_ok++; }

            int32_t lat_e7, lon_e7, alt_mm;
            local_to_gps(x, y, z, GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
                         &lat_e7, &lon_e7, &alt_mm);

            /* 健康度：逐帧观察 fix / hdop / 在线锚点数 */
            int dbg_fix = 0, dbg_sats = 0; float dbg_hdop = 0.0f;
            uwb_ekf_health(&ekf, now, &dbg_fix, &dbg_sats, &dbg_hdop);
            int dbg_anc = uwb_ekf_online_anchors(&ekf, now);
            int hdi = (int)(dbg_hdop * 100.0f + 0.5f);   /* 无 float printf，整数化 */

            int xi  = (int)(x  * 100.0f + (x  >= 0 ? 0.5f : -0.5f));
            int yi  = (int)(y  * 100.0f + (y  >= 0 ? 0.5f : -0.5f));
            int zi  = (int)(z  * 100.0f + (z  >= 0 ? 0.5f : -0.5f));
            int vxi = (int)(vx * 100.0f + (vx >= 0 ? 0.5f : -0.5f));
            int vyi = (int)(vy * 100.0f + (vy >= 0 ? 0.5f : -0.5f));
            int vzi = (int)(vz * 100.0f + (vz >= 0 ? 0.5f : -0.5f));

            printf("[%lu] x=%d.%02d y=%d.%02d z=%d.%02d  "
                   "vx=%d.%02d vy=%d.%02d vz=%d.%02d  "
                   "fix=%d anc=%d hdop=%d.%02d  "
                   "GPS: %ld.%07ld,%ld.%07ld,%ld.%03ldm\r\n",
                   (unsigned long)(HAL_GetTick() / 1000UL),
                   xi / 100,  (xi  >= 0 ? xi  : -xi ) % 100,
                   yi / 100,  (yi  >= 0 ? yi  : -yi ) % 100,
                   zi / 100,  (zi  >= 0 ? zi  : -zi ) % 100,
                   vxi / 100, (vxi >= 0 ? vxi : -vxi) % 100,
                   vyi / 100, (vyi >= 0 ? vyi : -vyi) % 100,
                   vzi / 100, (vzi >= 0 ? vzi : -vzi) % 100,
                   dbg_fix, dbg_anc, hdi / 100, (hdi >= 0 ? hdi : -hdi) % 100,
                   (long)(lat_e7 / 10000000),   labs(lat_e7 % 10000000),
                   (long)(lon_e7 / 10000000),   labs(lon_e7 % 10000000),
                   (long)(alt_mm / 1000),       labs(alt_mm % 1000));

            /* ===== 全链路诊断行(约每3帧一次，采圈测试用) =====
             * DBG t | d= 原始距离cm | age= 各锚点数据年龄ms(看时间偏斜)
             *      | ekf= EKF原始xy(cm,未量化) | v= 速度cm/s | e7= 量化前经纬度
             * 采完用脚本对比: EKF原始xy 是否随速度变大小(EKF层)，
             * 以及 e7 按4位量化回本地后 与 EKF原始xy 的差(量化层)。 */
            static uint32_t dbg_cnt = 0;
            if (++dbg_cnt % 3 == 0) {
                printf("DBG t=%lu d=%d,%d,%d,%d age=%lu,%lu,%lu,%lu "
                       "ekf=%d,%d v=%d,%d e7=%ld,%ld\r\n",
                       (unsigned long)now,
                       (int)(distances[0] * 100.0f), (int)(distances[1] * 100.0f),
                       (int)(distances[2] * 100.0f), (int)(distances[3] * 100.0f),
                       (unsigned long)(now - parser.last_update_ms[0]),
                       (unsigned long)(now - parser.last_update_ms[1]),
                       (unsigned long)(now - parser.last_update_ms[2]),
                       (unsigned long)(now - parser.last_update_ms[3]),
                       xi, yi, vxi, vyi,
                       (long)lat_e7, (long)lon_e7);
            }
        } else {
            uwb_ekf_init(&ekf, ANCHOR_POSITIONS, ANCHOR_COUNT);
            uwb_ekf_set_fixed_z(&ekf, FIXED_HEIGHT_M);   /* 复位重收敛 */
            has_pos = 0;
            stat_solve_fail++;
            printf("  [DIAG] EKF diverged -> reset #%lu\r\n",
                   (unsigned long)stat_solve_fail);
        }
    }

    /* ---- NMEA output (5Hz, non-blocking DMA chain) ---- */
    if (now - last_nmea_time >= (uint32_t)(1000.0f / NMEA_RATE_HZ)) {
        last_nmea_time = now;

        /* DMA guard: don't overwrite buffers while DMA still active */
        if (nmea_tx_state == NMEA_IDLE) {
            /* 健康度由 EKF 协方差/锚点新鲜度实时给出，失锁时如实反映，
               避免飞控把冻结/发散的位置当成健康 GPS。 */
            int fix_ok = 0, sats = 0; float hdop = 99.0f;
            uwb_ekf_health(&ekf, now, &fix_ok, &sats, &hdop);
            if (!has_pos) { fix_ok = 0; sats = 0; }   /* 首个有效解前不报锁 */

            nmea_gen_generate(&nmea, gx, gy, gz, gvx, gvy,
                              GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
                              now / 1000UL, fix_ok, sats, hdop);

            const char *ggpa = nmea_gen_ggpa(&nmea);
            const char *rmc  = nmea_gen_rmc(&nmea);
            const char *vtg  = nmea_gen_vtg(&nmea);
            const char *gsa  = nmea_gen_gsa(&nmea);

            /* non-blocking DMA chain: GGA→RMC→VTG→GSA via TX complete ISR */
            nmea_start_dma_send(ggpa, rmc, vtg, gsa);

            nmea_count++;
            stat_nmea_frames++;
            if (nmea_count == 1) {
                printf("\r\n  NMEA first sentences sent:\r\n");
                printf("    %s", ggpa);
                printf("    %s", rmc);
                printf("    %s", vtg);
                printf("    %s\r\n\r\n", gsa);
            }
            if (nmea_count % ((uint32_t)NMEA_RATE_HZ * 5) == 0) {
                printf("  TX GGA: %s", ggpa);   /* 看实际发出的定位质量位 */
            }
        }
    }

    /* ---- runtime stats (every 5s) ---- */
    if (now - last_stat_time >= 5000UL) {
        last_stat_time = now;
        printf("\r\n  === STATS (5s) ===\r\n");
        printf("  parse: ok=%lu err=%lu | solve: ok=%lu fail=%lu\r\n",
               stat_parse_ok, stat_parse_err, stat_solve_ok, stat_solve_fail);
        printf("  nmea_sent: %lu | dma_state: %d\r\n",
               stat_nmea_frames, (int)nmea_tx_state);
        printf("  uart2_rx: %lu bytes | parser: ok=%lu bad=%lu\r\n",
               uart2_rx_count,
               (unsigned long)g_parser_line_ok, (unsigned long)g_parser_line_bad);
        printf("  has_pos: %d | valid_cnt: %d\r\n",
               has_pos, uwb_parser_valid_count(&parser, HAL_GetTick()));
        if (g_uwb_has_raw_line) {
            printf("  last_raw: [%s]\r\n", g_uwb_last_raw_line);
            g_uwb_has_raw_line = 0;
        }
        printf("\r\n");
    }
    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
