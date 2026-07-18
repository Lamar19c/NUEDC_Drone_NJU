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
#include "mavlink.h"
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

/* ---- 固定高度：UWB 只解水平，z 交给此常值(或下视测距仪) ---- */
#define FIXED_HEIGHT_M  1.6f

/* ---- position cache ---- */
static float gx = 0.0f, gy = 0.0f, gz = FIXED_HEIGHT_M;
static float gvx = 0.0f, gvy = 0.0f;
static int   has_pos = 0;

/* ---- MAVLink counters ---- */
static uint32_t mav_count = 0;
static uint32_t last_mav_time = 0;
static uint32_t last_hb_time = 0;
static uint32_t loop_count = 0;

/* ---- non-blocking loop ---- */
static uint32_t last_loop_time = 0;

/* ---- runtime stats ---- */
static uint32_t stat_parse_ok = 0, stat_parse_err = 0;
static uint32_t stat_solve_ok = 0, stat_solve_fail = 0;
static uint32_t stat_mav_frames = 0;
static uint32_t last_stat_time = 0;

/* ---- USART2 receive complete callback ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart2_rx_count++;
        uwb_parser_feed(&parser, (char)rx2_char, HAL_GetTick());
        HAL_UART_Receive_IT(&huart2, &rx2_char, 1);
    }
}

/* ---- USART3 TX complete callback (MAVLink: blocking transmit, no DMA chain needed) ---- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    (void)huart;  /* no-op: MAVLink uses blocking transmit */
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
        /* MAVLink: abort any stuck transmit */
        HAL_UART_AbortTransmit(&huart3);
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
  mavlink_init(&huart3);

  HAL_UART_Receive_IT(&huart2, &rx2_char, 1);
  last_stat_time = HAL_GetTick();

  printf("\r\n========================================\r\n");
  printf("  UWB Indoor 3D Localization - STM32F103C8T6\r\n");
  printf("  (CV-EKF + MAVLink VISION_POSITION_ESTIMATE)\r\n");
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
  printf("Output: MAVLink VPE + HEARTBEAT @ USART3\r\n");
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

            /* 健康度：逐帧观察 fix / hdop / 在线锚点数 */
            int dbg_fix = 0, dbg_sats = 0; float dbg_hdop = 0.0f;
            uwb_ekf_health(&ekf, now, &dbg_fix, &dbg_sats, &dbg_hdop);
            int dbg_anc = uwb_ekf_online_anchors(&ekf, now);
            int hdi = (int)(dbg_hdop * 100.0f + 0.5f);

            int xi  = (int)(x  * 100.0f + (x  >= 0 ? 0.5f : -0.5f));
            int yi  = (int)(y  * 100.0f + (y  >= 0 ? 0.5f : -0.5f));
            int zi  = (int)(z  * 100.0f + (z  >= 0 ? 0.5f : -0.5f));
            int vxi = (int)(vx * 100.0f + (vx >= 0 ? 0.5f : -0.5f));
            int vyi = (int)(vy * 100.0f + (vy >= 0 ? 0.5f : -0.5f));
            int vzi = (int)(vz * 100.0f + (vz >= 0 ? 0.5f : -0.5f));

            printf("[%lu] x=%d.%02d y=%d.%02d z=%d.%02d  "
                   "vx=%d.%02d vy=%d.%02d vz=%d.%02d  "
                   "fix=%d anc=%d hdop=%d.%02d\r\n",
                   (unsigned long)(HAL_GetTick() / 1000UL),
                   xi / 100,  (xi  >= 0 ? xi  : -xi ) % 100,
                   yi / 100,  (yi  >= 0 ? yi  : -yi ) % 100,
                   zi / 100,  (zi  >= 0 ? zi  : -zi ) % 100,
                   vxi / 100, (vxi >= 0 ? vxi : -vxi) % 100,
                   vyi / 100, (vyi >= 0 ? vyi : -vyi) % 100,
                   vzi / 100, (vzi >= 0 ? vzi : -vzi) % 100,
                   dbg_fix, dbg_anc, hdi / 100, (hdi >= 0 ? hdi : -hdi) % 100);
        } else {
            uwb_ekf_init(&ekf, ANCHOR_POSITIONS, ANCHOR_COUNT);
            uwb_ekf_set_fixed_z(&ekf, FIXED_HEIGHT_M);   /* 复位重收敛 */
            has_pos = 0;
            stat_solve_fail++;
            printf("  [DIAG] EKF diverged -> reset #%lu\r\n",
                   (unsigned long)stat_solve_fail);
        }
    }

    /* ---- MAVLink output (5Hz VPE + 1Hz HEARTBEAT) ---- */
    if (now - last_mav_time >= (uint32_t)(1000.0f / NMEA_RATE_HZ)) {
        last_mav_time = now;

        if (has_pos) {
            /* UWB → NED:  X→East, Y→North → VPE.x=North, VPE.y=East, VPE.z=-Up */
            float heading = atan2f(gvx, gvy) * 57.295779513f;
            if (heading < 0.0f) heading += 360.0f;

            /* usec timestamp from HAL_GetTick (ms → us) */
            uint64_t now_us = (uint64_t)now * 1000UL;

            mavlink_send_vision_position(gy, gx, -gz,
                                         0.0f, 0.0f, 0.0f,
                                         heading, now_us);
            mav_count++;
            stat_mav_frames++;

            if (mav_count == 1) {
                printf("\r\n  MAVLink VPE: x=%.2f y=%.2f z=%.2f hdg=%.1f\r\n\r\n",
                       gy, gx, -gz, heading);
            }
        }

        /* HEARTBEAT @ 1Hz */
        if (now - last_hb_time >= 1000UL) {
            last_hb_time = now;
            mavlink_send_heartbeat(now);
        }
    }

    /* ---- runtime stats (every 5s) ---- */
    if (now - last_stat_time >= 5000UL) {
        last_stat_time = now;
        printf("\r\n  === STATS (5s) ===\r\n");
        printf("  parse: ok=%lu err=%lu | solve: ok=%lu fail=%lu\r\n",
               stat_parse_ok, stat_parse_err, stat_solve_ok, stat_solve_fail);
        printf("  mav_sent: %lu | has_pos: %d | valid_cnt: %d\r\n",
               stat_mav_frames, has_pos,
               uwb_parser_valid_count(&parser, HAL_GetTick()));
        printf("  uart2_rx: %lu bytes | parser: ok=%lu bad=%lu\r\n",
               uart2_rx_count,
               (unsigned long)g_parser_line_ok, (unsigned long)g_parser_line_bad);
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
