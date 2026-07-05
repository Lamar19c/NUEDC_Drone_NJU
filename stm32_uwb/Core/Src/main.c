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
#include "uwb_config.h"
#include "uwb_solver.h"
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
/* ---- printf redirect ---- */
int _write(int fd, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ---- USART2 interrupt receive ---- */
static uint8_t rx2_char;

/* ---- global objects ---- */
static struct UWB_Parser      parser;
static struct DistanceFilter  dist_filt;
static struct UWBSolver       solver;
static struct NMEA_Generator  nmea;

/* ---- position cache ---- */
static float gx = 0.0f, gy = 0.0f, gz = 1.0f;
static float gvx = 0.0f, gvy = 0.0f;
static int   has_pos = 0;

/* ---- NMEA counters ---- */
static uint32_t nmea_count = 0;
static uint32_t last_nmea_time = 0;
static uint32_t loop_count = 0;

/* ---- USART2 receive complete callback ---- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uwb_parser_feed(&parser, (char)rx2_char);
        HAL_UART_Receive_IT(&huart2, &rx2_char, 1);
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
  dist_filter_init(&dist_filt);
  uwb_solver_init(&solver, ANCHOR_POSITIONS, ANCHOR_COUNT);
  nmea_gen_init(&nmea);

  HAL_UART_Receive_IT(&huart2, &rx2_char, 1);

  printf("\r\n========================================\r\n");
  printf("  UWB Indoor 3D Localization - STM32F103C8T6\r\n");
  printf("  (dual median filter)\r\n");
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
    if (uwb_parser_valid_count(&parser) >= 3) {
        float distances[ANCHOR_COUNT];
        uwb_parser_get_distances(&parser, distances, ANCHOR_COUNT);
        uwb_parser_clear_distances(&parser);

        dist_filter_apply(&dist_filt, distances, ANCHOR_COUNT);

        float x, y, z, vx, vy, vz;
        if (uwb_solver_solve(&solver, distances, HAL_GetTick(),
                             &x, &y, &z, &vx, &vy, &vz)) {
            gx = x; gy = y; gz = z; gvx = vx; gvy = vy;
            has_pos = 1;
            loop_count++;

            int32_t lat_e7, lon_e7, alt_mm;
            local_to_gps(x, y, z, GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
                         &lat_e7, &lon_e7, &alt_mm);

            int xi  = (int)(x  * 100.0f + (x  >= 0 ? 0.5f : -0.5f));
            int yi  = (int)(y  * 100.0f + (y  >= 0 ? 0.5f : -0.5f));
            int zi  = (int)(z  * 100.0f + (z  >= 0 ? 0.5f : -0.5f));
            int vxi = (int)(vx * 100.0f + (vx >= 0 ? 0.5f : -0.5f));
            int vyi = (int)(vy * 100.0f + (vy >= 0 ? 0.5f : -0.5f));
            int vzi = (int)(vz * 100.0f + (vz >= 0 ? 0.5f : -0.5f));

            printf("[%lu] x=%d.%02d y=%d.%02d z=%d.%02d  "
                   "vx=%d.%02d vy=%d.%02d vz=%d.%02d  "
                   "GPS: %ld.%07ld,%ld.%07ld,%ld.%03ldm\r\n",
                   (unsigned long)(HAL_GetTick() / 1000UL),
                   xi / 100,  (xi  >= 0 ? xi  : -xi ) % 100,
                   yi / 100,  (yi  >= 0 ? yi  : -yi ) % 100,
                   zi / 100,  (zi  >= 0 ? zi  : -zi ) % 100,
                   vxi / 100, (vxi >= 0 ? vxi : -vxi) % 100,
                   vyi / 100, (vyi >= 0 ? vyi : -vyi) % 100,
                   vzi / 100, (vzi >= 0 ? vzi : -vzi) % 100,
                   (long)(lat_e7 / 10000000),   labs(lat_e7 % 10000000),
                   (long)(lon_e7 / 10000000),   labs(lon_e7 % 10000000),
                   (long)(alt_mm / 1000),       labs(alt_mm % 1000));
        }
    }

    /* ---- NMEA output (5Hz) ---- */
    uint32_t now = HAL_GetTick();
    if (now - last_nmea_time >= (uint32_t)(1000.0f / NMEA_RATE_HZ)) {
        last_nmea_time = now;

        nmea_gen_generate(&nmea, gx, gy, gz, gvx, gvy,
                          GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
                          now / 1000UL);

        const char *ggpa = nmea_gen_ggpa(&nmea);
        const char *rmc  = nmea_gen_rmc(&nmea);
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)ggpa, (uint16_t)strlen(ggpa));
        {
            uint32_t dma_timeout = HAL_GetTick() + 50;
            while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {
                if (HAL_GetTick() > dma_timeout) break;
            }
        }
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)rmc, (uint16_t)strlen(rmc));

        nmea_count++;
        if (nmea_count == 1) {
            printf("\r\n  NMEA first sentence sent:\r\n");
            printf("    %s", ggpa);
            printf("    %s\r\n\r\n", rmc);
        }
        if (nmea_count % ((uint32_t)NMEA_RATE_HZ * 5) == 0) {
            printf("  NMEA %lu -> USART3 %s\r\n",
                   nmea_count, has_pos ? "[UWB]" : "[origin]");
        }
    }

    HAL_Delay(LOOP_INTERVAL_MS);
    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */
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
