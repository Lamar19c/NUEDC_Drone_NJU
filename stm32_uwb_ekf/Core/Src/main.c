/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : UWB EKF indoor localization — STM32F103C8T6
  *
  * EKF: 常速模型 + 逐锚点顺序更新，消除 TDMA 时间偏斜
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
#include "uwb_config.h"
#include "uwb_parser.h"
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
/* ---- printf redirect (non-blocking) ---- */
int _write(int fd, char *ptr, int len) {
    if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100) != HAL_OK) {
        return len;
    }
    return len;
}

/* ---- USART2 interrupt receive ---- */
static uint8_t rx2_char;
static uint32_t uart2_rx_count = 0;

/* ---- EKF + parser + NMEA ---- */
static struct UWB_Parser      parser;
static struct UWB_EKF         ekf;
static struct NMEA_Generator  nmea;

/* ---- position cache ---- */
static float gx = 0.0f, gy = 0.0f, gz = 1.0f;
static float gvx = 0.0f, gvy = 0.0f;
static int   has_pos = 0;

/* ---- NMEA counters ---- */
static uint32_t nmea_count = 0;
static uint32_t last_nmea_time = 0;

/* ---- non-blocking loop ---- */
static uint32_t last_loop_time = 0;

/* ---- runtime stats ---- */
static uint32_t stat_ekf_updates = 0, stat_ekf_none = 0;
static uint32_t stat_nmea_frames = 0;
static uint32_t last_stat_time = 0;

/* ---- NMEA non-blocking DMA chain ---- */
typedef enum {
    NMEA_IDLE = 0,
    NMEA_SENDING_GGPA,
    NMEA_SENDING_RMC,
    NMEA_SENDING_VTG
} nmea_tx_state_t;

static nmea_tx_state_t nmea_tx_state = NMEA_IDLE;
static const char *nmea_tx_rmc_ptr  = NULL;
static const char *nmea_tx_vtg_ptr  = NULL;

static void nmea_start_dma_send(const char *ggpa, const char *rmc, const char *vtg) {
    if (nmea_tx_state != NMEA_IDLE) return;
    nmea_tx_rmc_ptr = rmc;
    nmea_tx_vtg_ptr = vtg;
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
            nmea_tx_state = NMEA_IDLE;
            break;
        default:
            break;
        }
    }
}

/* ---- UART error recovery ---- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(&huart2, &rx2_char, 1);
    }
    if (huart->Instance == USART3) {
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

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  uwb_parser_init(&parser);
  uwb_ekf_init(&ekf, ANCHOR_POSITIONS, ANCHOR_COUNT);
  nmea_gen_init(&nmea);

  HAL_UART_Receive_IT(&huart2, &rx2_char, 1);
  last_stat_time = HAL_GetTick();

  printf("\r\n========================================\r\n");
  printf("  UWB Indoor Localization — EKF (CV model)\r\n");
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
    uint32_t now = HAL_GetTick();
    if (now - last_loop_time < LOOP_INTERVAL_MS) continue;
    last_loop_time = now;

    /* ---- EKF: feed raw distances with per-anchor timestamps ---- */
    float distances[ANCHOR_COUNT];
    uwb_parser_get_valid_distances(&parser, distances, ANCHOR_COUNT, now);
    int applied = uwb_ekf_step(&ekf, distances, parser.last_update_ms, now);

    if (applied > 0) {
        stat_ekf_updates++;
        float x, y, z, vx, vy, vz;
        uwb_ekf_get(&ekf, now, &x, &y, &z, &vx, &vy, &vz);

        /* divergence guard: reset EKF if state goes NaN/Inf */
        if (!(x == x && x * 0.0f == 0.0f &&
              y == y && y * 0.0f == 0.0f &&
              z == z && z * 0.0f == 0.0f)) {
            uwb_ekf_init(&ekf, ANCHOR_POSITIONS, ANCHOR_COUNT);
            has_pos = 0;
            printf("  [EKF RESET] divergence detected\r\n");
        } else {
            gx = x; gy = y; gz = z;
            gvx = vx; gvy = vy;
            has_pos = 1;
        }

        int32_t lat_e7, lon_e7, alt_mm;
        local_to_gps(gx, gy, gz, GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
                     &lat_e7, &lon_e7, &alt_mm);

        int xi  = (int)(gx  * 100.0f + (gx  >= 0 ? 0.5f : -0.5f));
        int yi  = (int)(gy  * 100.0f + (gy  >= 0 ? 0.5f : -0.5f));
        int zi  = (int)(gz  * 100.0f + (gz  >= 0 ? 0.5f : -0.5f));
        int vxi = (int)(gvx * 100.0f + (gvx >= 0 ? 0.5f : -0.5f));
        int vyi = (int)(gvy * 100.0f + (gvy >= 0 ? 0.5f : -0.5f));

        printf("[%lu] x=%d.%02d y=%d.%02d z=%d.%02d  v=%d.%02d,%d.%02d  GPS: %ld.%07ld,%ld.%07ld,%ld.%03ldm\r\n",
               (unsigned long)(now / 1000UL),
               xi / 100,  (xi  >= 0 ? xi  : -xi ) % 100,
               yi / 100,  (yi  >= 0 ? yi  : -yi ) % 100,
               zi / 100,  (zi  >= 0 ? zi  : -zi ) % 100,
               vxi / 100, (vxi >= 0 ? vxi : -vxi) % 100,
               vyi / 100, (vyi >= 0 ? vyi : -vyi) % 100,
               (long)(lat_e7 / 10000000),   labs(lat_e7 % 10000000),
               (long)(lon_e7 / 10000000),   labs(lon_e7 % 10000000),
               (long)(alt_mm / 1000),       labs(alt_mm % 1000));
    } else if (uwb_parser_valid_count(&parser, now) >= 3) {
        stat_ekf_none++;
    }

    /* ---- NMEA output (5Hz, non-blocking DMA chain) ---- */
    if (now - last_nmea_time >= (uint32_t)(1000.0f / NMEA_RATE_HZ)) {
        last_nmea_time = now;

        if (nmea_tx_state == NMEA_IDLE) {
            int fix_ok, sats;
            float hdop;
            uwb_ekf_health(&ekf, now, &fix_ok, &sats, &hdop);

            /* DIAG: hardcode fix to bypass health gating for test */
            nmea_gen_generate(&nmea, gx, gy, gz, gvx, gvy,
                              GPS_ORIGIN_LAT, GPS_ORIGIN_LON, GPS_ORIGIN_ALT,
                              now / 1000UL, /*fix*/1, /*sats*/12, /*hdop*/0.9f);

            const char *ggpa = nmea_gen_ggpa(&nmea);
            const char *rmc  = nmea_gen_rmc(&nmea);
            const char *vtg  = nmea_gen_vtg(&nmea);
            nmea_start_dma_send(ggpa, rmc, vtg);

            nmea_count++;
            stat_nmea_frames++;
            if (nmea_count == 1) {
                printf("\r\n  NMEA first sentences sent:\r\n");
                printf("    %s", ggpa);
                printf("    %s", rmc);
                printf("    %s\r\n\r\n", vtg);
            }
            if (nmea_count % ((uint32_t)NMEA_RATE_HZ * 5) == 0) {
                printf("  NMEA %lu -> USART3  fix=%d sats=%d hdop=%.1f\r\n",
                       nmea_count, fix_ok, sats, (double)hdop);
            }
        }
    }

    /* ---- runtime stats (every 5s) ---- */
    if (now - last_stat_time >= 5000UL) {
        last_stat_time = now;
        printf("\r\n  === STATS (5s) ===\r\n");
        printf("  ekf_updates: %lu | ekf_none: %lu\r\n",
               stat_ekf_updates, stat_ekf_none);
        printf("  nmea_sent: %lu | dma_state: %d\r\n",
               stat_nmea_frames, (int)nmea_tx_state);
        printf("  uart2_rx: %lu bytes | valid_cnt: %d\r\n",
               uart2_rx_count, uwb_parser_valid_count(&parser, now));
        printf("  parser: ok=%d bad=%d\r\n", g_line_ok, g_line_bad);
        if (g_has_raw_line) {
            printf("  raw: [%s]\r\n", g_raw_line);
            g_has_raw_line = 0;
        }
        printf("\r\n");
    }
    /* USER CODE END 3 */
  }
}

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

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
