/**
 * spi_dw3000.c — SPI1 驱动 DW3000
 *   PA4=CS(软件)  PA5=SCK  PA6=MISO  PA7=MOSI
 *   CPOL=0 CPHA=1  9MHz (72/8)
 */
#include "hardware/board.h"

SPI_HandleTypeDef hspi_dwm;

void dw3000_spi_cs_low(void) {
    HAL_GPIO_WritePin(DWM_CS_PORT, DWM_CS_PIN, GPIO_PIN_RESET);
}
void dw3000_spi_cs_high(void) {
    HAL_GPIO_WritePin(DWM_CS_PORT, DWM_CS_PIN, GPIO_PIN_SET);
}

void dw3000_spi_write(const uint8_t *tx, uint16_t len) {
    HAL_SPI_Transmit(&hspi_dwm, (uint8_t *)tx, len, 10);
}
void dw3000_spi_read(uint8_t *rx, uint16_t len) {
    HAL_SPI_Receive(&hspi_dwm, rx, len, 10);
}

/* ── decadriver 要求的 writetospi / readfromspi ── */
int writetospi(uint16_t hdr_len, const uint8_t *hdr_buf,
               uint32_t body_len, const uint8_t *body_buf) {
    dw3000_spi_cs_low();
    if (hdr_len)  dw3000_spi_write(hdr_buf, hdr_len);
    if (body_len) dw3000_spi_write(body_buf, (uint16_t)body_len);
    dw3000_spi_cs_high();
    return 0;
}
int readfromspi(uint16_t hdr_len, const uint8_t *hdr_buf,
                uint32_t body_len, uint8_t *body_buf) {
    dw3000_spi_cs_low();
    if (hdr_len)  dw3000_spi_write(hdr_buf, hdr_len);
    if (body_len) dw3000_spi_read(body_buf, (uint16_t)body_len);
    dw3000_spi_cs_high();
    return 0;
}

void spi_dw3000_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* CS — 推挽输出, 初始高 */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin   = DWM_CS_PIN;
    HAL_GPIO_Init(DWM_CS_PORT, &g);
    dw3000_spi_cs_high();

    /* SCK + MOSI — 复用推挽 */
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pin   = DWM_SCK_PIN | DWM_MOSI_PIN;
    HAL_GPIO_Init(DWM_SCK_PORT, &g);

    /* MISO — 浮空输入 */
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_NOPULL;
    g.Pin   = DWM_MISO_PIN;
    HAL_GPIO_Init(DWM_MISO_PORT, &g);

    hspi_dwm.Instance               = SPI1;
    hspi_dwm.Init.Mode              = SPI_MODE_MASTER;
    hspi_dwm.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi_dwm.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi_dwm.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi_dwm.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi_dwm.Init.NSS               = SPI_NSS_SOFT;
    hspi_dwm.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi_dwm.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi_dwm);
}
