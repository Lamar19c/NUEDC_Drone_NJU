/**
 * i2c_eep.c — I2C1 EEPROM AT24C64 (PB6=SCL, PB7=SDA) 100kHz
 *   地址 0x50 (A0-A2 → GND)
 */
#include "hardware/board.h"

I2C_HandleTypeDef hi2c_eep;

int eep_read(uint16_t addr, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c_eep, EEP_I2C_ADDR, addr,
                            I2C_MEMADD_SIZE_16BIT, buf, len, 100);
}

int eep_write(uint16_t addr, const uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Write(&hi2c_eep, EEP_I2C_ADDR, addr,
                             I2C_MEMADD_SIZE_16BIT, (uint8_t *)buf, len, 100);
}

void i2c_eep_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_AF_OD;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin   = EEP_SCL_PIN | EEP_SDA_PIN;
    HAL_GPIO_Init(EEP_SCL_PORT, &g);

    hi2c_eep.Instance              = I2C1;
    hi2c_eep.Init.ClockSpeed       = 100000;
    hi2c_eep.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    hi2c_eep.Init.OwnAddress1      = 0;
    hi2c_eep.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c_eep.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c_eep.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c_eep);
}
