/*
 * bmp280.c
 *
 *  Created on: Oct 23, 2024
 *      Author: Agung Ibnu
 */


#include "stm32l4xx_hal.h"


// BMP280 I2C address (SDO = GND: 0x76, SDO = VCC: 0x77)
#define BMP280_ADDRESS 0x76 << 1  // Shift left by 1 for 7-bit address format

// BMP280 register addresses
#define BMP280_REG_TEMP_MSB   0xFA  // Temperature MSB register
#define BMP280_REG_TEMP_LSB   0xFB  // Temperature LSB register
#define BMP280_REG_TEMP_XLSB  0xFC  // Temperature XLSB register
#define BMP280_REG_PRESS_MSB  0xF7  // Pressure MSB register
#define BMP280_REG_PRESS_LSB  0xF8  // Pressure LSB register
#define BMP280_REG_PRESS_XLSB 0xF9  // Pressure XLSB register
#define BMP280_REG_CTRL_MEAS  0xF4  // Control measurement register
#define BMP280_REG_CONFIG     0xF5  // Configuration register

extern I2C_HandleTypeDef hi2c1;

// Function to write to a BMP280 register
HAL_StatusTypeDef BMP280_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDRESS, data, 2, HAL_MAX_DELAY);
}

// Function to read from a BMP280 register
HAL_StatusTypeDef BMP280_ReadRegister(uint8_t reg, uint8_t *data, uint16_t len) {
    // Send the register address to read from
    HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    // Read the register data
    return HAL_I2C_Master_Receive(&hi2c1, BMP280_ADDRESS, data, len, HAL_MAX_DELAY);
}

// Function to initialize the BMP280 sensor
void BMP280_Init(void) {
    // Example: Set to normal mode with oversampling settings
    BMP280_WriteRegister(BMP280_REG_CTRL_MEAS, 0x27);  // Temp x1, Press x1, Normal mode
    BMP280_WriteRegister(BMP280_REG_CONFIG, 0xA0);  // Config settings (standby, filter)
}

int32_t BMP280_ReadTemperature(void) {
    uint8_t temp_raw[3];
    BMP280_ReadRegister(BMP280_REG_TEMP_MSB, temp_raw, 3);
    int32_t temp = (int32_t)(((uint32_t)(temp_raw[0] << 16) | (uint32_t)(temp_raw[1] << 8) | temp_raw[2]) >> 4);
    return temp;
}



