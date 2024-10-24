/*
 * bmp280.c
 *
 *  Created on: Oct 23, 2024
 *      Author: Agung Ibnu
 */
#include "stm32l4xx_hal.h"
#include "main.h"
#include "math.h"
#include "stdint.h"

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
#define BMP280_CALIB00_REG    0x88  // Calib

// Define the BMP280 types
typedef int32_t BMP280_S32_t;
typedef uint32_t BMP280_U32_t;
typedef int64_t BMP280_S64_t;

extern I2C_HandleTypeDef hi2c1;

// Declare the calibration parameters (you should read these from BMP280)
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
BMP280_S32_t t_fine;

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
    serialPrint("BMP280 Initialization");
}

int32_t BMP280_GetRawTemperature(void) {
    uint8_t temp_data[3];  // Array to store the three temperature registers

    // Read the three temperature registers (MSB, LSB, XLSB)
    BMP280_ReadRegister(BMP280_REG_TEMP_MSB, temp_data, 3);

    // Combine the three parts into a 20-bit value
    int32_t raw_temp = (int32_t)((((uint32_t)temp_data[0]) << 12) |
                                 (((uint32_t)temp_data[1]) << 4)  |
                                 (((uint32_t)temp_data[2]) >> 4));

    return raw_temp;  // Return the raw 20-bit temperature value
}

int32_t BMP280_GetRawPressure(void) {
    uint8_t press_data[3];  // Array to store the three pressure registers

    // Read the three pressure registers (MSB, LSB, XLSB)
    BMP280_ReadRegister(BMP280_REG_PRESS_MSB, press_data, 3);

    // Combine the three parts into a 20-bit value
    int32_t raw_press = (int32_t)((((uint32_t)press_data[0]) << 12) |
                                  (((uint32_t)press_data[1]) << 4)  |
                                  (((uint32_t)press_data[2]) >> 4));

    return raw_press;  // Return the raw 20-bit pressure value
}

void BMP280_ReadCalibrationData(void) {
    uint8_t calib_data[24];

    // Read 24 bytes of calibration data from the BMP280
    BMP280_ReadRegister(BMP280_CALIB00_REG, calib_data, 24);

    // Parse the calibration data into the appropriate variables
    dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dig_T3 = (calib_data[5] << 8) | calib_data[4];
    dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dig_P9 = (calib_data[23] << 8) | calib_data[22];
}

// Compensation function for temperature

// Compensation function for temperature
double bmp280_compensate_T_int32(BMP280_S32_t adc_T) {
    double var1, var2, T;

    var1 = (((double)adc_T)/16382.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
    var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0)) * ((double)dig_T3);
    t_fine = (BMP280_S32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0;
    return T;
}

// Compensation function for pressure
double bmp280_compensate_P_int32(BMP280_S32_t adc_P) {
   double var1, var2, p;

   var1 = ((double)t_fine/2.0) - 64000.0;
   var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
   var2 = var2 + var1 * ((double)dig_P5) * 2.0;
   var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
   var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
   var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
   if (var1 == 0.0){
	   return 0;
   }

   p = 1048576.0 - (double)adc_P;
   p = (p - (var2 / 4096.0)) * 6250.0 / var1;
   var1 = ((double)dig_P9) * p * p / 2147483648.0;
   var2 = p * ((double)dig_P8) / 32768.0;
   p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
   return p * 1.0f;

}

// Function to read and calculate pressure in hPa
float BMP280_GetPressure_hPa(void) {
    int32_t raw_pressure;
    BMP280_ReadCalibrationData();
    // Read raw temperature and pressure data from BMP280
    raw_pressure = BMP280_GetRawPressure();  // Implement this function to read raw pressure
    // Compensate the pressure and convert to hPa
    float pressure_pa = bmp280_compensate_P_int32(raw_pressure);
    return pressure_pa / 100.0f;  // Convert Pa to hPa
}

float BMP280_GetTemperature(void){
	int32_t raw_temp;
	BMP280_ReadCalibrationData();
	raw_temp = BMP280_GetRawTemperature();  // Implement this function to read raw temp
	 // Compensate the temperature to get fine temperature value
	float temperature_c =  bmp280_compensate_T_int32(raw_temp);
	return temperature_c * 1.0f;
}
