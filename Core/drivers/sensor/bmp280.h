/*
 * bmp280.h
 *
 *  Created on: Oct 23, 2024
 *      Author: Agung Ibnu
 */

#ifndef DRIVERS_SENSOR_BMP280_H_
#define DRIVERS_SENSOR_BMP280_H_





void BMP280_Init(void);

int32_t BMP280_ReadTemperature(void);
float BMP280_CompensateTemperature(int32_t raw_temp);

float BMP280_GetPressure_hPa(void);

#endif /* DRIVERS_SENSOR_BMP280_H_ */
