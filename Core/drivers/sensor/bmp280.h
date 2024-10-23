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

#endif /* DRIVERS_SENSOR_BMP280_H_ */
