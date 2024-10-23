/*
 * sensor.h
 *
 *  Created on: Oct 23, 2024
 *      Author: Agung Ibnu
 */

#ifndef DRIVERS_SENSOR_SENSOR_H_
#define DRIVERS_SENSOR_SENSOR_H_

#include "stm32l4xx_hal.h"

typedef struct {
	float temperature;
	float humidity;
}SensorData;

void sensor_init(void);
SensorData sensor_readData(void);

float get_barometric();

#endif /* DRIVERS_SENSOR_SENSOR_H_ */
