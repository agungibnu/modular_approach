/*
 * sensor.c
 *
 *  Created on: Oct 23, 2024
 *      Author: Agung Ibnu
 */


#include "sensor.h"
#include "main.h"

void sensor_init(){

	for (int i = 0; i < 5; i++){
		serialPrint("Sensor initializing");
		HAL_Delay(1000);
	}
}

SensorData sensor_readData(){

	SensorData data;

	data.temperature = 25.5f;
	data.humidity = 86.7f;

	return data;
}

float get_barometric(){

	float data;

	data = 1009.12;

	return data;
}
