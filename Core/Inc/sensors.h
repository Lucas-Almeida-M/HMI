/*
 * sensors.h
 *
 *  Created on: Sep 17, 2024
 *      Author: lucas
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "cmsis_os2.h"
#include "main.h"

#define MEDIA_MOVEL 50

#define MAX_NUM_ADC 5

typedef struct Adc_values_
{
	uint16_t windowNum;
	uint16_t numSamples;
	uint32_t sum;
	uint16_t mean;
}Adc_values;

typedef struct DC_meas_
{
	Adc_values adc[MAX_NUM_ADC];
	uint64_t sampleCnt;
}DC_meas;


typedef struct Sensors_Val_
{
	uint16_t adcVal[MAX_NUM_ADC];
}Sensors_Val;

void adc_update_sample(void);
void adc_get_values(uint16_t *values);


#endif /* INC_SENSORS_H_ */
