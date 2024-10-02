/*
 * sensors.c
 *
 *  Created on: Sep 17, 2024
 *      Author: lucas
 */


#include "sensors.h"
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"

DC_meas dcMeas;
extern uint16_t adcint[];
extern osMessageQueueId_t sensors_queueHandle;

void dc_meas_init(void)
{
	memset(&dcMeas , 0, sizeof(dcMeas));

	dcMeas.adc[0].windowNum = MEDIA_MOVEL;
	dcMeas.adc[1].windowNum = MEDIA_MOVEL;
	dcMeas.adc[2].windowNum = MEDIA_MOVEL;
	dcMeas.adc[3].windowNum = MEDIA_MOVEL;
	dcMeas.adc[4].windowNum = MEDIA_MOVEL;
	dcMeas.sampleCnt = 0;
}

void adc_update_sample(void)
{
	for (int i = 0; i < MAX_NUM_ADC; i++ )
	{
		if (dcMeas.adc[i].numSamples >= dcMeas.adc[i].windowNum)
		{
			dcMeas.adc[i].sum += (uint16_t) ( (float) (adcint[i] * 0.08 * 10) );
			dcMeas.adc[i].sum -= dcMeas.adc[i].sum / (dcMeas.adc[i].windowNum + 1);
			dcMeas.adc[i].mean = dcMeas.adc[i].sum / dcMeas.adc[i].windowNum;
		}
		else
		{
			dcMeas.adc[i].numSamples++;
			dcMeas.adc[i].sum += adcint[i];
			dcMeas.adc[i].mean = dcMeas.adc[i].sum / dcMeas.adc[i].numSamples;
		}
	}
	dcMeas.sampleCnt++;
	if (dcMeas.sampleCnt % 100 == 0)
	{
		Sensors_Val sensorsVal = {0};
		for (int i = 0; i <= MAX_NUM_ADC; i++ )
		{
			sensorsVal.adcVal[i] = dcMeas.adc[i].mean;
		}
		xQueueSendToBackFromISR(sensors_queueHandle, &sensorsVal, 0);
	}

}
