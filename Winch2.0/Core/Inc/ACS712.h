/*
 * ACS712.h
 *
 *  Created on: Aug 9, 2022
 *      Author: LENOVO
 */

#ifndef INC_ACS712_H_
#define INC_ACS712_H_

#include "main.h"

#define ADC_SCALE_10		1023
#define ADC_SCALE_12		(4096 - 1)

#define VREF_5v5			5.0
#define VREF_3v3			3.3f


typedef enum{

	ACS712_05A,
	ACS712_20A,
	ACS712_30A
}ACS712_Sel_t;


typedef struct{

	uint16_t zero;
	float I;

	float rawVoltage;
	__IO uint32_t rawAdc;

}ACS712_Handle_t;


uint8_t ACS712_Init();

uint8_t Calibarate();

void SetZeroPoint(uint32_t _zero);

float GetRawVoltage(ACS712_Handle_t *acs);
float GetCurrentDC(ACS712_Handle_t *acs);


#endif /* INC_ACS712_H_ */
