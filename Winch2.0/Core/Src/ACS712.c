/*
 * ACS712.c
 *
 *  Created on: Aug 9, 2022
 *      Author: LENOVO
 */


#include "ACS712.h"


//static uint8_t Scale = ACS712_20A

//static float sensitivity;

uint8_t ACS712_Init()
{

//	switch(Scale)
//	{
//
//	case ACS712_05A:
//
//		sensitivity = 0.185;
//		break;
//
//	case ACS712_20A:
//		sensitivity = 0.100;
//		break;
//
//	case ACS712_30A:
//		sensitivity = 0.066;
//		break;
//	}

	return 0;
}

uint8_t Calibarate()
{

	return 0;
}

float GetCurrentDC(ACS712_Handle_t *acs)
{
	float rawVolt = 0.0f;
	rawVolt = (float) acs->rawAdc * (VREF_3v3 / ADC_SCALE_12) ;

	acs->rawVoltage = rawVolt;
	acs->I = (rawVolt - 2.5) / 0.1;
	return acs->I;
}
