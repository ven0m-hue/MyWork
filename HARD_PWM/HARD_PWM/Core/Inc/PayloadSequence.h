/*
 * PayloadSequence.h
 *
 *  Created on: 08-Aug-2022
 *      Author: LENOVO
 */

#ifndef INC_PAYLOADSEQUENCE_H_
#define INC_PAYLOADSEQUENCE_H_

#include "main.h"

typedef struct
{
	//Gp Variables
	uint16_t gp_i;
	uint32_t tau;		//Threshold

	//Encoder Vars
	uint32_t Counts;
	uint32_t Length;

	bool forward;
	bool backward;


}PayloadSequence_Handle_t;


////////////////////////////////////////////////////WinchAPIs////////////////////////////

void Winch_Down_Ramp_Up_Down();
void Winch_Down_Gp_Sequnece();
void Winch_Up_Ramp_Up_Down();
void Bombay_Door_Open();
void Bombay_Door_Close();
void Winch_Motor_Direction_Set();
void Bombay_Door_Direction_Set();
bool Bay_Door_Close_Or_Not();


///////////////////////////////////////////////WINCHMACROS/////////////////////////////

//Winch Motor Mapping to the DutyCycle
#define _8_BIT_MAP(X)         ((X)*0.3921568627)

#define __PI				3.14159265


#define MAX_DUTYCYCLE 			255   //16bit timer

#define PWM_START 				121
#define INTERMITENT_DC			180


//For Winch Up start
#define PWM_UP_START			30

#define STEP 					1    // Mapping it with the 8bit 255 gives each step 257 ie. 65535/255 = 257

#define PWM_FIXED				120
#define PWM_FIXED_START			PWM_FIXED


#define PWM_RAMP_UP_DURATION   35
#define PWM_INTERMITANT_UP     20   //90 //90



#define PWM_RAMP_DOWN_DURATION  85//For heavier payload  lighter payload ->75

#define PWM_WINCH_DOWN_RAMP_DOWN_DURATION	30

#define PWM_CONSTANT 			30

//Encoder Display Time
#define ENCODER_RAMP_UP_COUNT 		PWM_RAMP_UP_DURATION * 205
#define ENCODER_RAMP_DOWN_COUNT 	(ENCODER_RAMP_UP_COUNT + PWM_RAMP_DOWN_DURATION * 255)

#define PWM_ON_DELAY(X)		((X)*0.011776)


//GP Sequence
#define GP_DIV				2



#define DELAY_255			3000
#define PWM_255				255


//This part is for BOMBAY_DOORS
#define BOMBAY_OPEN_CLOSE			255
#define BOMBAY_DOOR_ONOFF_TIME		1700


//Payload Mass Macros for re-wind after roof touch

#define PAYLOAD_0		20
#define PAYLOAD_2		20
#define PAYLOAD_3		30

#endif /* INC_PAYLOADSEQUENCE_H_ */
