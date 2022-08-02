/*
 * main.h
 *
 *  Created on: Jun 22, 2021
 *      Author: 91900
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define SYS_CLOCK_FREQ_50MHz  			50
#define SYS_CLOCK_FREQ_80MHz 			80
#define SYS_CLOCK_FREQ_120MHz			120
#define SYS_CLOCK_FREQ_180MHz			180


#define DutyCycle(X)      ((X)*0.033 + 33)
#define DutyCycleServo(X) ((X)*0.3611111111 + 15)

#define _8_BIT_MAP(X)         ((X)*0.3921568627)

#define __PI				3.14159265



//////////////////////////WINCH MACROS////////////////////////////////////

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


#define ENCODER_RAMP_UP_COUNT 		PWM_RAMP_UP_DURATION * 205
#define ENCODER_RAMP_DOWN_COUNT 	(ENCODER_RAMP_UP_COUNT + PWM_RAMP_DOWN_DURATION * 255)

#define PWM_ON_DELAY(X)		((X)*0.011776)

#define GP_DIV				2



#define DELAY_255			3000
#define PWM_255				255


//This part is for BOMBAY_DOORS
#define BOMBAY_OPEN_CLOSE			255
#define BOMBAY_DOOR_ONOFF_TIME		1500


//Payload Mass Macros for re-wind after roof touch

#define PAYLOAD_0		20
#define PAYLOAD_2		20
#define PAYLOAD_3		30

#endif /* MAIN_H_ */
