/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define encoder_a_Pin GPIO_PIN_0
#define encoder_a_GPIO_Port GPIOA
#define encoder_b_Pin GPIO_PIN_1
#define encoder_b_GPIO_Port GPIOA
#define spring_thing_ext_Pin GPIO_PIN_3
#define spring_thing_ext_GPIO_Port GPIOA
#define curr_sensor_Pin GPIO_PIN_4
#define curr_sensor_GPIO_Port GPIOA
#define winch_dir_Pin GPIO_PIN_5
#define winch_dir_GPIO_Port GPIOA
#define winch_pwm_Pin GPIO_PIN_6
#define winch_pwm_GPIO_Port GPIOA
#define bay_door_pwm_Pin GPIO_PIN_7
#define bay_door_pwm_GPIO_Port GPIOA
#define roof_top_ext_Pin GPIO_PIN_0
#define roof_top_ext_GPIO_Port GPIOB
#define bay_dir_Pin GPIO_PIN_8
#define bay_dir_GPIO_Port GPIOA
#define pixhawk_signal_Pin GPIO_PIN_6
#define pixhawk_signal_GPIO_Port GPIOB



/*Private Macros*/


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

#define PWM_CONSTANT 			40


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

//Current Sensor
#define ADC_SCALE_10		1023
#define ADC_SCALE_12		(4096 - 1)

#define VREF_5v5			5.0
#define VREF_3v3			3.3f


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
