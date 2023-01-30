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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

/* USER CODE END Includes */
#include "PID.h"
#include "AS5600.h"

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */
#include "common\mavlink.h"
/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define blue_led_Pin GPIO_PIN_13
#define blue_led_GPIO_Port GPIOC
#define encoder_a_Pin GPIO_PIN_0
#define encoder_a_GPIO_Port GPIOA
#define encoder_b_Pin GPIO_PIN_1
#define encoder_b_GPIO_Port GPIOA
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
#define roof_top_ext_EXTI_IRQn EXTI0_IRQn
#define bay_dir_Pin GPIO_PIN_8
#define bay_dir_GPIO_Port GPIOA
#define spring_thing_ext_Pin_Pin GPIO_PIN_3
#define spring_thing_ext_Pin_GPIO_Port GPIOB
#define spring_thing_ext_Pin_EXTI_IRQn EXTI3_IRQn
#define pixhawk_signal_Pin GPIO_PIN_6
#define man_winch_Pin GPIO_PIN_10
#define pixhawk_signal_GPIO_Port GPIOB
#define man_winch_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */



//System Macros
#define SYS_CLOCK_FREQ_50MHz  			50
#define SYS_CLOCK_FREQ_80MHz 			80
#define SYS_CLOCK_FREQ_120MHz			120
#define SYS_CLOCK_FREQ_180MHz			180



//MISC Macros
#define DutyCycle(X)      ((X)*0.033 + 33)
#define DutyCycleServo(X) ((X)*0.3611111111 + 15)

#define _8_BIT_MAP(X)         ((X)*0.3921568627)

#define __PI				3.14159265


#define RAD2DEG(X)			((X)*(180/__PI))


#define __PWM_CCR_CHECK		2.5098

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
#define PWM_INTERMITANT_UP     40   //90 //90



#define PWM_RAMP_DOWN_DURATION  120//For heavier payload  lighter payload ->75

#define PWM_WINCH_DOWN_RAMP_DOWN_DURATION	30

#define PWM_CONSTANT 			60


#define ENCODER_RAMP_UP_COUNT 		PWM_RAMP_UP_DURATION * 205
#define ENCODER_RAMP_DOWN_COUNT 	(ENCODER_RAMP_UP_COUNT + PWM_RAMP_DOWN_DURATION * 255)

#define PWM_ON_DELAY(X)		((X)*0.011776)

#define GP_DIV				2



#define DELAY_255			3000
#define PWM_255				255


//This part is for BOMBAY_DOORS
#define BOMBAY_OPEN_CLOSE			255
#define BOMBAY_DOOR_ONOFF_TIME		1600


//Payload Mass Macros for re-wind after roof touch

#define PAYLOAD_0		20
#define PAYLOAD_2		20
#define PAYLOAD_3		30

//Payload Mass based PWM
#define PWM_PAYLOAD_0   PAYLOAD_0
#define PWM_PAYLOAD_2   PAYLOAD_2
#define PWM_PAYLOAD_3   PAYLOAD_3
#define PWM_STOP		0

#define PWM_PAYLOAD_2o 40


//For PixHawk Interfacing
#define THROTTLE_FULL		1900
#define THROTTLE_HALF		1450
#define THROTTLE_NULL		1100


//For Manual Winch
#define MAN_WINCH_UP	THROTTLE_FULL
#define MAN_WINCH_STOP  THROTTLE_HALF
#define MAN_WINCH_DOWN  THROTTLE_NULL


//Timer IC
#define TIMCLOCK   			90000000
#define PRESCALAR  			90

//Spring Thing
#define POOP_BACK_AT_H		14.00 //This is in meters, on when to activate the spring thing.

//Magnetic Encoder
#define __RADIUS			1.8//2.3  //This is in centi meters which is later converted to the meters.

//PController Macros
#define LEN_TO_WINCH_DOWN	25//21.00
#define THRESHOLD_LEN		20//16.4//18.00  //In meters.


#define WHAT_PAYLOAD	PAYLOAD_2

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
