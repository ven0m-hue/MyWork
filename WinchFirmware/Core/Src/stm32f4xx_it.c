/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern GPIO_InitTypeDef tim3ch1gpio;
extern I2C_HandleTypeDef hi2c1;
extern AS5600_Handle_t as5600;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t receivedData;
extern uint8_t data_buffer[5];
extern uint32_t count;
extern bool recepCmplt;
extern int16_t Data;
extern uint32_t Data1;
extern UART_HandleTypeDef huart1;

extern I2C_HandleTypeDef hi2c1;
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern bool Start_Flag;

extern bool bay_door_close;
extern bool close_door;

extern bool poop_back;
extern bool spring_trig;
//UART1 Communication Var
char* data_btn = "Spring Thing!!!!";
char* E_Stop = "Emergency Stop";
char* parked = "Payload Parked";
extern uint16_t rawAngle;
/* External variables --------------------------------------------------------*/
//extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
//extern DMA_HandleTypeDef hdma_adc1;
//extern ADC_HandleTypeDef hadc1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{

  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}


void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}


void USART1_IRQHandler(void)
{
	HAL_UART_Receive(&huart1, &receivedData, 1, HAL_MAX_DELAY);
		if (receivedData == '\r')
			{
				recepCmplt = true;
				Data = atoi((char*)data_buffer);

				data_buffer[count++] = '\r';

				Data1 = -Data;

				if(Data >= 0)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //GPIOC and PIN_0 changed
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(Data)/100);

				}

				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //GPIOC and PIN_0 changed
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(-Data)/100);

				}


				HAL_UART_Transmit(&huart1, data_buffer, count, HAL_MAX_DELAY);  //send to terminal
				memset(data_buffer, 0, sizeof(data_buffer));
				count = 0;
		}

		else if(receivedData == ' ')
		{

			HAL_UART_Transmit(&huart1, (uint8_t *)E_Stop, strlen(E_Stop), HAL_MAX_DELAY);  //send to terminal
			memset(data_buffer, 0, sizeof(data_buffer));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);
			//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		}
		else if(receivedData == 'r')
		{

			HAL_GPIO_Init(GPIOA, &tim3ch1gpio);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		}

		else if(receivedData == 'v')
		{

		}

		else if(receivedData == 's')
		{
			//curr = false;
			Start_Flag = true;
		}

		else
		{
			data_buffer[count++] = receivedData;
		}

	return;

	HAL_UART_IRQHandler(&huart1);

}

void USART2_IRQHandler(void)
{
	/*
	 * This handler is reserved for the Mavlink interface. Future Implementation.
	 */
	HAL_UART_IRQHandler(&huart2);
}
/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
	/*
	 * This is for the Bay Roof. After triggering...
	 * 1.Turn off the winch motor --> Currently it does not solve the double triggering issue.
	 * 2.Initiate the bay close door seq.
	 *
	 * PB0
	 */

	if(close_door)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)parked, strlen(parked), HAL_MAX_DELAY);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);


		//Flip the bay_door flag to initiate bayclose door seq
		bay_door_close = true;


		HAL_GPIO_WritePin(winch_dir_GPIO_Port, winch_dir_Pin, GPIO_PIN_RESET);

		/*
		 * Does not work well with the 2 channel motor driver
		 *
			for(int i =0; i<6000; i++){

				__HAL_TIM_SET_COMPARE(&hhtim3, TIM_CHANNEL_1, hhtim3.Init.Period * _8_BIT_MAP(PAYLOAD_2)/100);
			}
		*/
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			spring_trig = false;
	}


  HAL_GPIO_EXTI_IRQHandler(roof_top_ext_Pin);

}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
	/*
	 *  This subroutine handles the Spring thing interrupt
	 *
	 *  PB3
	 */

 	if(poop_back)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)data_btn, strlen(data_btn), HAL_MAX_DELAY);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * 0/100);

		for(int i =0; i<12000; i++)
		{

			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(PAYLOAD_3)/100);

		}

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * 0/100);
		//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);


		poop_back = false;
		spring_trig = true;

	}

  HAL_GPIO_EXTI_IRQHandler(spring_thing_ext_Pin_Pin);

}

void I2C1_EV_IRQHandler(void)
{
	rawAngle = as5600.rawAngle;
}
///**
//  * @brief This function handles ADC1 global interrupt.
//  */
//void ADC_IRQHandler(void)
//{
//  /* USER CODE BEGIN ADC_IRQn 0 */
//
//  /* USER CODE END ADC_IRQn 0 */
//  HAL_ADC_IRQHandler(&hadc1);
//  /* USER CODE BEGIN ADC_IRQn 1 */
//
//  /* USER CODE END ADC_IRQn 1 */
//}
//
///**
//  * @brief This function handles DMA2 stream0 global interrupt.
//  */
//void DMA2_Stream0_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
//
//  /* USER CODE END DMA2_Stream0_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_adc1);
//  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */
//
//  /* USER CODE END DMA2_Stream0_IRQn 1 */
//}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
//void OTG_FS_IRQHandler(void)
//{
//  /* USER CODE BEGIN OTG_FS_IRQn 0 */
//
//  /* USER CODE END OTG_FS_IRQn 0 */
//  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
//  /* USER CODE BEGIN OTG_FS_IRQn 1 */
//
//  /* USER CODE END OTG_FS_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
