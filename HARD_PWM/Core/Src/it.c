/*
 * it.c
 *
 *  Created on: Jun 22, 2021
 *      Author: 91900
 */

#include "main.h"
extern TIM_HandleTypeDef tim2;
extern TIM_HandleTypeDef tim3;
extern TIM_HandleTypeDef tim6;
extern TIM_HandleTypeDef tim4;

extern GPIO_InitTypeDef tim3ch1gpio;

extern GPIO_InitTypeDef m_dir;

extern UART_HandleTypeDef huart2;
extern uint8_t receivedData;
extern uint8_t data_buffer[5];
extern uint32_t count;
extern bool recepCmplt;
extern int32_t Data;

extern UART_HandleTypeDef huart1;
extern uint8_t receivedData1;
extern uint8_t data_buffer1[5];
extern uint32_t count1;
extern bool recepCmplt1;
extern uint32_t Data1;


//Encdoer Vars
extern uint32_t Clicks;
extern int16_t click;
extern int16_t Pulse;

extern bool bay_door_close;

//Internal Variable
char* data_btn = "Pressed!!!!";
char* E_Stop = "Emergency Stop";
char* parked = "Payload Parked";

extern bool poop_back;
extern bool spring_trig;
//Core Tick Interrupt

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


//Timer Interrupt

void TIM2_IRQHandler(void)
{

	//tim2.Instance->CNT
	Clicks = __HAL_TIM_GET_COUNTER(&tim2);
	click = (int16_t)Clicks;
	Pulse = click * 0.25;

	HAL_TIM_IRQHandler(&tim2);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim4);
}




//UART Interrupt

void USART2_IRQHandler(void)
{
	HAL_UART_Receive(&huart2, &receivedData, 1, HAL_MAX_DELAY);
		if (receivedData == '\r')
			{
				recepCmplt = true;
				Data = atoi((char*)data_buffer);

				data_buffer[count++] = '\r';

				Data1 = -Data;

				if(Data >= 0)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //GPIOC and PIN_0 changed
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
					__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(Data)/100);

				}

				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //GPIOC and PIN_0 changed
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
					__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(-Data)/100);

				}


				HAL_UART_Transmit(&huart2, data_buffer, count, HAL_MAX_DELAY);  //send to terminal
				memset(data_buffer, 0, sizeof(data_buffer));
				count = 0;
		}

		else if(receivedData == ' ')
		{

			HAL_UART_Transmit(&huart2, (uint8_t *)E_Stop, strlen(E_Stop), HAL_MAX_DELAY);  //send to terminal
			memset(data_buffer, 0, sizeof(data_buffer));
			__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(0)/100);
			//HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

			HAL_TIM_PWM_Stop(&tim3, TIM_CHANNEL_1);
		}
		else if(receivedData == 'r')
		{

			HAL_GPIO_Init(GPIOA, &tim3ch1gpio);
		}

		else
		{
			data_buffer[count++] = receivedData;
		}

	return;

	HAL_UART_IRQHandler(&huart2);
}




///All the external interrupts

void EXTI1_IRQHandler()
{

	/*
	 *  This subroutine handles the Spring thing interrupt
	 *
	 *  PC1
	 */

	if(poop_back)
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)data_btn, strlen(data_btn), HAL_MAX_DELAY);

		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * 0/100);

		HAL_TIM_PWM_Stop(&tim3, TIM_CHANNEL_1);


		poop_back = false;
		spring_trig = true;

	}


	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}


void EXTI2_IRQHandler()
{
	/*
	 * This is for the Bay Roof. After triggering...
	 * 1.Turn off the winch motor --> Currently it does not solve the double triggering issue.
	 * 2.Initiate the bay close door seq.
	 *
	 * PC2
	 */

	HAL_UART_Transmit(&huart2, (uint8_t *)parked, strlen(parked), HAL_MAX_DELAY);

	__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(0)/100);


	//Flip the bay_door flag to initiate bayclose door seq
	bay_door_close = true;


	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	for(int i =0; i<6000; i++){

		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(PAYLOAD_2)/100);
	}

	__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(0)/100);

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI3_IRQHandler()
{
	/*
	 * This is for the Bay Door
	 *
	 * PC3
	 */

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI15_10_IRQHandler(void)

//PC13
{
	//HAL_Delay(10);
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)){
		GPIOA -> ODR ^= GPIO_PIN_5; // toggle LD2 LED
		 }

//	__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * 0/100);
//
//	while(1);

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}
