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

extern UART_HandleTypeDef huart2;
extern uint8_t receivedData;
extern uint8_t data_buffer[5];
extern uint32_t count;
extern bool recepCmplt;
extern uint32_t Data;

extern UART_HandleTypeDef huart1;
extern uint8_t receivedData1;
extern uint8_t data_buffer1[5];
extern uint32_t count1;
extern bool recepCmplt1;
extern uint32_t Data1;



void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&tim2);
}


void USART2_IRQHandler(void)
{
	HAL_UART_Receive(&huart2, &receivedData, 1, HAL_MAX_DELAY);
		if (receivedData == '\r')
			{
				recepCmplt = true;
				Data = atoi((char*)data_buffer);

				data_buffer[count++] = '\r';

			__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * DutyCycle(Data)/100);
			HAL_UART_Transmit(&huart2, data_buffer, count, HAL_MAX_DELAY);  //send to terminal
			memset(data_buffer, 0, sizeof(data_buffer));
			count = 0;
		}
		else
		{
			data_buffer[count++] = receivedData;

		}

	return;

	HAL_UART_IRQHandler(&huart2);
}

//void USART1_IRQHandler(void)
//{
//
//
//	HAL_UART_Receive(&huart1, &receivedData1, 1, HAL_MAX_DELAY);
//		if (receivedData1 == '\r')
//			{
//				recepCmplt1 = true;
//				Data1 = atoi((char*)data_buffer1);
//
//				data_buffer1[count1++] = '\r';
//
//
//			HAL_UART_Transmit(&huart2, data_buffer1, count1, HAL_MAX_DELAY);  //send to terminal
//			memset(data_buffer, 0, sizeof(data_buffer1));
//			count1 = 0;
//		}
//		else
//		{
//			data_buffer1[count1++] = receivedData1;
//
//		}
//
//	return;
//
//	HAL_UART_IRQHandler(&huart1);
//}

void EXTI9_5_IRQHandler()
{
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_9)){
			GPIOA -> ODR ^= GPIO_PIN_5; // toggle LD2 LED
		 }

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}
//void EXTI15_10_IRQHandler(void)
//{
//	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)){
//		GPIOA -> ODR ^= GPIO_PIN_5; // toggle LD2 LED
//		 }
//
//	__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * 0/100);
//
//	while(1);
//
//	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
//}
