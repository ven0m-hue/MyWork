/*
 * msp.c
 *
 *  Created on: Jun 22, 2021
 *      Author: Venom
 */

#include "main.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;


GPIO_InitTypeDef tim2ch1gpio;
GPIO_InitTypeDef tim3ch1gpio;

void HAL_MspInit(void)
{
	/*
	 * Low level initialization
	 * 1. Set up the priority grouping of the arm cortex mx processor
	 * 2. Enable the required sustem exceptions of the arm cortex mx processors
	 * 3. Configure the Priority for the system exceptions
	 */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	SCB->SHCSR |= 0x7 << 16; // usage fault, memory fault, bus fault systems

	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{

	/*
	 * 1.Enable the peripheral clocks
	 * 2.Config the gpio pins to its respective alternate functionality pins refer datasheet
	 * PA0 ->ch1, PA1->ch2, PB10->ch3, PB11->ch4
	 * 3.Enable the interrupt priority and interrupt
	 */
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	tim3ch1gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6; // Added GPIO Pin to Control the LED using PWM
	tim3ch1gpio.Mode = GPIO_MODE_AF_PP;
	tim3ch1gpio.Alternate = GPIO_AF2_TIM3; // According to the data sheet
	HAL_GPIO_Init(GPIOA, &tim3ch1gpio);

	tim2ch1gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	tim2ch1gpio.Mode = GPIO_MODE_AF_PP;
	tim2ch1gpio.Alternate = GPIO_AF1_TIM2; // According to the data sheet
	HAL_GPIO_Init(GPIOB, &tim2ch1gpio);

	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_SetPriority(TIM2_IRQn, 14, 0);


}


void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	/*
	 * here we are going to do the low level inits. of the USART2 peripheral
	 * 1. Enable the clock of USART2 peripheral and GPIOA peripheral
	 * 2. Do the pin muxing config
	 * 3. Enable the IRQ and set up the priority
	 */
	GPIO_InitTypeDef gpio_uart;

	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();

	 gpio_uart.Pin = GPIO_PIN_2 | GPIO_PIN_9;// UART2_TX AND UART1_TX
	 gpio_uart.Mode = GPIO_MODE_AF_PP;
	 gpio_uart.Pull = GPIO_PULLUP;
	 gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
	 gpio_uart.Alternate =  GPIO_AF7_USART2; // Alternate functionality for TX_RX
	 HAL_GPIO_Init(GPIOA,&gpio_uart);

	 gpio_uart.Pin = GPIO_PIN_3 | GPIO_PIN_10 ;//UART1_RX
	 HAL_GPIO_Init(GPIOA,&gpio_uart);

	 HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn,3,0);

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_SetPriority(USART1_IRQn,6,0);

	 __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);


}



void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM2)
  {

    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0-WKUP     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

  }

}

