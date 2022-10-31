/*
 * msp.c
 *
 *  Created on: Jun 22, 2021
 *      Author: Venom
 */

#include "main.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_adc1;

extern void Error_handler();

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

	tim3ch1gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 ; // Added GPIO Pin to Control the LED using PWM
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


void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  if(htim_ic->Instance==TIM1)
	  {

	    /* Peripheral clock enable */
	    __HAL_RCC_TIM1_CLK_ENABLE();

	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    /**TIM1 GPIO Configuration
	    PA9     ------> TIM1_CH2
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_9;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	  }
	  else if(htim_ic->Instance==TIM4)
	  {

	    /* Peripheral clock enable */
	    __HAL_RCC_TIM4_CLK_ENABLE();

	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    /**TIM4 GPIO Configuration
	    PB6     ------> TIM4_CH1
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_6;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    /* TIM4 interrupt Init */
	    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(TIM4_IRQn);

	  }

}


void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* htim_ic)
{
  if(htim_ic->Instance==TIM1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PA9     ------> TIM1_CH2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

  }
  else if(htim_ic->Instance==TIM4)
  {

    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    /* TIM4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

  }

}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {

    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA4     ------> ADC1_IN4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_adc1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_adc1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}

