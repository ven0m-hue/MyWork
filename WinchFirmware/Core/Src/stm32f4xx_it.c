/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
//Encoder Variables
extern uint32_t Clicks;
extern int16_t click;
extern int16_t Pulse;

//Spring thing variables
extern bool poop_back;
extern bool spring_trig;

//Bay Door variables
extern bool close_door;
extern bool bay_door_close;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
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

  while (1){}

}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{

  while (1){}
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1){}
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{

  while (1){}
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{

  while (1){}
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{

}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
   HAL_IncTick();
   HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers*/
/******************************************************************************/

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_adc1);

}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{

  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);

}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
	//tim2.Instance->CNT
	Clicks = __HAL_TIM_GET_COUNTER(&htim2);
	click = (int16_t)Clicks;
	Pulse = click * 0.25;

	HAL_TIM_IRQHandler(&htim2);
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

/**
  * @brief This function handles External global interrupt 0.
  *
  */

void EXTI0_IRQHandler()
{

	/*
	 *  This subroutine handles the Spring thing interrupt
	 *
	 *  PB0
	 */
	if(poop_back)
	{
		//HAL_UART_Transmit(&huart2, (uint8_t *)data_btn, strlen(data_btn), HAL_MAX_DELAY);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * 0/100);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);


		poop_back = false; // Disable the flag
		spring_trig = true; // Activate the flag

	}

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}

/**
  * @brief This function handles External global interrupt 3.
  *
  */

void EXTI3_IRQHandler()
{

	/*
	 * This is for the Bay Roof. After triggering...
	 * 1.Turn off the winch motor --> Currently it does not solve the double triggering issue.
	 * 2.Initiate the bay close door seq.
	 *
	 * PC3
	 */

	if(close_door)
	{
		//HAL_UART_Transmit(&huart2, (uint8_t *)parked, strlen(parked), HAL_MAX_DELAY);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);


		//Flip the bay_door flag to initiate bayclose door seq
		bay_door_close = true;


		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

		/*
		 * Does not work well with the 2 channel motor driver
		 *
			for(int i =0; i<6000; i++){

				__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(PAYLOAD_2)/100);
			}
		*/

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);
	}


	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);

}

