/*
 * main.c
 *
 *  Created on: July 30, 2022
 *      Author: Venom
 */

//Includes
#include "main.h"



#define MAX_DUTYCYCLE 			255   //16bit timer
#define STEP 					1     // Mapping it with the 8bit 255 gives each step 257 ie. 65535/255 = 257



//Prototypes
void SystemClockConfig(uint8_t clock_freq);
void Error_handler(void);
void UART2_Init(void);
void UART1_Init(void);
void Timer3_Init(void);
void Timer2_Init(void);
void Timer6_Init(void);
void LSE_Config(void);
void GPIO_Init(void);

void MX_MOTO_RAMP_UP(void);
void MX_MOTO_RAMP_DOWN(void);
//static void MX_MOTO_RAMP_UP(void);
//static void MX_AUTO_STOP(void);
//void CalibrateFunction();

// Handle variable of general purpose timer 2 made global
TIM_HandleTypeDef tim3;
TIM_HandleTypeDef tim2;
TIM_OC_InitTypeDef timerPWMconfig;


UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;
//PID_Handle_t pid;
//Pwm handle made global


//Global variables
uint32_t clock_freq;  // for clock config
uint32_t flash_latency;

//UART2
uint8_t receivedData;
uint8_t data_buffer[5];
uint32_t count = 0;
bool recepCmplt = false;
uint32_t Data;
//UART1
uint8_t receivedData1;
uint8_t data_buffer1[5];
uint32_t count1 = 0;
bool recepCmplt1 = false;
uint32_t Data1 = 0;

bool calib = false;
uint32_t PID;
uint16_t counter = 0;
uint16_t i = 0;
uint16_t indx = 0;
bool btn_pressed = false;

uint8_t buf[64];

uint32_t tick = 1;

void Universal_Inits() {

	HAL_Init();
	SystemClockConfig(SYS_CLOCK_FREQ_50MHz);
	LSE_Config();
	Timer3_Init();
	Timer2_Init();
	UART2_Init();
	UART1_Init();
	GPIO_Init();
	//MX_PID_Init();
}


int main()
{
	//Inits



	Universal_Inits();


	//uint16_t brightness = 0;


	memset(buf, 0, sizeof(buf));

	//Transmit to the terminal at start
	char* user_data = "REDWING LABS\r\n";
	uint16_t data_len = strlen(user_data);
	HAL_UART_Transmit(&huart2, (uint8_t*)user_data, data_len, HAL_MAX_DELAY);

	HAL_TIM_Base_Start_IT(&tim2);

	if (HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_1) != HAL_OK) Error_handler();


	//MX_MOTO_RAMP_UP();   //ACCEL Phase

	//HAL_Delay(1000);   //Const Phase

	//MX_MOTO_RAMP_DOWN();  //DACCEL Phase

	///////////////////////////////////////////////////////////


	while(1)
	{
		if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			while(1)
			{
				//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)
				__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(20)/100);
			}

		}
		else{}
		//MX_MOTO_RAMP_UP();
		//
	}

	while(1);
	return 0;
}



void MX_MOTO_RAMP_UP(void)
{
	for(i = 50; i< MAX_DUTYCYCLE; i ++ )
	{
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);
		HAL_Delay(20);    //This finishes the ramp up in 5sec
	}
}

void MX_MOTO_RAMP_DOWN(void)
{
	for(; i > 0; i -- )
	{
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);
		HAL_Delay(20);    //This finishes the ramp up in 5sec
	}
}



void GPIO_Init(void)
{
	//Low level and high level initialization

	GPIO_InitTypeDef btn, ledgpio, ext_btn; //Enable strucutre

	__HAL_RCC_GPIOC_CLK_ENABLE(); //Enable the clock
	__HAL_RCC_GPIOA_CLK_ENABLE();

	btn.Pin = GPIO_PIN_9;
	btn.Mode = GPIO_MODE_IT_RISING;
	btn.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOC, &ext_btn);

	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);

	btn.Pin = GPIO_PIN_13;
	btn.Mode = GPIO_MODE_INPUT;
	btn.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOC, &btn);

	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn,15,0);

}

void Timer3_Init(void)
{
	 // Input/output capture and not interrupt capture.
	/*
	 * Timer 31111 initialization and instantiation

	 * Clk = Peripheral_frequency //50MHz
	 * pre-scaler_counter_clk = clk/(pre-scaler + 1)
	 * time_period = 1/pre-scaler_counter_clk //
	 * Period = time_period * time_delay //
	 * Load the number to the ARR register.(Only 16 bit wide)
	 * side note-> Max ARR value can be upto and not more than 65535 since its a 16 bit register
	 *
	 */
	tim3.Instance = TIM3;
	tim3.Init.CounterMode = TIM_COUNTERMODE_UP; // set as up counter
	tim3.Init.Period = 3000 - 1; // for one milli second
	tim3.Init.Prescaler = 50 - 1;//49;
	if(HAL_TIM_PWM_Init(&tim3) != HAL_OK) Error_handler();  //Timer 2 is configured
	 /*
		 * Working with the timer2 Output channel for PWM generation, for more info @ref general purpose timer in reference manual
		 * 1. Init the timer Output to Compare the time base
		 * 2. Config  the output channel for PWM
	 */
	memset(&timerPWMconfig, 0 , sizeof(timerPWMconfig));
	timerPWMconfig.OCMode = TIM_OCMODE_PWM1;
	timerPWMconfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	// PWM for 0% DutyCycl
	timerPWMconfig.Pulse = tim3.Init.Period * 10/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim3, &timerPWMconfig, TIM_CHANNEL_1) != HAL_OK) Error_handler();



#if 0
	// PWM for 25% DutyCycle
	timerPWMconfig.Pulse = tim3.Init.Period * 25/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim3, &timerPWMconfig, TIM_CHANNEL_1) != HAL_OK) Error_handler();
	// PWM for 50% DutyCycle
	timerPWMconfig.Pulse = tim3.Init.Period * 50/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim3, &timerPWMconfig, TIM_CHANNEL_2) != HAL_OK) Error_handler();
	// PWM for 75% DutyCycle
	timerPWMconfig.Pulse = tim3.Init.Period * 75/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim3, &timerPWMconfig, TIM_CHANNEL_3) != HAL_OK) Error_handler();
	// PWM for 95% DutyCycle
	timerPWMconfig.Pulse = tim3.Init.Period * 95/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim3, &timerPWMconfig, TIM_CHANNEL_4) != HAL_OK) Error_handler();
#endif
}


void Timer2_Init()
{
	/*
	 * Clk = Peripheral_frequency //16MHz
	 * pre-scaler_counter_clk = clk/(pre-scaler + 1)  // Around 24 (thumb rule)
	 * time_period = 1/pre-scaler_counter_clk // 640000
	 * Period = time_period * time_delay //  100ms * time_period -> 64000
	 * Load the number to the ARR register.(Only 16 bit wide)
	 * side note-> Max ARR value can be upto and not more than 65535 since its a 16 bit register
	 *
	 */
	tim2.Instance = TIM2;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP; // set as up counter
	tim2.Init.Prescaler = 50 - 1; // 49
	tim2.Init.Period = 10000 - 1;// Important, without which timer won't start min value is 1.
	if (HAL_TIM_PWM_Init(&tim2) != HAL_OK) Error_handler();

	/*
	 * Timer 2 output channel on PA5, more info refer @datasheet, alternate functions
	 * 1. Init the Timer2 Cannel 1 to Output Compare, more info refer @reference manual, general purpose timer
	 * 2. Config the output channel for pwm
	 */
	timerPWMconfig.OCMode = TIM_OCMODE_PWM1;
	timerPWMconfig.OCPolarity = TIM_OCNPOLARITY_HIGH;
	timerPWMconfig.Pulse = 0;  //With 0

	if(HAL_TIM_PWM_ConfigChannel(&tim2, &timerPWMconfig, TIM_CHANNEL_1) != HAL_OK) Error_handler();



}


void HAL_SYSTICK_Callback()
{

	indx++;
	++tick;

	if(indx == 1000)
	{

		sprintf((char*)buf, "TICKS: %lu\r\n", tick);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
		indx = 0;
	}
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//}

void SystemClockConfig(uint8_t clock_freq)
{
	/*
				 *  Use the external clock sourced from the stlink onboard debugger mcu's clock
				 *  1. Initialize the oscillator corresponding to their respective regDef
				 *  2. Initialize the oscillator as a bypass, as it is sourced from another mcu
				 *  3. Init the RCC clock config to succefully init the HSE
				 *  4. PLL by sourcing HSE clock
				 */
				RCC_OscInitTypeDef osc_init;
				RCC_ClkInitTypeDef clk_init;

				memset(&osc_init,0, sizeof(osc_init));

				osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;  // For HSE or LSE since we need both
				osc_init.HSEState = RCC_HSE_BYPASS;  // This is for HSE
				osc_init.LSEState = RCC_LSE_ON;      // This is for LSE
				osc_init.PLL.PLLState = RCC_PLL_ON;
				osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

				switch(clock_freq)
				{
					case SYS_CLOCK_FREQ_50MHz:
					{
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 100;
						osc_init.PLL.PLLP = 2;
						osc_init.PLL.PLLQ = 2;
						osc_init.PLL.PLLR = 2;
						// PLL is  configured
						/*
						 *  In this project configure the following
						 *  1. Config AHB bus clock as 50MHz  /1
						 *  2. Config APB1 bus clock as 25MHz /2 ie divide by 2
						 *  3. Config APB2 bus clock as 25MHz /2 ie divide by 2
						 *  Configured using the RCC config registers
						 */
						clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
						clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
						clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
						clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV2;
						clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

						flash_latency = FLASH_LATENCY_1;

						break;

					}
					case SYS_CLOCK_FREQ_80MHz:
					{
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 160;
						osc_init.PLL.PLLP = 2;
						osc_init.PLL.PLLQ = 2;
						osc_init.PLL.PLLR = 2;
						// PLL is  configured
						/*
						 *  In this project configure the following
						 *  1. Config AHB bus clock as 80MHz  /1
						 *  2. Config APB1 bus clock as 40MHz /2 ie divide by 2
						 *  3. Config APB2 bus clock as 80MHz /1 ie divide by 1
						 *  Configured using the RCC config registers
						 */
						clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
						clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
						clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
						clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV2;
						clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV1;

						flash_latency = FLASH_LATENCY_2;

						break;
					}
					case SYS_CLOCK_FREQ_120MHz:
					{
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 240;
						osc_init.PLL.PLLP = 2;
						osc_init.PLL.PLLQ = 2;
						osc_init.PLL.PLLR = 2;
						// PLL is  configured
						/*
						 *  In this project configure the following
						 *  1. Config AHB bus clock as 120MHz  /1
						 *  2. Config APB1 bus clock as 40MHz /2 ie divide by 2
						 *  3. Config APB2 bus clock as 60MHz /2 ie divide by 2
						 *  Configured using the RCC config registers
						 */
						clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
						clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
						clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
						clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV4;
						clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

						flash_latency = FLASH_LATENCY_3;
						break;
					}
					/*
					 * Side Note: PLLN value must be greater than 50 and lesser than 432
					 */
					case SYS_CLOCK_FREQ_180MHz:
					{
						/*
						 * To drive the mcu to 180 MHz we need to drive power controller register
						 * 1.@Ref Power Controller and Set the regulator voltage scale 1, as per the data sheet
						 * 2.Turn on the Over drive mode
						 */
						__HAL_RCC_PWR_CLK_ENABLE(); // ALways enable the clock for anything

						__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

						__HAL_PWR_OVERDRIVE_ENABLE();

						// osc init
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 360;
						osc_init.PLL.PLLP = 2;   // default
						osc_init.PLL.PLLQ = 2;   // default
						osc_init.PLL.PLLR = 2;   // default
						// PLL is  configured
						/*
						 *  In this project configure the following -> to know more refer @Clock Configuration from the ioc
						 *  1. Config AHB bus clock as 120MHz  /1
						 *  2. Config APB1 bus clock as 40MHz /2 ie divide by 2
						 *  3. Config APB2 bus clock as 60MHz /2 ie divide by 2
						 *  Configured using the RCC config registers
						 */
						clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
						clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
						clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
						clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV4;
						clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

						flash_latency = FLASH_LATENCY_5;
					}
					default: return;
				}

			if(HAL_RCC_OscConfig(&osc_init) != HAL_OK) Error_handler();

		    // after this line if everything is okay HSE is succefully turned on
			if (HAL_RCC_ClockConfig(&clk_init, flash_latency) != HAL_OK) Error_handler();

			/*
			 * SYSTICK CONFIG
			 * Since we have changed the clock config from default frequency to the application specific
			 * We need to change the clock config  going into the arm cortex processor. (prcoessor side clock config).
			 */

			HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/ 1000);
			HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // There is a pre-scalar @Ref ClockTree
}

void LSE_Config()
{
#if 0
	/*
	 * LSE config around 32.768KHz
	 */
	RCC_OscInitTypeDef osc_init;
	memset(&osc_init,0, sizeof(osc_init));

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	osc_init.LSEState = RCC_LSE_ON;
	if(HAL_RCC_OscConfig(&osc_init)) Error_handler();
#endif
	// Selects the clock soure to output on any one of the pin
	// MCO1 = PA8 and MCO2 = PA9
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCOSOURCE_LSE, RCC_MCODIV_1);
}

void UART2_Init(void)
{
	/*
	 * High level initialization
	 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart2) != HAL_OK) Error_handler();  // If there is a problem

}


void UART1_Init(void)
{
	/*
	 * High level initialization
	 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart1) != HAL_OK) Error_handler();  // If there is a problem

}

void Error_handler(void)
{
	while(1);
}
