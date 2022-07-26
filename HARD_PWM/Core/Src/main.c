/*
 * main.c
 *
 *  Created on: July 30, 2022
 *      Author: Venom
 */

//Includes
#include "main.h"
#include "stdbool.h"

#define MAX_DUTYCYCLE 			255   //16bit timer

#define PWM_START 				121
#define INTERMITENT_DC			180


#define STEP 					1    // Mapping it with the 8bit 255 gives each step 257 ie. 65535/255 = 257

#define PWM_FIXED				120
#define PWM_FIXED_START			PWM_FIXED


#define PWM_RAMP_UP_DURATION   35
#define PWM_INTERMITANT_UP     20   //90 //90



#define PWM_RAMP_DOWN_DURATION  85//For heavier payload  lighter payload ->75

#define PWM_WINCH_DOWN_RAMP_DOWN_DURATION	30

#define PWM_CONSTANT 			20


#define ENCODER_RAMP_UP_COUNT 		PWM_RAMP_UP_DURATION * 205
#define ENCODER_RAMP_DOWN_COUNT 	(ENCODER_RAMP_UP_COUNT + PWM_RAMP_DOWN_DURATION * 255)

#define PWM_ON_DELAY(X)		((X)*0.011776)

#define GP_DIV				2

uint16_t gp_i = 64;


#define DELAY_255			3000
#define PWM_255				255


//This part is for BOMBAY_DOORS
#define BOMBAY_OPEN_CLOSE			255
#define BOMBAY_DOOR_ONOFF_TIME		1500


//Prototypes
void SystemClockConfig(uint8_t clock_freq);
void Error_handler(void);
void UART2_Init(void);
void UART1_Init(void);
void Timer3_Init(void);
void Timer2_Init(void);
void LSE_Config(void);
void GPIO_Init(void);

void MX_WINCH_DOWN_MOTO_RAMP_UP_DOWN(void);
void MX_WINCH_UP_MOTO_RAMP_UP_DOWN(void);
void MX_WINCH_DOWN_GP_RAMP_UP(void);

void MX_BomBay_Door_Open(void);
void MX_BomBay_Door_Close(void);

void MX_Jump();

// Handle variable of general purpose timer 2 made global
TIM_HandleTypeDef tim3;
TIM_HandleTypeDef tim2;
TIM_OC_InitTypeDef timerPWMconfig;


//GPIO Handles
GPIO_InitTypeDef btn;
GPIO_InitTypeDef ledgpio;
GPIO_InitTypeDef ext_btn;
GPIO_InitTypeDef m_dir;
GPIO_InitTypeDef mdoor_dir;
GPIO_InitTypeDef b_door;
GPIO_InitTypeDef b_roof;



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
int16_t Data;
//UART1
uint8_t receivedData1;
uint8_t data_buffer1[5];
uint32_t count1 = 0;
bool recepCmplt1 = false;


bool calib = false;
uint32_t PID;
uint16_t counter = 0;
uint16_t i = 0;
uint16_t j = 0;
uint16_t indx = 0;
bool btn_pressed = false;

uint8_t buf[64];
uint32_t tick = 1;

uint32_t Data1 = 0;
uint32_t Clicks = 0;
int16_t click = 0;
int16_t Pulse = 0;


//Variable to store the Counts
uint32_t Counts = 0;


//Bool flag for the bombay door close
bool bay_door_close = false;

void Universal_Inits() {

	HAL_Init();
	SystemClockConfig(SYS_CLOCK_FREQ_50MHz);
	LSE_Config();
	Timer3_Init();
	Timer2_Init();
	UART2_Init();
	UART1_Init();
	GPIO_Init();
}


int main()
{
	//Inits

	Universal_Inits();

	memset(buf, 0, sizeof(buf));

	//Transmit to the terminal at start
	char* user_data = "REDWING LABS\r\n";
	uint16_t data_len = strlen(user_data);
	HAL_UART_Transmit(&huart2, (uint8_t*)user_data, data_len, HAL_MAX_DELAY);

	if(HAL_TIM_Encoder_Start_IT(&tim2, TIM_CHANNEL_ALL) != HAL_OK)  Error_handler();

	if (HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_1) != HAL_OK) Error_handler();
	if (HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_2) != HAL_OK) Error_handler();


//	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

//	__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(10)/100);
//	MX_MOTO_RAMP_UP();   //ACCEL Phase

//	__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(0)/100);
	//HAL_Delay(1000);   //Const Phase

//	MX_MOTO_RAMP_DOWN();  //DACCEL Phase

	///////////////////////////////////////////////////////////

	//MX_BomBay_Door_Close();
	MX_BomBay_Door_Open();

	HAL_Delay(1000);

	/*
	 * Winch Down With Payload begin Sequence
	 */
//	MX_WINCH_DOWN_GP_RAMP_UP();
//	MX_WINCH_DOWN_MOTO_RAMP_UP_DOWN();


	/*
	 * This portion of the code deals with winch up sequence
	 */
	//This is the wait period for the winch up sequence.
	//HAL_Delay(5000);

	//MX_WINCH_UP_MOTO_RAMP_UP_DOWN();


	//Until the flag for door open is not set do nothing
	while(!(bay_door_close));

	//If it breaks the loop, it means hook has reached the bay roof
	//Start the Door Close sequence
	MX_BomBay_Door_Close();


	while(1){};

}



/*
 *  Program to write and store the encoder count to the flash mem.
 *  1. Systick tracks the indx every specified time.
 *  2. Time is based on the Ramp_Up PWM
 *  3. Second time after the Ramp_Down PWM.
 */

void HAL_SYSTICK_Callback()
{

	indx++;  //Monitors the time
	++tick;  // Updates the tick


	if(indx == ENCODER_RAMP_UP_COUNT)
	{
		sprintf((char*)buf, "TICKS: %d\r\n", Pulse);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

		//Do the Flash Write Sequence here.
		//Flash_Write_Data(0x08060000, (uint32_t*)Pulse, 1);

		indx = 0;
	}

	else if(indx == ENCODER_RAMP_DOWN_COUNT)
	{
		sprintf((char*)buf, "TICKS: %d\r\n", Pulse);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

				//Do the Flash Write Sequence here.
				//Flash_Write_Data(0x08060000, (uint32_t*)Pulse, 1);

				indx = 0;
	}

	else{}

}


void MX_BomBay_Door_Open(void)
{

	if(BOMBAY_OPEN_CLOSE > 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_2, tim3.Init.Period * _8_BIT_MAP(BOMBAY_OPEN_CLOSE)/100);

		HAL_Delay(BOMBAY_DOOR_ONOFF_TIME);

		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_2, tim3.Init.Period * _8_BIT_MAP(0)/100);

	}

	else
	{
		//Halt and Do nothing.
		MX_Jump();
	}
}


void MX_BomBay_Door_Close()
{
	if(BOMBAY_OPEN_CLOSE > 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_2, tim3.Init.Period * _8_BIT_MAP(BOMBAY_OPEN_CLOSE)/100);

		HAL_Delay(1600);
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_2, tim3.Init.Period * _8_BIT_MAP(0)/100);
	}

	else
	{
		//Halt and Do nothing.
		MX_Jump();

	}
}


//
//
//void HAL_SYSTICK_Callback()
//{
//
//	indx++;
//	//++tick;
//
//	if(indx == 1000)
//	{
//		sprintf((char*)buf, "Counts: %d\r\n", Pulse);
//
////		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
//		indx = 0;
//	}
//}

void MX_WINCH_DOWN_GP_RAMP_UP(void)
{


	while(gp_i >= 16)
	{
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(PWM_FIXED)/100);

		HAL_Delay(PWM_ON_DELAY(PWM_FIXED));

		sprintf((char*)buf, "Period: %d, %d\r\n", gp_i, Pulse);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(0)/100);

		gp_i /= GP_DIV;
		HAL_Delay(gp_i);

	}

}

void MX_WINCH_DOWN_MOTO_RAMP_UP_DOWN(void)
{
	for(i = PWM_START; i< INTERMITENT_DC; i ++ )
	{
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);
		sprintf((char*)buf, "PWM: %d, %d\r\n", i, Pulse);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

		HAL_Delay(PWM_INTERMITANT_UP);    //This finishes the ramp up in
	}


	for(i = INTERMITENT_DC; i> 0; i -- )
		{
			__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);
			sprintf((char*)buf, "PWM: %d, %d\r\n", i, Pulse);

			HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

			HAL_Delay(PWM_RAMP_DOWN_DURATION);    //This finishes the ramp up in
		}


 	Counts = Pulse;


}

void MX_WINCH_UP_MOTO_RAMP_UP_DOWN(void)
{

	//First things first change the direction
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);


	uint32_t loop_5 = Counts * 0.04;  //Set the threshold

	//Ramp Up Sequence

	for(i = PWM_START; i< INTERMITENT_DC; i ++ )
	{
		if(Pulse > loop_5)
		{
			__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);
			sprintf((char*)buf, "PWM: %d, %d\r\n", i, Pulse);

			HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

			HAL_Delay(PWM_INTERMITANT_UP);    //This finishes the ramp up in

		}

		//This is unlikely to ever happen but for safety.
		else
		{
			//Write a very short but effective ramp_down so that there is not jerk at zero.
			__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(40)/100);
			HAL_Delay(20);
			__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(20)/100);
			HAL_Delay(20);
			__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(0)/100);

			//And just stop, something's fishy!
			MX_Jump();

		}

	}


	//Ramp Down Sequence

	for(i = INTERMITENT_DC; i> 0; i -- )
		{
			if(Pulse > loop_5)
			{
				//There is enough room to spool at the current rate do nothing different.
				__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);
				sprintf((char*)buf, "PWM: %d, %d\r\n", i, Pulse);

				HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

				HAL_Delay(PWM_RAMP_DOWN_DURATION);    //This finishes the ramp up in
			}

			else break;
		}

		//After breaking
		//There is room to spool but not so much. Run at constant speed untill button gets triggered
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(PWM_CONSTANT)/100);
		sprintf((char*)buf, "About to reach the payload bay @ PWM: %d\r\n", PWM_CONSTANT);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

}




void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	Clicks = __HAL_TIM_GET_COUNTER(htim);
	click = (int16_t)Clicks;
	Pulse = click * 0.25;

}


void GPIO_Init(void)
{
	//Low level and high level initialization
	__HAL_RCC_GPIOC_CLK_ENABLE(); //Enable the clock
	__HAL_RCC_GPIOA_CLK_ENABLE();

	//This is for the Door Motor
	mdoor_dir.Pin = GPIO_PIN_8;
	mdoor_dir.Mode = GPIO_MODE_OUTPUT_PP;
	mdoor_dir.Pull = GPIO_NOPULL;
	mdoor_dir.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &mdoor_dir);

	//This is for the Winch Motor
	m_dir.Pin = GPIO_PIN_0;
	m_dir.Mode = GPIO_MODE_OUTPUT_PP;
	m_dir.Pull = GPIO_NOPULL;
	m_dir.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &m_dir);


	//This is disabled for now cuz its causing interrupt at the start up, need to inspect this
	ext_btn.Pin = GPIO_PIN_1;
	ext_btn.Mode = GPIO_MODE_IT_FALLING;
	ext_btn.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &ext_btn);

	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_SetPriority(EXTI1_IRQn,15,0);


	//Ext Int Pin 2 for the roof
	b_roof.Pin = GPIO_PIN_2;
	b_roof.Mode = GPIO_MODE_IT_FALLING;
	b_roof.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &b_roof);

	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_SetPriority(EXTI2_IRQn,15,0);

	//Ext Int Pin 3 for the door
	b_door.Pin = GPIO_PIN_3;
	b_door.Mode = GPIO_MODE_IT_RISING;
	b_door.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &b_door);

	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_SetPriority(EXTI3_IRQn,15,0);

	//This is for the inbuilt led
	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);

	//This is for the inbuilt btn int
	btn.Pin = GPIO_PIN_13;
	btn.Mode = GPIO_MODE_IT_RISING;
	btn.Pull = GPIO_PULLUP;

	HAL_GPIO_Init(GPIOC, &btn);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn,15,0);

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
	timerPWMconfig.Pulse = tim3.Init.Period * 0/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim3, &timerPWMconfig, TIM_CHANNEL_1) != HAL_OK) Error_handler();


	timerPWMconfig.Pulse = tim3.Init.Period * 0/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim3, &timerPWMconfig, TIM_CHANNEL_2) != HAL_OK) Error_handler();


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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	/*
	 * Clk = Peripheral_frequency //16MHz
	 * pre-scaler_counter_clk = clk/(pre-scaler + 1)  // Around 24 (thumb rule)
	 * time_period = 1/pre-scaler_counter_clk // 640000
	 * Period = time_period * time_delay //  100ms * time_period -> 64000
	 * Load the number to the ARR register.(Only 16 bit wide)
	 * side note-> Max ARR value can be upto and not more than 65535 since its a 16 bit register
	 *
	 */
//	tim2.Instance = TIM2;
//	tim2.Init.CounterMode = TIM_COUNTERMODE_UP; // set as up counter
//	tim2.Init.Prescaler = 0; // 49
//	tim2.Init.Period = 10000 - 1;// Important, without which timer won't start min value is 1.
//	if (HAL_TIM_PWM_Init(&tim2) != HAL_OK) Error_handler();
//
//	/*
//	 * Timer 2 output channel on PA5, more info refer @datasheet, alternate functions
//	 * 1. Init the Timer2 Cannel 1 to Output Compare, more info refer @reference manual, general purpose timer
//	 * 2. Config the output channel for pwm
//	 */
//	timerPWMconfig.OCMode = TIM_OCMODE_PWM1;
//	timerPWMconfig.OCPolarity = TIM_OCNPOLARITY_HIGH;
//	timerPWMconfig.Pulse = 0;  //With 0
//
//	if(HAL_TIM_PWM_ConfigChannel(&tim2, &timerPWMconfig, TIM_CHANNEL_1) != HAL_OK) Error_handler();

	tim2.Instance = TIM2;
	tim2.Init.Prescaler = 0;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim2.Init.Period = 65535;
	tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&tim2, &sConfig) != HAL_OK)
	{
		Error_handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&tim2, &sMasterConfig) != HAL_OK)
	{
		Error_handler();
	}


}


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

//For now do nothing
void MX_Jump()
{
	while(1);
}


//////////////////
#if 0

if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
			while(1)
			{

				__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(90)/100);
			}
		}

		else
		{

		}

#endif
