/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "myMav.h"
/* Private includes ----------------------------------------------------------*/
//#include "usb_device.h"
//#include "usbd_cdc_if.h"
/* Private typedef -----------------------------------------------------------*/

/*
 * TODO:
 * 1. Test rig compatible winch 2.0 full routine.
 */

/* Structure definitions ------------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

I2C_HandleTypeDef hi2c1;
AS5600_Handle_t as5600;
/* Private macro -------------------------------------------------------------*/
/*
 * @main.h
 */

/* Private variables ---------------------------------------------------------*/
//UART2 variables
uint8_t receivedData;
uint8_t data_buffer[5];
uint32_t count;
bool recepCmplt;
int16_t Data = 0;
uint32_t Data1 = 0;
uint8_t buf[64];



//Timer 2 IC Variables
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0;
uint32_t usWidth = 0;
float frequency = 0;


//Encoder Variables
uint16_t LastRead = 0;
uint16_t CurrRead = 0;
uint16_t rawAngle = 0;
int16_t rev = 0;
int16_t Counts = 0;
float Length = 0.0f;

//Bool flag for the Winch Start Seq
bool Start_Flag = false;
uint32_t trig = 0;  //keep tack of since the inception
bool e_stop = false;
bool START_THE_SEQUENCE = false;

//Spring thing variables
bool poop_back = false;
bool spring_trig = false;
__IO uint32_t spring_trig_count = 0;

//Bay Door variables
bool close_door = false;
bool bay_door_close = false;


//Global Application level variables
uint16_t i = 0;  //PWM Ramp index

//Variable for the GP sequence
uint16_t gp_i = 128;
float leg_len = 0;
//Usb Variables
//uint8_t buf[64];
//uint8_t buffer[5];
//int16_t Data;
//uint32_t Data1 = 0;

//Systick Handler
uint8_t buf_tick[64];
uint16_t indx = 0;

//PID Handler
uint32_t tick = 0;
uint32_t prevTick = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART1_Init(void);
static void MX_UART2_Init(void);
static void MX_I2C1_Init(void);

//Application side
static void MX_WINCH_START_SEQ(void);
static void MX_WINCH_DOWN_MOTO_RAMP_UP_DOWN(void);
static void MX_WINCH_UP_MOTO_RAMP_UP_DOWN(void);
static void MX_WINCH_DOWN_GP_RAMP_UP(void);
static void MX_SOFT_START_P_CONTROLLER_RAMP_UP(void);
void MX_WINCH_P_CONTROLLER(void);

static void MX_BomBay_Door_Open(void);
static void MX_BomBay_Door_Close(void);
static void MX_Jump(void);

/* Private user code ---------------------------------------------------------*/
void MX_Universal_Init()
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  //MX_USB_DEVICE_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART1_Init();
  MX_UART2_Init();
  MX_I2C1_Init();


}

void MX_Peripheral_Start_Init()
{
	/*
	 *  1. Encoder IT Start
	 *  2. PWM CH1, CH2 Start
	 *  3. TIM Input Cpature Mode Start
	 *  4. ADC DMA Start
	 */
	if(HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL) != HAL_OK)  Error_Handler();

	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

	if(HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1)!= HAL_OK) Error_Handler();

	//if(HAL_ADC_Start_DMA(&hadc1, &Buf, 1) != HAL_OK) Error_Handler();

	memset(buf, 0, sizeof(buf));

	//Transmit to the terminal at start to confirm the initiation.
	//char* user_data = "REDWING LABS\r\n";
	//uint16_t data_len = strlen(user_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)user_data, data_len, HAL_MAX_DELAY);

	/*
	 * 1.GPIO PIN Inits
	 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	/*
	 * AS5600 ME init
	 * Get the raw angle.
	 * Store the Init raw angle to a global var.
	 * Then count the revolutions on the basis of that raw angle.
	 * i.e. if the raw angle is 20 deg, then every time the angle goes above 20 is one revolution.
	 */
	as5600.I2Chandle = &hi2c1;
	while(!AS5600_Init(&as5600))
	{
		sprintf((char*)buf, "Can't detect the Magnet\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
		HAL_Delay(500);
	}

	HAL_Delay(500); /*Time to set*/

	AS5600_GetRawAngle(&as5600);
	CurrRead = as5600.rawAngle;

	LastRead = CurrRead;

	rev = 0;
	Length = 0;
	//sprintf((char*)buf, "Initial Angle : %d\r\n", rawAngle);
	//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
//
	memset(buf, 0, sizeof(buf));

	/*Transmit to the terminal at start to confirm the initiation.*/
	//user_data = "Initialization successful\r\n";
	//data_len = strlen(user_data);
	//HAL_UART_Transmit(&huart1, (uint8_t*)user_data, data_len, HAL_MAX_DELAY);


}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/
	MX_Universal_Init();

  /* MCU Peripherals Start Commands-------------------------------------------*/
	MX_Peripheral_Start_Init();
	//HAL_UART_MspDeInit(&huart1);


//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(60)/100);
  /* Application Level Calls--------------------------------------------------*/

	/*
	 * Winch Start Sequence
	 * 1. One reception of a signal of particular width start the winch sequence
	 * or else keep looping until forever.
	 */
	MX_WINCH_START_SEQ();

	HAL_GPIO_WritePin(blue_led_GPIO_Port, blue_led_Pin, GPIO_PIN_SET);


	/*
	 * Winch Down With Payload Sequence
	 *
	 * 1. Initial GP sequence so as to reduce the initial stuttering issue.
	 * 2. Winch Down with Time Based RampUp and RampDown Sequence.
	 * 3. Spring thing activated, once the payload soft lands the spring triggers.
	 * 4. In the backend the encoder keeps counting and stores the final count as the setPoint for the winch up sequence.
	 *
	 * Spring triggering is the end of Winch Down Sequence.
	 */
	//MX_BomBay_Door_Open();

	//HAL_Delay(2000); //Delay for the door to settle and prep for winch down.

	MX_WINCH_DOWN_GP_RAMP_UP();


	/*
	 * This portion of the code deals with winch up sequence,
	 * 1. For now time based wait, specified now for testing 5 sec.
	 * 2. Stating the Motor Ramp Up sequence.
	 * 3. After threshold counts elapses run at a constant rate. Wait for the hook to trigger the roof switch.
	 * 4. Once the roof switch triggers, initiate the Mission End Sequence.
	 * 5. Mission End Sequence: Cut down the motor supply, close the door.
	 *
	 * note TODO: current thing for detecting if payload is landed.
	 *
	 */

	MX_SOFT_START_P_CONTROLLER_RAMP_UP();

	MX_WINCH_P_CONTROLLER();


	leg_len = Length; //Store the length of the first leg.

	HAL_Delay(8000);  //Delay time


	MX_WINCH_UP_MOTO_RAMP_UP_DOWN();

	//Start the Door Close sequence
	//MX_BomBay_Door_Close();

	while(1)
	{}

	return 0;

}

///////////////////////////////////////////////////////////////////////////////////////////APPLICATION_LEVEL_ROUTINES//////////////////////////////////////////////////////////////


static void MX_WINCH_START_SEQ()
{
	/*
	 * This part loops until a signal of particular pulse width is captured.
	 */
	//UNUSED();
	while(!(START_THE_SEQUENCE)){};

	Start_Flag = false;

	//MavLinkReceiveHoverCurr(&huart2, receivedData);

}


void MX_BomBay_Door_Open(void)
{

	if(BOMBAY_OPEN_CLOSE > 0)
	{
		HAL_GPIO_WritePin(winch_dir_GPIO_Port, winch_dir_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(BOMBAY_OPEN_CLOSE)/100);

		HAL_Delay(BOMBAY_DOOR_ONOFF_TIME);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);

	}

	else
	{
		//Halt and Do nothing.
		MX_Jump();
	}
}


void MX_BomBay_Door_Close()
{
	//To avoid false triggers
	while(!(bay_door_close));

	if(BOMBAY_OPEN_CLOSE > 0)
	{
		HAL_GPIO_WritePin(winch_dir_GPIO_Port, winch_dir_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(BOMBAY_OPEN_CLOSE)/100);

		HAL_Delay(2200);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);
	}

	else
	{
		//Halt and Do nothing.
		MX_Jump();

	}
}


void MX_WINCH_DOWN_GP_RAMP_UP(void)
{


	while(gp_i >= 16)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(PWM_FIXED)/100);

		HAL_Delay(PWM_ON_DELAY(PWM_FIXED));

		//sprintf((char*)buf, "PWM: %d, Length: %f\r\n", i, Length);
		//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
		//CDC_Transmit_FS((uint8_t *)buf, sizeof(buf));

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(0)/100);

		gp_i /= GP_DIV;
		HAL_Delay(gp_i);

	}

}

void MX_WINCH_DOWN_MOTO_RAMP_UP_DOWN(void)
{
	for(i = PWM_START; i< INTERMITENT_DC; i ++ )
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(i)/100);
		//sprintf((char*)buf, "PWM: %d, Length: %f\r\n", i, Length);

		//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
		//CDC_Transmit_FS((uint8_t *)buf, sizeof(buf));

		HAL_Delay(PWM_INTERMITANT_UP);    //This finishes the ramp up in
	}



	for(i = INTERMITENT_DC; i> 0; i -- )
		{

			if(!spring_trig)
			{

				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(i)/100);
				//current = ((float) Buf * (VREF_3v3 / ADC_SCALE_12) - 2.5) * 10;

				//sprintf((char*)buf, "PWM: %d, %d, %f\r\n", i, rev, current);
				//sprintf((char*)buf, "PWM: %f, RPM: %f\r\n", i*0.019605, rev);
				//sprintf((char*)buf, "PWM: %d, Length: %f\r\n", i, Length);
				//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

				HAL_Delay(PWM_RAMP_DOWN_DURATION);    //This finishes the ramp up in

				//Spring Thing
				//poop_back = true;

				//Its only after this point the Spring thing and the Current thing needs to be activated
					if( Length > POOP_BACK_AT_H )
					{
						//Spring Thing
						poop_back = true;

					}

			}

			else
			{
				break;
			}

		}



 	Counts = rev;

 	//Length =  -((2 * __PI * 3.14 * Counts) * 0.1428);

 	//sprintf((char*)buf, "PWM | Current | Length: %d, %d, %f, %f\r\n", i, rev, ((float) Buf * (VREF_3v3 / ADC_SCALE_12) - 2.5) * 10, Length);
 	//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
 	//CDC_Transmit_FS((uint8_t *)buf, sizeof(buf));

 	memset(buf, 0, sizeof(buf));

}

void MX_WINCH_UP_MOTO_RAMP_UP_DOWN(void)
{

	//First things first change the direction
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //GPIOC and PIN_0 changed

	uint32_t loop_5 = Counts * 0.1;  //Set the threshold

	//Ramp Up Sequence

	for(i = PWM_UP_START; i< INTERMITENT_DC; i ++ )
	{
		if(rev > loop_5)
		{
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(i)/100);
			sprintf((char*)buf, "PWM: %d, Length: %f\r\n", i, Length);

			//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
			//CDC_Transmit_FS((uint8_t *)buf, sizeof(buf));

			HAL_Delay(PWM_INTERMITANT_UP);    //This finishes the ramp up in

		}

		//This is unlikely to ever happen but for safety.
		else
		{
			//Write a very short but effective ramp_down so that there is not jerk at zero.
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(40)/100);
			HAL_Delay(20);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(20)/100);
			HAL_Delay(20);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);

			//And just stop, something's fishy!
			MX_Jump();

		}

	}


	//Ramp Down Sequence

	for(i = INTERMITENT_DC; i> 60; i -- )
		{
			if(rev > loop_5)
			{
				//There is enough room to spool at the current rate do nothing different.
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(i)/100);
				sprintf((char*)buf, "PWM: %d, Length: %f\r\n", i, Length); //i*0.019605

				//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
				//CDC_Transmit_FS((uint8_t *)buf, sizeof(buf));

				HAL_Delay(PWM_RAMP_DOWN_DURATION);    //This finishes the ramp up in
			}

			else break;
		}


	//Its only after this point we need to activate the door close interrupt flag
	close_door = true;


	//After breaking
	//There is room to spool but not so much. Run at constant speed untill button gets triggered
	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(PWM_CONSTANT)/100);
	//sprintf((char*)buf, "About to reach the payload bay @ PWM: %d\r\n", PWM_CONSTANT);

	//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
	//CDC_Transmit_FS((uint8_t *)buf, sizeof(buf));

}


static void MX_SOFT_START_P_CONTROLLER_RAMP_UP(void)
{
	for(i = PWM_START; i< INTERMITENT_DC; i ++ )
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(i)/100);
		//sprintf((char*)buf, "PWM: %d, Length: %f\r\n", i, Length);

		//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
		//CDC_Transmit_FS((uint8_t *)buf, sizeof(buf));

		HAL_Delay(PWM_INTERMITANT_UP);    //This finishes the ramp up in
	}
}


/*
 * This sub-routine interfaces the P-based controller
 */
void MX_WINCH_P_CONTROLLER(void)
{
	PID_Handle_t pid;
	uint32_t motor_output = 0;

	pid.Ts = 10; // 10 milliseconds.
	pid.kp = 1.5;
	PID_Init(&pid);



	while((Length <= LEN_TO_WINCH_DOWN)  && !(spring_trig))
	{
		motor_output = P_Compute(&pid, Length, LEN_TO_WINCH_DOWN, uwTick);

		if(motor_output <= 30) motor_output = __8BIT_OUTPUT_MIN;

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(motor_output)/100);

		//sprintf((char*)buf, "PWM: %ld, Length: %f, Tick: %ld\r\n", motor_output, Length, tick); //i*0.019605

		//HAL_Delay(10);

		if(Length >= THRESHOLD_LEN)
		{
			poop_back = true;

			/*
			 * After Threshold length is spooled,
			 * 1. Keep asking for mavlink based packets.
			 * 2. Verify the data from here on.
			 * 3. Keep a bool variable to keep track.
			 * 4. If the current drops by more than set amount of current then break the sequence.
			 *
			 *
			 * Way to do this is enable a flag at this point, could make use of poop_back
			 * Once this flag is enabled, mavlink packets for current data could be requested in the interrupt mode.
			 * Defer the interrupt, here to enable one more flag so that it can do further computation and arrive at a decision.
			 */


//			if((curr_curr > 0) && (curr_curr < hover_curr))
//			{
//				//Stop the motor?
//
//				//Then Store the Currene data in prev_curr
//				prev_curr = curr_curr;
//
//				//Disable the current_affair
//				poop_back = false;
//				curr_curr = 0;
//			}

		//	else curr_curr = 0; /*This is to assure that new value has been updated*/

		}
		//Flag
		//prevTick = tick;

		//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);


	}

	//__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * 0/100);


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	for(int i =0; i<12000; i++)
	{

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(PAYLOAD_3)/100);

	}

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, htim3.Init.Period * _8_BIT_MAP(0)/100);

	if(rev < 0) Counts = -rev;
	else Counts = rev;

}

/////////////////////////////////////////////////////////////////////////////////////////////INTERRUPT_CALLBACKS/////////////////////////////////////////////////////////////////

/*
 *  Current Sensor DMA Callback
 */

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	current = ((float) Buf * (VREF_3v3 / ADC_SCALE_12) - 2.5) * 10;
//	//sprintf((char*)buf, "@Current: %f\r\n", current);
//	//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
//
//	if(curr)
//	{
//		current = ((float) Buf * (VREF_3v3 / ADC_SCALE_12) - 2.5) * 10;
//		if((current < 1.0f)  &&  (current > -1.0f))
//		{
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);
//			sprintf((char*)buf, "Payload Soft landed:@Current: %f\r\n", current);
//			//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);
//			CDC_Transmit_FS(buf, sizeof(buf));
//		}
//	}
//
//}




/*
 * This sub routine does the input signal matching via the IC compare interrupt
 * Based on the pre-defined signal type, routine scans for the particular DC signal
 * If obtained initiates the Winch Start Sequence.
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		if (Is_First_Captured==0) // if the first rising edge is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
		}

		else   // If the first rising edge is captured, now we will capture the second edge
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			float refClock = TIMCLOCK/(PRESCALAR);

			frequency = refClock/Difference;

			float mFactor = 1000000/refClock;

			usWidth = Difference*mFactor;

			//Here the conditions begin, for now we can keep it simple
			if(usWidth >= THROTTLE_FULL)
			{
				//Do Something
				Start_Flag = true;

			}

			else if(usWidth >= THROTTLE_HALF && usWidth < THROTTLE_FULL)
			{
				//Do Something different
				Start_Flag = false;
				trig = 0;

			}

			else if(usWidth < THROTTLE_HALF)
			{
				//Do something else
				e_stop = true;
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period * _8_BIT_MAP(0)/100);
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

			}
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_First_Captured = 0; // set it back to false
		}
	}
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

	if(Start_Flag)
	{
		++trig;
		if(trig >= 5000)
		{
			START_THE_SEQUENCE = true;
			Start_Flag = false;

			//DeInit the IC interrupt
			HAL_TIM_IC_MspDeInit(&htim4);


		}


		else {
			START_THE_SEQUENCE = false;
		}
	}

	if(indx == 10)  // every 10 millisecond
	{
		//Calculate the rpm
		indx = 0;
		AS5600_GetRawAngle(&as5600);

		CurrRead = as5600.rawAngle;

		if((LastRead - CurrRead)  > 2047) rev ++;

		if((LastRead - CurrRead)  < -2047) rev --;

		//sprintf((char*)buf, "Rev : %d\r\n", rev);
		//HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

		LastRead = CurrRead;

		if(rev < 0) Length = (2 * __PI * __RADIUS * (-rev)) * 0.01;   //Converting centi to meters

		else Length = (2 * __PI * __RADIUS * (rev)) * 0.01;   //Converting centi to meters

		}


	else{}

}




/////////////////////////////////////////////////////////////////////////////////////////////LOW_LEVEL_PERIPHERALS_INITs////////////////////////////////////////////////////////////////


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
}



/**
  * @brief UART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART1_Init(void)
{
	/*
	 * High level initialization
	 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();  // If there is a problem

}

/**
  * @brief UART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART2_Init(void)
{
	/*
	 * High level initialization
	 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();  // If there is a problem

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* @note:

  	 * Clk = Peripheral_frequency //16MHz
  	 * pre-scaler_counter_clk = clk/(pre-scaler + 1)  // Around 24 (thumb rule)
  	 * time_period = 1/pre-scaler_counter_clk // 640000
  	 * Period = time_period * time_delay //  100ms * time_period -> 64000
  	 * Load the number to the ARR register.(Only 16 bit wide)
  	 * side note-> Max ARR value can be upto and not more than 65535 since its a 16 bit register
  	 *
  	 */

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

	/*
	 * Timer 3 initialization and instantiation

	 * Clk = Peripheral_frequency //50MHz
	 * pre-scaler_counter_clk = clk/(pre-scaler + 1)
	 * time_period = 1/pre-scaler_counter_clk //
	 * Period = time_period * time_delay //
	 * Load the number to the ARR register.(Only 16 bit wide)
	 * side note-> Max ARR value can be upto and not more than 65535 since its a 16 bit register
	 *
	 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 60-1; //60MHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64 - 1;  //Generates 15KHz frequency signal.
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 60-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, winch_dir_Pin|bay_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : spring_thing_ext_Pin */
  GPIO_InitStruct.Pin = spring_thing_ext_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(spring_thing_ext_Pin_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_SetPriority(EXTI3_IRQn,15,0);

  /*Configure GPIO pins : winch_dir_Pin bay_dir_Pin */
  GPIO_InitStruct.Pin = winch_dir_Pin|bay_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : roof_top_ext_Pin */
  GPIO_InitStruct.Pin = roof_top_ext_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(roof_top_ext_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI0_IRQn,15,0);

  /*Configure GPIO pins : internal LED */
  GPIO_InitStruct.Pin = blue_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(blue_led_GPIO_Port, &GPIO_InitStruct);


}


/////////////////////////////////////////////////////////////////////////////////////Error Handlers//////////////////////////////////////////////////////

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void MX_Jump()
{
	__disable_irq();
	while(1);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{

	/*
	 * Can Add ASSERT Statements here, for debugging purpose only.
	 */
  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
