/*
 * PayloadSequence.c
 *
 *  Created on: 09-Aug-2022
 *      Author: LENOVO
 */


#include "PayloadSequence.h"



void Winch_Down_Ramp_Up_Down()
{

	for(size_t i = PWM_START; i< INTERMITENT_DC; i ++ )
	{
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);
		sprintf((char*)buf, "PWM: %d, %d\r\n", i, Pulse);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

		HAL_Delay(PWM_INTERMITANT_UP);    //This finishes the ramp up in
	}

	//Its only after this point the Spring thing needs to be activated
	poop_back = true;

	for(size_t i = INTERMITENT_DC; i> 0; i -- )
	{
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);
		sprintf((char*)buf, "PWM: %d, %d\r\n", i, Pulse);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

		HAL_Delay(PWM_RAMP_DOWN_DURATION);    //This finishes the ramp up in
	}

	Counts = Pulse;
	Length = (2 * __PI * 3.14 * Counts) * 0.1428;

}

void Winch_Down_Gp_Sequnece()
{

	while(PayloadSequence_Handle_t.gp_i >= 16)
	{
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(PWM_FIXED)/100);

		HAL_Delay(PWM_ON_DELAY(PWM_FIXED));

		sprintf((char*)buf, "Period: %d, %d\r\n", gp_i, Pulse);

		HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(0)/100);

		gp_i /= GP_DIV;
		HAL_Delay(PayloadSequence_Handle_t.gp_i);

	}
}

void Winch_Up_Ramp_Up_Down()
{
	//First things first change the direction
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);


		uint32_t loop_5 = Counts * 0.1;  //Set the threshold

		//Ramp Up Sequence

		for(size_t i = PWM_UP_START; i< INTERMITENT_DC; i ++ )
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

		for(size_t i = INTERMITENT_DC; i> 0; i -- )
			{
				if(Pulse > loop_5)
				{
					//There is enough room to spool at the current rate do nothing different.
					__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(i)/100);

					/*
					 * Debug Code
					 */
					sprintf((char*)buf, "PWM: %d, %d\r\n", i, Pulse);
					HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

					HAL_Delay(PWM_RAMP_DOWN_DURATION);    //This finishes the ramp up in
				}

				else break;
			}

			//After breaking
			//There is room to spool but not so much. Run at constant speed untill button gets triggered
			__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * _8_BIT_MAP(PWM_CONSTANT)/100);

			/*
			 * Debug Code
			 */
			sprintf((char*)buf, "About to reach the payload bay @ PWM: %d\r\n", PWM_CONSTANT);
			HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), HAL_MAX_DELAY);

}


void Bombay_Door_Open()
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


void Bombay_Door_Close()
{
	if(BOMBAY_OPEN_CLOSE > 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_2, tim3.Init.Period * _8_BIT_MAP(BOMBAY_OPEN_CLOSE)/100);

		HAL_Delay(2200);
		__HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_2, tim3.Init.Period * _8_BIT_MAP(0)/100);
	}

	else
	{
		//Halt and Do nothing.
		MX_Jump();

	}
}


void Winch_Motor_Direction_Set()
{

}

void Bombay_Door_Direction_Set()
{

}


bool Bay_Door_Close_Or_Not()
{
	/*
	 * After closing the door send signal to the FCU for the sequence completion.
	 */


}
