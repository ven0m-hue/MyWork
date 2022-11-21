/*
 * AS5600.C
 *
 *  Created on: 11-Oct-2022
 *      Author: LENOVO
 */


#include "AS5600.h"


uint8_t AS5600_Init(AS5600_Handle_t *hAS56)
{
	/*Initialize with the necessary mode*/
	uint8_t temp = 0;
	AS5600_OpStatus_t ret = AS5600_ERROR;

	/*
	 * Set Power Mode
	 * Default is the normal mode, so no need to set it.
	 */


	/*
	 * Configure chip
	 * Not really required.
	 */

	/*
	 * Get the status of the Magnetic sensor by touching the mag register sensor
	 */
	const AS5600_StatusRegister_t agcReg = AS5600_REGISTER_AGC_H;
	if(readByte(hAS56->I2Chandle, WHOAMI, agcReg) >= AS55600_SUCCESS)
	{
		ret = AS55600_SUCCESS;
		hAS56->agcCount = readByte(hAS56->I2Chandle, WHOAMI, agcReg); /* useful while debugging */
	}

	else ret = AS5600_ERROR;

	const AS5600_StatusRegister_t statusReg = AS5600_REGISTER_STATUS;

	if((temp = readByte(hAS56->I2Chandle, WHOAMI, statusReg)) >= AS55600_SUCCESS)
	{
		//temp = readByte(hAS56->I2Chandle, WHOAMI, statusReg);

		ret = (temp & AS5600_MD) ? AS55600_SUCCESS : AS5600_ERROR;
	}

	else ret = AS5600_ERROR;

	return ret;
}

uint8_t AS5600_GetRawAngle(AS5600_Handle_t* hAS56)
{
	const AS5600_OpRegister_t angleReg = AS5600_REGISTER_RAWANGLE_H;
	AS5600_OpStatus_t ret = AS5600_ERROR;

	uint8_t raw[2] = {0};

	if((hAS56->I2Chandle == NULL)) return ret;

		//readByte(hAS56->I2Chandle, Address, subAddress);
	if(readMem(hAS56->I2Chandle, WHOAMI, angleReg, raw) != AS55600_SUCCESS) return ret;

	else ret = AS55600_SUCCESS;

	hAS56->rawAngle = ((uint16_t)((uint16_t)raw[0] << 8 | raw[1]));

	return ret;
}

uint8_t AS5600_GetRawAngleIT(AS5600_Handle_t* hAS56)
{
	const AS5600_OpRegister_t angleReg = AS5600_REGISTER_RAWANGLE_H;
	AS5600_OpStatus_t ret = AS5600_ERROR;

	uint8_t raw[2] = {0};

	if((hAS56->I2Chandle == NULL)) return ret;

		//readByte(hAS56->I2Chandle, Address, subAddress);
	if(readMemIT(hAS56->I2Chandle, WHOAMI, angleReg, raw) != AS55600_SUCCESS) return ret;

	else ret = AS55600_SUCCESS;

	hAS56->rawAngle = (float)((uint16_t)((uint16_t)raw[0] << 8 | raw[1])) * 0.087;

	return ret;
}

uint8_t AS5600_GetScaledAngle(AS5600_Handle_t* hAS56)
{
	const AS5600_OpRegister_t angleReg = AS5600_REGISTER_ANGLE_H;
	AS5600_OpStatus_t ret = AS5600_ERROR;

	uint8_t raw[2] = {0};

	if((hAS56->I2Chandle == NULL)) return ret;

			//readByte(hAS56->I2Chandle, Address, subAddress);
	if(readMem(hAS56->I2Chandle, WHOAMI, angleReg, raw) != AS55600_SUCCESS) return ret;

	else ret = AS55600_SUCCESS;


	hAS56->sacledAngle = (uint16_t)((uint16_t)raw[0] << 8 | raw[1]);

	return ret;
}

uint8_t writeByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data)
{

	 uint8_t txData[] = {subAddress, data};
	 if(HAL_I2C_Master_Transmit(I2Chandle, Address, txData, 2, AS5600_I2C_TIMEOUT) != HAL_ERROR)
		 return 1;

	 else
		 return 0;

}

uint8_t readByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress)
{
	uint8_t rxData[1] = {0};
	uint8_t txData[] = {subAddress};
	HAL_I2C_Master_Transmit(I2Chandle, Address, txData, 1, AS5600_I2C_TIMEOUT);

	if(HAL_I2C_Master_Receive(I2Chandle, Address, rxData, 1, AS5600_I2C_TIMEOUT) != HAL_ERROR) return rxData[0];

	else return 0;

}

/*
 * Writes 2 bytes of data.
 * This API is non-generic and blocking mode.
 */

uint8_t writeMem(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data)
{


	return 0;
}

/*
 * Reads bytes of data
 */

uint8_t readMem(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* raw)
{

	uint8_t rawData[2];

	if(HAL_I2C_Mem_Read(I2Chandle, Address, subAddress, I2C_MEMADD_SIZE_8BIT, rawData, 2, AS5600_I2C_TIMEOUT) != HAL_ERROR)
	{
		raw[0] = rawData[0];
		raw[1] = rawData[1];

		return 1;
	}

	else return 0;

}


/*
 * Writes 2 bytes of data.
 * This API is non-generic and non-blocking mode.
 * User can have their own interface for reading the data in a seperate callback function.
 */
uint8_t writeMemIT(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data)
{


	return 0;

}


/*
 * Reads bytes of data
 * This API is non-generic and non-blocking mode.
 * User can have their own interface for reading the data in a seperate callback function.
 */
uint8_t readMemIT(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* raw)
{

	uint8_t rawData[2];

	if(HAL_I2C_Mem_Read_IT(I2Chandle, Address, subAddress, I2C_MEMADD_SIZE_8BIT, rawData, 2) != HAL_ERROR)
	{
		raw[0] = rawData[0];
		raw[1] = rawData[1];

		return 1;
	}

	else return 0;

}



