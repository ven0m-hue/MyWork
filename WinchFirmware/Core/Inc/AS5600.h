/*
 * AS5600.h
 *
 *  Created on: 11-Oct-2022
 *      Author: LENOVO
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "main.h"


/*
 * Structure Handler
 */
typedef struct
{
	I2C_HandleTypeDef *I2Chandle;

	uint16_t sacledAngle;
uint16_t rawAngle;

	float rotation;

	_Bool magStrength;

	uint8_t agcCount;


}AS5600_Handle_t;


/*
 *  API Calls
 */
uint8_t AS5600_Init(AS5600_Handle_t *hAS56);
uint8_t AS5600_GetRawAngle(AS5600_Handle_t* hAS56);
uint8_t AS5600_GetScaledAngle(AS5600_Handle_t* hAS56);

uint8_t AS5600_GetRawAngleIT(AS5600_Handle_t* hAS56);

/*
 * Helper Functions
 */
uint8_t writeByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data);
uint8_t readByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress);

uint8_t writeMem(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data);
uint8_t readMem(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* raw);

uint8_t writeMemIT(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data);
uint8_t readMemIT(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* raw);

/*
 *  Register Definations
 */

//Using enum instead of the macro definations
typedef enum
{

	AS5600_REGISTER_ZMCO = 0x00,

	AS5600_REGISTER_ZPOS_H, //0x01 and 0x02
	AS5600_REGISTER_ZPOS_L,

	AS5600_REGISTER_MPOS_H, //and 0x04
	AS5600_REGISTER_MPOS_L,

	AS5600_REGISTER_MANG_H, //and 0x06
	AS5600_REGISTER_MANG_L,

	AS5600_REGISTER_CONF_H, //and 0x8
	AS5600_REGISTER_CONF_L

}AS5600_ConfRegister_t;

typedef enum
{
	AS5600_REGISTER_RAWANGLE_H = 0x0C, //and 0x0D -> Lower byte |0b xxxx HHHH LLLL LLLL| only 12 bits
	AS5600_REGISTER_RAWANGLE_L = 0x0D,

	AS5600_REGISTER_ANGLE_H = 0x0E, //and 0x0F -> Lower byte |0b xxxx HHHH LLLL LLLL| only 12 bits
	AS5600_REGISTER_ANGLE_L = 0x0F

}AS5600_OpRegister_t;

typedef enum
{
	AS5600_REGISTER_STATUS = 0x0B,

	AS5600_REGISTER_AGC_H = 0x1A,

	AS5600_REGISTER_MAG_H = 0x1B,
	AS5600_REGISTER_MAG_L

}AS5600_StatusRegister_t;


typedef enum
{
	AS5600_ERROR = 0,
	AS55600_SUCCESS
}AS5600_OpStatus_t;

typedef enum{

	AS5600_MH = 0x08,
	AS5600_ML = 0x10,
	AS5600_MD = 0x20

}AS5600_AGCStatus_t;

typedef enum
{

	AS5600_NOTHEALTHY = 0,
	AS5600_HEALTHY = 1

}AS5600_Health_t;

#define BURN_CMD		0xFF
#define WHOAMI			(0x36<<1) // Device address, left shifted by one for 7bit addresses.

#define AS5600_I2C_TIMEOUT	100

#endif /* INC_AS5600_H_ */
