/*
 * main.h
 *
 *  Created on: Jun 22, 2021
 *      Author: 91900
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define SYS_CLOCK_FREQ_50MHz  			50
#define SYS_CLOCK_FREQ_80MHz 			80
#define SYS_CLOCK_FREQ_120MHz			120
#define SYS_CLOCK_FREQ_180MHz			180


#define DutyCycle(X)      ((X)*0.033 + 33)
#define DutyCycleServo(X) ((X)*0.3611111111 + 15)

#define _8_BIT_MAP(X)         ((X)*0.3921568627)

#endif /* MAIN_H_ */
