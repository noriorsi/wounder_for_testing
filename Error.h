/*
 * Error.h
 *
 *  Created on: 2017. márc. 31.
 *      Author: MEMS
 */

#ifndef SRC_ERROR_H_
#define SRC_ERROR_H_

/*************************************************
 * Includes
 **************************************************/
#include "em_cmu.h"
#include "em_gpio.h"
#include "RFDuino.h"
#include "SysTick.h"
#include "EmMode.h"


/*************************************************
 * Error numbers
 **************************************************/

#define 	DEFAULT_ERROR 									0
#define 	GPIO_ERROR_NUMBER 								1
#define		SEND_STRING_ENCODING_ERROR_NUMBER				2
#define		SEND_STRING_BUFFER_OVERFLOW_ERROR_NUMBER		3
#define		SEND_DOUBLE_BUFFER_OVERFLOW_ERROR_NUMBER		4
#define		SEND_INT_BUFFER_OVERFLOW_ERROR_NUMBER			5
/*************************************************
 * Functions
 **************************************************/
void PrintAndAbort(unsigned int errnum);
void ErrorHandler(unsigned int errnum);
void GPIOError(GPIO_Port_TypeDef port, unsigned int pin);

#endif /* SRC_ERROR_H_ */
