
/*
 * RFDuino.h
 *
 *  Created on: 2017. júl. 13.
 *      Author: szmik_000
 */

#ifndef SRC_RFDUINO_H_
#define SRC_RFDUINO_H_

/**************************************************
 *  Includes
**************************************************/
#include "em_cmu.h"
#include "em_usart.h"
#include "em_gpio.h"
#include <stdio.h>
#include <stdarg.h>
#include "em_core.h"
#include <stdlib.h>
#include <math.h>
#include "SysTick.h"
#include "GPIO.h"
#include "Error.h"
#include "StateMachine.h"
#include "Commands.h"

/**************************************************
 *  Functions
**************************************************/
void InitRFDuino();

void USART0_RX_IRQHandler(void);
void USART0_TX_IRQHandler(void);

uint16_t fix_overflow(uint16_t index);

void send_int(int data);
void send_double(double data);
void send_string(char* string);

void SendEmpty(unsigned n);

void send_debug(char* string);

void send_RFDuino_command(char* cmd);

void RFDuino_GiveIT();

void SendRXBuffer();

void InitRFduinoUART();

/**************************************************
 *  Global Defines
**************************************************/
#define USER_LOCATION 0 //This is where the USART is physically located

#define  PRINT_BUFFER_SIZE     	512
#define  RX_BUFFER_SIZE		  	100

#define 	true 	1
#define 	false 	0
#define 	TRUE 	1
#define 	FALSE 	0

#define RESOLUTION 3 //resolution after decimal point

#define BAUD 9600

#define DELAY_AFTER_SENDING 10 //ms




#define BUF_SIZE    64
#define RX_SIZE     60

#define SEND_TYPE_IDENTIFIERS
/**************************************************
 *  Structs
**************************************************/
typedef struct print_buffer
{
      uint16_t head;
      uint16_t tail;
      uint8_t data[PRINT_BUFFER_SIZE];
      uint8_t now_printing;
} print_buffer_struct;


typedef struct rx_buff{
	uint16_t index;
	uint8_t data[RX_BUFFER_SIZE];
}rx_buffer_struct;

#endif /* SRC_RFDUINO_H_ */

