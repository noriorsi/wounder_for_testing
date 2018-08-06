/*
 * RTC.h
 *
 *  Created on: 2017. aug. 24.
 *      Author: szmik_000
 */

#ifndef SRC_RTC_H_
#define SRC_RTC_H_

/**************************************************
 *  Includes
**************************************************/
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_rtc.h"
#include "GPIO.h"
#include "StateMachine.h"
#include "EEPROM.h"
#include "ADC.h"


/**************************************************
 *  Globals
**************************************************/

#define 	true	1
#define		false	0

#define 	RTC_IT_INTERVAL		1000	// ms

typedef enum{
  SECOND,
  MINUTE,
  HOUR,
  MAX_TIMES
}time_enum;

typedef struct{
	int seconds;
	int minutes;
	int hours;
}RTC_time_struct;

typedef struct{
	uint16_t 	year;
	uint8_t 	month;
	uint8_t 	day;
	uint8_t 	hour;
	uint8_t 	minute;
}RTC_date_struct;


/**************************************************
 *  Functions
**************************************************/
void InitRTC();
void RTC_IRQHandler();

int getTimeRTC(time_enum t);
RTC_time_struct getTimeStructRTC();
void RTC_Setup(CMU_Select_TypeDef osc);
unsigned didElapseGivenSeconds(int seconds, RTC_time_struct previous_time );
void SendDate();

#endif /* SRC_RTC_H_ */
