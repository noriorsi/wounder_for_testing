/*
 * StateMachine.h
 *
 *  Created on: 2017. aug. 15.
 *      Author: szmik_000
 */

#ifndef SRC_STATEMACHINE_H_
#define SRC_STATEMACHINE_H_

/**************************************************
 *  Includes
**************************************************/

#include "RFDuino.h"
#include "Modes.h"
#include "Commands.h"

#define 	delay_between_measurement	4 //s
/**************************************************
 *  Globals
**************************************************/


typedef enum{
  IDLE_STATE,
  MODE2_STATE,
  WAITING_FOR_COMMAND_STATE,
  MAX_STATES
}state_enum;

typedef enum{
  NO_EVENT,
  STOP_EVENT,
  STARTM2_EVENT,
  RFDUINO_GPIO_IT_EVENT,
  RTC_IT_EVENT,
  TIMEOUT_EVENT,
  MAX_EVENTS
}event_enum;

extern state_enum (*state_table[MAX_STATES][MAX_EVENTS])(void);
extern state_enum state;
extern state_enum next_state;
extern event_enum event;

#define		WAITING_FOR_COMMAND_STATE_TIMEOUT	10 //ms

/**************************************************
 *  Functions
**************************************************/

state_enum No_Event_Handler(void);
state_enum Stop_Event_Handler(void);
state_enum Error_Event_Handler(void);
state_enum StartM2_Event_Handler(void);
state_enum RFduino_GPIO_IT_Event_Handler(void);
state_enum RTC_IT_Event_Handler(void);

void TimeoutChecker();

state_enum Timeout_Event_Handler(void);


#endif /* SRC_STATEMACHINE_H_ */
