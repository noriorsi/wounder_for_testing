/*
 * SysTick.c
 *
 *  Created on: 23 Feb 2017
 *      Author: Miklós
 */

#include "SysTick.h"

/* Counts 1ms timeTicks */
volatile uint32_t msTicks = 0;

void InitSysTick(){
	SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000);
}

void Delay(uint32_t dlyTicks)
{
      uint32_t curTicks;

      curTicks = msTicks;
      while ((msTicks - curTicks) < dlyTicks) ;
}

void SysTick_Handler(void)
{
      /* Increment counter necessary in Delay()*/
      msTicks++;
}

/*************************************************************
 * returns the current time
 *************************************************************/
uint32_t getTime(void){
	return msTicks;
}
