#include "em_device.h"
#include "em_chip.h"
#include "SysTick.h"
#include "GPIO.h"
#include "RFDuino.h"
#include "SI7021.h"
#include "ADC.h"
#include "EmMode.h"
#include "Modes.h"
#include "TIME.h"
#include "StateMachine.h"
#include "flash.h"
#include "EEPROM.h"



int main(void)
{

  /* Chip errata */
  CHIP_Init();
  InitRFDuino();
  InitCMD();
  InitSysTick();
  InitGPIO();
  InitSI7021();
  InitADC();

  InitEmulatedEEPROM();
  InitRTC();
  //InitFlash();
  FlashLeds(3);
  InitRFduinoUART();
  EnterPowerSaving();


  //SetGPIO(MCULED1_PORT,MCULED1_PIN,1);
  //Delay (1000);
  //SetGPIO(MCULED1_PORT,MCULED1_PIN,0);
  /* Infinite loop */
  while (1) {


	 //SetGPIO(MCULED3_PORT,MCULED3_PIN,1);

	   TimeoutChecker();


	  if( (event >= 0) && (event < MAX_EVENTS) ){
	 	    next_state = state_table[state][event]();
	 	    state = next_state;
	 	  }

	    }

}
