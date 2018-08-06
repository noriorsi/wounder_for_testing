/*
 * GPIO.c
 *
 *  Created on: 20 Feb 2017
 *      Author: Miklós
 */

#include "GPIO.h"
#include "RFDuino.h"
/******************************************************************************************
 * 				STATIC VARIABLES
 *******************************************************************************************/
unsigned int EveryGPIO_member_count = 0; //The number of GPIOs initialized so far
unsigned int startMode = 0;
/******************************************************************************************
 * 				STATIC FUNCTIONS
 *******************************************************************************************/

static void SetGPIO_struct(GPIO_struct this, GPIO_Port_TypeDef port, unsigned int pin, GPIO_Mode_TypeDef mode, unsigned int out, unsigned int isSet ){
	this.port = port;
	this.pin = pin;
	this.mode = mode;
	this.out = out;
	this.isSet = isSet;

	GPIOSetup(this);
}



//This function gives a starting value to the struct variable "EveryGPIO"
static void Setup_EveryGPIO_struct(){
	for(int i=0; i<MAX_GPIO_COUNT; ++i){
		EveryGPIO[i].port = 0;
		EveryGPIO[i].pin = 0;
		EveryGPIO[i].mode = 0;
		EveryGPIO[i].out = 0;
		EveryGPIO[i].isSet = false; //Set every value to signal that no GPIO has been set so far
	}
}

//This functions saves a GPIO_struct to the next place in EveryGPIO
static void SaveToEveryGPIO(GPIO_struct this){
	EveryGPIO[EveryGPIO_member_count].port = this.port;
	EveryGPIO[EveryGPIO_member_count].pin = this.pin;
	EveryGPIO[EveryGPIO_member_count].mode = this.mode;
	EveryGPIO[EveryGPIO_member_count].out = this.out;
	EveryGPIO[EveryGPIO_member_count].isSet = this.isSet;

	EveryGPIO_member_count++;
}

/* This functions returns false if a GPIO cannot be used.
 * Possible causes: - The GPIO is already set by user.
 * 					- The GPIO is on the list of unavaialble GPIOs
 */
static unsigned isGPIOAvailable(GPIO_struct gpio){
	for(int i=0; i<EveryGPIO_member_count; ++i){
		if(EveryGPIO[i].isSet){ //If there is an already set gpio
			if(gpio.port == EveryGPIO[i].port){ //If it is on the same port
				if(gpio.pin == EveryGPIO[i].pin){ //And also on the same pin
					return false; //this gpio is already taken and cannot be used
				}
			}
		}
	}
	return true;
}


/*************************************************
 * Returns on which pin occured the IT
 * If there was no IT returns -1
 **************************************************/
static int getITpin(){
	for(int i=0; i<16;++i){ //There are a total of 16 possible IT pins according to datasheet
		uint16_t mask = 1<<i;
		if(GPIO->IF & mask) return i;
	}
	return -1;
}


static void InitADC_GPIO(){
	SetGPIO_struct(ADC[0], ADC0_PORT, ADC0_PIN, gpioModeInput, 0, false);
	SetGPIO_struct(ADC[1], ADC1_PORT, ADC1_PIN, gpioModeInput, 0, false);
	SetGPIO_struct(ADC[2], ADC2_PORT, ADC2_PIN, gpioModeInput, 0, false);
	SetGPIO_struct(ADC[3], ADC3_PORT, ADC3_PIN, gpioModeInput, 0, false);
	SetGPIO_struct(ADC[4], ADC4_PORT, ADC4_PIN, gpioModeInput, 0, false);

}

//Sets up interrupt for the desired pins
static void SetupInterrupt(){
	  GPIO_IntConfig(USART_RX_PORT, USART_RX_PIN, true, false, true);
	 //GPIO_IntConfig(MAX30100_INT1_PORT, MAX30100_INT1_PIN, true, false, true);
	  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	  NVIC_EnableIRQ(GPIO_ODD_IRQn);

}
 static void InitRFduino(){
 	SetGPIO_struct(RFduino[3], RFDuino_IT_PORT, RFDuino_IT_PIN, gpioModePushPull, 0, false);
 }

static void InitMCULEDS(){
	SetGPIO_struct(MCULEDS[0], MCULED1_PORT, MCULED1_PIN, gpioModePushPull, 0, false);
	SetGPIO_struct(MCULEDS[1], MCULED2_PORT, MCULED2_PIN, gpioModePushPull, 0, false);
	SetGPIO_struct(MCULEDS[2], MCULED3_PORT, MCULED3_PIN, gpioModePushPull, 0, false);
}
// not used yet
static void InitBatteryMeas(){
	SetGPIO_struct(BatteryMeas, BAT_MEAS_EN_PORT, BAT_MEAS_EN_PIN, gpioModePushPull, 0, false);
}

/*************************************************
 * Initializes GPIO and our pins
 **************************************************/
void InitGPIO(){
	CMU_ClockEnable(cmuClock_GPIO, true); //Enable GPIO peripheral

	Setup_EveryGPIO_struct(); //Initializes the struct
	InitADC_GPIO();
	SetupInterrupt();
	InitRFduino();
	InitMCULEDS();
	InitBatteryMeas(); // not used
}

void SetGPIO(GPIO_Port_TypeDef port, unsigned int pin, unsigned int data){
	if(data==1) GPIO_PinOutSet(port, pin);
	else 		GPIO_PinOutClear(port, pin);
}


/*************************************************
 * Reads the value of a pin.
 **************************************************/
unsigned GetGPIO(GPIO_Port_TypeDef port, unsigned int pin){
	return GPIO_PinInGet(port,pin);
}


/*************************************************
 * Uses his own structure to setup a gpio
 **************************************************/
void GPIOSetup(GPIO_struct gpio){
	if(isGPIOAvailable(gpio)){
		GPIO_PinModeSet(gpio.port, gpio.pin, gpio.mode, gpio.out);
		gpio.isSet = true;
		SaveToEveryGPIO(gpio);
	}
	else{ //Error handling
		GPIOError(gpio.port, gpio.pin);
	}
}


void GPIO_Unified_IRQ(void){

	uint32_t mask = 0;
#ifndef LEDS_OFF
	SetGPIO(MCULED1_PORT, MCULED1_PIN, 1);

	for(int i=0; i<100; ++i); //a small delay only for a flash of a led
	SetGPIO(MCULED1_PORT, MCULED1_PIN, 0);
#endif
	switch(getITpin()){
		case USART_RX_PIN:{
			event = RFDUINO_GPIO_IT_EVENT;
			mask = 1<<USART_RX_PIN;
			break;
		}

		default: break;
	}

	GPIO_IntClear(mask);
	GPIO_IntDisable(mask);


}


/**************************************************************************//**
 * @brief GPIO Interrupt handler for odd pins.
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

void GPIO_EVEN_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}


/*************************************************
 * Returns if the button has been pushed
 **************************************************/
unsigned StartMode(){
	return startMode;
}


void FlashLeds(unsigned int n){

#ifdef LEDS_OFF
	return;
#endif

	for(int i =0; i<n; ++i){
		SetGPIO(MCULED1_PORT, MCULED1_PIN, 1);
		SetGPIO(MCULED2_PORT, MCULED2_PIN, 1);
		SetGPIO(MCULED3_PORT, MCULED3_PIN, 1);
		Delay(30);
		SetGPIO(MCULED1_PORT, MCULED1_PIN, 0);
		SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);
		SetGPIO(MCULED3_PORT, MCULED3_PIN, 0);
		Delay(30);
	}
}
