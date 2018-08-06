
/***************** TUTORIAL *****************/
/*			 How to add new GPIO
 *
 * In this example I am going to go through the steps needed to setup the ADC0 analog input pin on PD4.
 * The ADC setup is located in ADC.h but we will not need those here. We only need to know that we want to use PD4.
 *
 * The steps:
 * 1.) In GPIO.h (this file) add the following lines:
 * //--------ADC--------//
 *	#define 	ADC0_PORT		gpioPortD
 *	#define 	ADC0_PIN		4
 *
 * 2.) In GPIO.h (at the structs field) add the following:
 * GPIO_struct ADC_0;
 *
 * 3.) In GPIO.c make a new static function to give starting value to the struct made in 2.)
 * static void InitADC(){
 *	SetGPIO_struct(ADC_0, ADC0_PORT, ADC0_PIN, gpioModeInput, 0, false);
 * }
 *
 * 4.) Call this static function in InitGPIO
 *
 * 5.) Build and upload, if no error messages showed up on the LCD it is all set.
 */




#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_


/*************************************************
 * Includes
 **************************************************/
#include "em_cmu.h"
#include "em_gpio.h"
#include "Error.h"
#include "ADC.h"
#include "StateMachine.h"


/*************************************************
 * Defines
 **************************************************/
#define 	true 	1
#define 	false 	0
#define 	TRUE 	1
#define 	FALSE 	0

#define MAX_GPIO_COUNT 					37

#define 	NUMBER_OF_RFDUINO_PINS		4
#define 	NUMBER_OF_MCULED_PINS		3
#define 	NUMBER_OF_ADC_PINS			5
#define 	RX_PIN_INT_MASK				(1<<USART_RX_PIN)


//--------SI7021--------//

#define 	I2C_SCL_PORT	gpioPortC
#define 	I2C_SCL_PIN		1

#define 	I2C_SDA_PORT	gpioPortC
#define 	I2C_SDA_PIN		0


//--------ADC--------//
// Force0
#define 	ADC0_PORT		gpioPortD
#define 	ADC0_PIN		4
// Force1
#define		ADC1_PORT		gpioPortD
#define 	ADC1_PIN		5
// Force2
#define 	ADC2_PORT		gpioPortD
#define 	ADC2_PIN		6
// Force3
#define 	ADC3_PORT		gpioPortD
#define 	ADC3_PIN		7
// Force4
#define 	ADC4_PORT		gpioPortE
#define 	ADC4_PIN		12


//Battery voltage
//Battery measurement enable pin
// not used yet
#define 	BAT_MEAS_EN_PORT	gpioPortC
#define 	BAT_MEAS_EN_PIN		4

//--------RFduino--------//
// I will change PD6 to PB11 for button always high-testing fsr
//#define	Button1_PORT			gpioPortD
//#define 	Button1_PIN				6

#define 	RFDuino_IT_PORT		gpioPortF
#define 	RFDuino_IT_PIN		5

#define 	USART_TX_PORT		gpioPortE
#define 	USART_TX_PIN		10

#define 	USART_RX_PORT		gpioPortE
#define 	USART_RX_PIN		11

//--------LEDS--------//
#define 	MCULED1_PORT		gpioPortA
#define 	MCULED1_PIN			1

#define 	MCULED2_PORT		gpioPortA
#define 	MCULED2_PIN			2

#define 	MCULED3_PORT		gpioPortC
#define 	MCULED3_PIN			2


/*************************************************
 * Structs
 **************************************************/

typedef struct GPIO_struct{
	GPIO_Port_TypeDef port;
	unsigned int pin;
	GPIO_Mode_TypeDef mode;
	unsigned int out;
	unsigned int isSet;
}GPIO_struct;

GPIO_struct EveryGPIO[MAX_GPIO_COUNT]; //struct that stores every setup gpio

//GPIO_struct Button1; //struct that stores 1 push button setup

//GPIO_struct SPI[NUMBER_OF_SPI_PINS];
GPIO_struct ADC[NUMBER_OF_ADC_PINS];
GPIO_struct SI7021;
GPIO_struct RFduino[NUMBER_OF_RFDUINO_PINS];

GPIO_struct MCULEDS[NUMBER_OF_MCULED_PINS];
GPIO_struct BatteryMeas;

/*************************************************
 * Functions
 **************************************************/
void InitGPIO();

void SetGPIO(GPIO_Port_TypeDef port, unsigned int pin, unsigned int data);
unsigned GetGPIO(GPIO_Port_TypeDef port, unsigned int pin);
void GPIOSetup(GPIO_struct gpio);
void GPIO_Unified_IRQ(void);
void GPIO_ODD_IRQHandler(void);
void GPIO_EVEN_IRQHandler(void);
unsigned StartMode();
void FlashLeds(unsigned int n);
#endif /* SRC_GPIO_H_ */
