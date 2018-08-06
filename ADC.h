

#ifndef SRC_ADC_H_
#define SRC_ADC_H_

/**************************************************
 *  Includes
**************************************************/

#include "em_adc.h"
#include "em_cmu.h"
#include "GPIO.h"
#include "em_dma.h"
#include "em_timer.h"
#include "em_prs.h"
#include "rtc.h"
#include "commands.h"
#include "Modes.h"

#define DMA_CHANNEL    0
#define NUM_SAMPLES    5
#define LFRCOFREQ 32768
#define ADC_BUFFER_SIZE	64

/* Change this to increase or decrease sampling frequency. */
#define WAKEUP_US 1000
/* Change this to increase or decrease number of samples. */
#define N_SAMPLES 512

#define ADC_FORCE0		0
#define ADC_FORCE1		1
#define ADC_FORCE2		2
#define ADC_FORCE3		3
#define ADC_FORCE4		4
#define ADC_BATTERY		5

#define ILLEGAL_VALUE           0xffff          /* Invalid 12 bit ADC value */
#define N_SAMPLES               512
#define ADC_CLOCK               11000000        /* ADC conversion clock */
#define ADC_DMA_CHANNEL         0
#define ADC_GAIN_CAL_VALUE      0xffd0          /* ADC gain calibration value */

#define ADC_SE_VFS_X100         330             /* Reference VDD x 100 */
#define ADC_SE_VFS_X1000        3300            /* Reference VDD x 1000 */
#define ADC_DIFF_VFS_X100       660             /* Reference 2xVDD x 100 */
#define ADC_12BIT_MAX           4096            /* 2^12 */

#define VMCU			3.3
#define	ADC_MAX_VALUE	4096.0
#define VBAT			3.0
#define ADC_SCAN_CH_NUMBER      5

//#define RESISTOR1		10.0
//#define RESISTOR2		33.0
#define RTCC_PRS_CHANNEL        0                       /* =ADC_PRS_CH_SELECT */
#define RTCC_PRS_CH_SELECT      rtccPRSCh0              /* =ADC_PRS_CH_SELECT */
/* Change this to increase or decrease sampling frequency. */
#define RTC_WAKEUP_MS           10
#define RTC_WAKEUP_COUNT        (((32768 * RTC_WAKEUP_MS) / 1000) - 1)
//#define ADC_BATTERY		1
/**************************************************
 *  Functions
**************************************************/
void InitADC();
void InitErrata(void);
void RTC_IRQHandler_ADC(void);
void adcReset(void);
void rtcSetup(void);
void adcTImerPrs(void);
void adcTimerPrsSetup(void);
uint32_t ADC_Calibration(ADC_TypeDef *adc, ADC_Ref_TypeDef ref);
void adcDmaSetup(void);
uint32_t adcScanDma(unsigned channel);
uint32_t GetADCvalue_Force(unsigned channel);
double getVDD(unsigned int average);
double ADC_to_Voltage(uint32_t ADCvalue);
double Voltage_to_force(double volt);
//double ADC_to_Voltage_forBattery(uint32_t ADCvalue);
//uint16_t Convert32to16bit(uint32_t source);
//double BatteryVoltageMeasurement();
#endif /* SRC_ADC_H_ */
