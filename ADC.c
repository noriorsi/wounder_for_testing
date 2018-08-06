#include "ADC.h"


/**************************************************
 *  Initializes the ADC module
**************************************************/
int16_t sampleBuffer[N_SAMPLES];
uint32_t samples[NUM_SAMPLES];

volatile bool adcFinished = true;
volatile uint32_t sampleValue;//16?
volatile uint32_t sampleCount;
/* Buffer for ADC single and scan conversion */
uint32_t adcBuffer[ADC_BUFFER_SIZE];

int i;

void InitADC(){
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_HFRCOBandSet(cmuHFRCOBand_11MHz);
	CMU_ClockEnable(cmuClock_ADC0, true);
	CMU_ClockEnable(cmuClock_DMA, true);

	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
//	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;
	init.ovsRateSel = adcOvsRateSel2;
	init.lpfMode = adcLPFilterBypass;
	init.warmUpMode = adcWarmupNormal;
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(7000000, 0);
	init.tailgate = 0;
	//ADC_Calibration(ADC0,adcRefVDD);
	//adcScanDma();
	ADC_Init(ADC0, &init);
	//adcDmaSetup();

}


uint32_t ADC_Calibration(ADC_TypeDef *adc, ADC_Ref_TypeDef ref)
{
  int32_t  sample;
  uint32_t cal;

  /* Binary search variables */
  uint8_t high;
  uint8_t mid;
  uint8_t low;

  /* Reset ADC to be sure we have default settings and wait for ongoing */
  /* conversions to be complete. */
  ADC_Reset(adc);

  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);

  /* Set an oversampling rate for more accuracy */
  init.ovsRateSel = adcOvsRateSel4096;
  ADC_Init(adc, &init);

  /* Init for single conversion use, measure DIFF0 with selected reference. */
  singleInit.reference = ref;
  singleInit.input     = adcSingleInputDiff0;
  singleInit.acqTime   = adcAcqTime16;
  singleInit.diff      = true;
  /* Enable oversampling rate */
  singleInit.resolution = adcResOVS;

  ADC_InitSingle(adc, &singleInit);

  /* ADC is now set up for offset calibration */
  /* Offset calibration register is a 7 bit signed 2's complement value. */
  /* Use unsigned indexes for binary search, and convert when calibration */
  /* register is written to. */
  high = 128;// maybe I need 8 bit  ? 256
  low  = 0;

  /* Do binary search for offset calibration*/
  while (low < high)
  {
    /* Calculate midpoint */
    mid = low + (high - low) / 2;

    /* Midpoint is converted to 2's complement and written to both scan and */
    /* single calibration registers */
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEOFFSET_MASK | _ADC_CAL_SCANOFFSET_MASK);
    cal     |= (uint8_t)(mid - 63) << _ADC_CAL_SINGLEOFFSET_SHIFT;
    cal     |= (uint8_t)(mid - 63) << _ADC_CAL_SCANOFFSET_SHIFT;
    adc->CAL = cal;

    /* Do a conversion */
    ADC_Start(adc, adcStartSingle);
    while (adc->STATUS & ADC_STATUS_SINGLEACT)
      ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(adc);

    /* Check result and decide in which part of to repeat search */
    /* Calibration register has negative effect on result */
    if (sample < 0)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else if (sample > 0)
    {
      /* Repeat search in top half. */
      low = mid + 1;
    }
    else
    {
      /* Found it, exit while loop */
      break;
    }
  }

  /* Now do gain calibration, only INPUT and DIFF settings needs to be changed */
  adc->SINGLECTRL &= ~(_ADC_SINGLECTRL_INPUTSEL_MASK | _ADC_SINGLECTRL_DIFF_MASK);
  adc->SINGLECTRL |= (adcSingleInputCh4 << _ADC_SINGLECTRL_INPUTSEL_SHIFT);
  adc->SINGLECTRL |= (adcSingleInputCh5 << _ADC_SINGLECTRL_INPUTSEL_SHIFT);
  adc->SINGLECTRL |= (adcSingleInputCh6 << _ADC_SINGLECTRL_INPUTSEL_SHIFT);
  adc->SINGLECTRL |= (adcSingleInputCh7 << _ADC_SINGLECTRL_INPUTSEL_SHIFT);
  adc->SINGLECTRL |= (adcSingleInputCh0 << _ADC_SINGLECTRL_INPUTSEL_SHIFT);
  adc->SINGLECTRL |= (false << _ADC_SINGLECTRL_DIFF_SHIFT);

  /* Gain calibration register is a 7 bit unsigned value. */
  high = 128;
  low  = 0;

  /* Do binary search for gain calibration */
  while (low < high)
  {
    /* Calculate midpoint and write to calibration register */
    mid = low + (high - low) / 2;
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEGAIN_MASK | _ADC_CAL_SCANGAIN_MASK);
    cal     |= mid << _ADC_CAL_SINGLEGAIN_SHIFT;
    cal     |= mid << _ADC_CAL_SCANGAIN_SHIFT;
    adc->CAL = cal;

    /* Do a conversion */
    ADC_Start(adc, adcStartSingle);
    while (adc->STATUS & ADC_STATUS_SINGLEACT)
      ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(adc);

    /* Check result and decide in which part to repeat search */
    /* Compare with a value atleast one LSB's less than top to avoid overshooting */
    /* Since oversampling is used, the result is 16 bits, but a couple of lsb's */
    /* applies to the 12 bit result value, if 0xffd is the top value in 12 bit, this */
    /* is in turn 0xffd0 in the 16 bit result. */
    /* Calibration register has positive effect on result */
    if (sample > ADC_GAIN_CAL_VALUE)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else if (sample < ADC_GAIN_CAL_VALUE)
    {
      /* Repeat search in top half. */
      low = mid + 1;
    }
    else
    {
      /* Found it, exit while loop */
      break;
    }
  }
  return adc->CAL;
}



void adcReset(void)
{
  /* Rest ADC registers */
  ADC_Reset(ADC0);
  NVIC_DisableIRQ(ADC0_IRQn);

  /* Disable clocks */
  CMU_ClockEnable(cmuClock_DMA, false);
  CMU_ClockEnable(cmuClock_PRS, false);
  CMU_ClockEnable(cmuClock_TIMER0, false);

  /* Enable key interrupt, for INT and WFE example */
  //runKey = false;
  //menuKey = false;
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO->IFC = _GPIO_IFC_MASK;
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}



/***************************************************************************//**
* @brief Configure ADC for scan mode, using DMA to fetch ADC data.
*******************************************************************************/
uint32_t adcScanDma(unsigned channel)
{
	//adcReset();
	ADC_Init_TypeDef     init     = ADC_INIT_DEFAULT;
	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;
	adcDmaSetup();

	/* Init common issues for both single conversion and scan mode */
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(ADC_CLOCK/2, 0);
	ADC_Init(ADC0, &init);



		  /* Init for scan mode. */
	scanInit.reference = adcRefVDD;
	scanInit.input     = ADC_SCANCTRL_INPUTMASK_CH4 |
						 ADC_SCANCTRL_INPUTMASK_CH5 |
						 ADC_SCANCTRL_INPUTMASK_CH6 |
						 ADC_SCANCTRL_INPUTMASK_CH7 |
			             ADC_SCANCTRL_INPUTMASK_CH0 ;
		  ADC_InitScan(ADC0, &scanInit);

  DMA_ActivateBasic(DMA_CHANNEL,
                    true,
                    false,
					sampleBuffer,
                    (void *)((uint32_t)&(ADC0->SCANDATA)),
                    (NUM_SAMPLES - 1));

  /* Start Scan */
  ADC_Start(ADC0, adcStartScan);

  /* Poll for scan comversion complete */
  while (ADC0->STATUS & ADC_STATUS_SCANACT)
      ;
  sampleBuffer[0] = (ADC_DataScanGet(ADC0) * ADC_SE_VFS_X1000) / ADC_12BIT_MAX;
  sampleBuffer[1] = (sampleBuffer[1] * ADC_SE_VFS_X1000) / ADC_12BIT_MAX;
  sampleBuffer[2] = (sampleBuffer[2] * ADC_SE_VFS_X1000) / ADC_12BIT_MAX;
  sampleBuffer[3] = (sampleBuffer[3] * ADC_SE_VFS_X1000) / ADC_12BIT_MAX;
  sampleBuffer[4] = (sampleBuffer[4] * ADC_SE_VFS_X1000) / ADC_12BIT_MAX;
  //adcReset();
  return sampleBuffer[channel];

  }



double ADC_to_Voltage(uint32_t ADCvalue){
	double adc_scale = (double) ADCvalue / ADC_MAX_VALUE;
	//Calculate with the voltage divider resistors
	//return (ADCvalue / ADC_MAX_VALUE * VMCU );
	return (adc_scale * vddVoltage);
}

void adcTImerPrs(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  adcTimerPrsSetup();

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  ADC_Init(ADC0, &init);

  /* Init for single conversion. */
  singleInit.input = adcSingleInputCh3;
  singleInit.reference  = adcRefVDD;
  /* Enable PRS for ADC */
  singleInit.prsEnable  = true;
  ADC_InitSingle(ADC0, &singleInit);

  /* Select TIMER0 as source and TIMER0OF (Timer0 overflow) as signal (rising edge) */
  CMU_ClockEnable(cmuClock_PRS, true);
  PRS_SourceSignalSet(ADC_FORCE0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgePos);

  /* Enable ADC Interrupt when Single Conversion Complete */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

  /* Enable ADC interrupt vector in NVIC*/
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);

  adcFinished = false;
  sampleValue = ILLEGAL_VALUE;


  TIMER_Enable(TIMER0, true);

  /* Exit if PB0 is pressed */
  while (!adcFinished)
  {
    /* Enter EM1 and wait for timer triggered ADC conversion */
    //EMU_EnterEM1();
    if (sampleValue != ILLEGAL_VALUE)
    {
      /* Write result to LCD */
      sampleValue = (sampleValue * ADC_SE_VFS_X1000) / ADC_12BIT_MAX;
     // SegmentLCD_Number(sampleValue);
      sampleValue = ILLEGAL_VALUE;
    }
  }

  /* Stop TIMER0 */
  TIMER_Enable(TIMER0, false);
 // LCD_NUMBER_OFF();
 //SegmentLCD_Write("PRS");
  adcReset();
}



/**************************************************
 *  Makes an ADC scan on CH4 = PD4 and returns the value,
 *  CH5 = PD5
 *  CH6 = PD6
 *  CH7 = PD7
 *  CH0 = PE12
**************************************************/
// the analog reading from the FSR resistors divider

// I invite them in Modes.c

uint32_t GetADCvalue_Force(unsigned channel) {
	//CMU_ClockEnable(cmuClock_ADC0, true);
//uint32_t sample;
//adcReset();
	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;
	scanInit.reference = adcRefVDD;
	uint32_t input_channel_mask;
	//while (ADC0->STATUS & ADC_STATUS_SCANACT) ;
	switch(channel){
	case ADC_FORCE0: input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH4;
	//	sample = ADC_DataScanGet(ADC0);
		break;
	case ADC_FORCE1: input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH5;
	//sample = ADC_DataScanGet(ADC0);
		break;
	case ADC_FORCE2: input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH6;
	//sample = ADC_DataScanGet(ADC0);
		break;
	case ADC_FORCE3:input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH7;
	//sample = ADC_DataScanGet(ADC0);
		break;
	case ADC_FORCE4: input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH0;
	//sample = ADC_DataScanGet(ADC0);
		break;
	default:
		input_channel_mask = ADC_SCANCTRL_INPUTMASK_DEFAULT; break;

	}

	scanInit.input     = input_channel_mask;
	ADC_InitScan(ADC0, &scanInit);
	ADC_Start(ADC0, adcStartScan);
	while (ADC0->STATUS & ADC_STATUS_SCANACT) ;

	//ADC_Calibration(ADC0,adcRefVDD);
	return ADC_DataScanGet(ADC0);

}


/**********************************************************************/


// the analog reading converted to voltage - not used jet
/*
double ADC_to_Voltage_forBattery(uint32_t ADCvalue){
	//Calculate with the voltage divider resistors
	return ((ADCvalue / ADC_MAX_VALUE * VMCU * (RESISTOR1+RESISTOR2)/RESISTOR2);
}*/
/*******************************************************************************
*******************CALCULATIONS************************************************
 * *****************************************************************************
 */


double Voltage_to_force(double volt){
	volt *= 1000.0;			 // now its in mV
	double fsrResistance;	 //ohm
	double fsrConductance;
	double fsrForce;		 // Newton
	double vmcu = 3300.0;	 // now its in mV
	fsrResistance = vmcu - volt;
	fsrResistance *= 10000.0;// 10K resistor in ohm
	fsrResistance /= volt;
	fsrConductance = 1000000.0;
	fsrConductance /= fsrResistance;

		if (fsrConductance<=1000.0){
					fsrForce = fsrConductance/80.0; //first figure from datasheet
				}else{
					fsrForce = fsrConductance - 1000.0;
					fsrForce /= 30.0; 				//second figure from datasheet
				}
		return fsrForce;
}


/*double BatteryVoltageMeasurement(){
	SetGPIO(BAT_MEAS_EN_PORT, BAT_MEAS_EN_PIN, 1);
	uint32_t adc_temp = GetADCvalue();
	SetGPIO(BAT_MEAS_EN_PORT, BAT_MEAS_EN_PIN, 0);
	return ADC_to_Voltage(adc_temp);
}*/

/**************************************************
 *  Truncates the 32 bit unsigned to a 16 bit unsigned
**************************************************/
/*
uint16_t Convert32to16bit(uint32_t source){
	return (source>>16);
}*/

// MIKI MAGIC

static uint32_t GetADCvalueVDD(){

	ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
	singleInit.reference = adcRef2V5; //Internal 2.5V reference
	singleInit.input = adcSingleInputVDDDiv3;
	ADC_InitSingle(ADC0, &singleInit);
	ADC_Start(ADC0, adcStartSingle);
	while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

	return ADC_DataSingleGet(ADC0);
}

double getVDD(unsigned int average){
	uint32_t sum = 0;

	for(int i=0; i<average; ++i){
		uint32_t temp = GetADCvalueVDD();
		if(i>0) sum+=temp;
	}

	uint32_t avg = sum/(average-1);

	if(average<=1) avg = GetADCvalueVDD();

	double vdd = ( (double)avg / ADC_MAX_VALUE)*2.5*3;

	return vdd;
}

// Battery ADC_SCANCTRL_INPUTMASK_CH1
double BatteryVoltageMeasurement(){
	uint32_t adc_temp = 0;
	SetGPIO(BAT_MEAS_EN_PORT, BAT_MEAS_EN_PIN, 1);
	adc_temp = GetADCvalue_Force(ADC_BATTERY);
	SetGPIO(BAT_MEAS_EN_PORT, BAT_MEAS_EN_PIN, 0);

	return ADC_to_Voltage(adc_temp);
}
