
#include "ADC.h"


/**************************************************
 *  Initializes the ADC module
**************************************************/
void InitADC(){

		CMU_ClockEnable(cmuClock_HFPER, true);
		CMU_HFRCOBandSet(cmuHFRCOBand_11MHz);
		CMU_ClockEnable(cmuClock_ADC0, true);
		CMU_ClockEnable(cmuClock_DMA, true);

		ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	//	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;
		init.ovsRateSel = adcOvsRateSel128 ;
		init.lpfMode = adcLPFilterRC;
		init.warmUpMode = adcWarmupNormal;
		init.timebase = ADC_TimebaseCalc(0);
		init.prescale = ADC_PrescaleCalc(7000000, 0);
		init.tailgate = 0;
		//ADC_Calibration(ADC0,adcRefVDD);
		//adcScanDma();
		ADC_Init(ADC0, &init);
}

/***************************************************************************//**
* @brief Configure DMA for ADC scan mode.
*******************************************************************************/
void adcDmaSetup(void)
{
  DMA_Init_TypeDef       dmaInit;
  DMA_CfgDescr_TypeDef   descrCfg;
  DMA_CfgChannel_TypeDef chnlCfg;

  CMU_ClockEnable(cmuClock_DMA, true);

  /* Configure general DMA issues */
  dmaInit.hprot        = 0;
  //dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Configure DMA channel used */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = false;
  chnlCfg.select    = DMAREQ_ADC0_SCAN;
  chnlCfg.cb        = NULL;
  DMA_CfgChannel(ADC_DMA_CHANNEL, &chnlCfg);

  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(ADC_DMA_CHANNEL, true, &descrCfg);
}


int16_t sampleBuffer[N_SAMPLES];
uint32_t samples[NUM_SAMPLES];
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
  sampleBuffer[5] = (sampleBuffer[5] * ADC_SE_VFS_X1000) / ADC_12BIT_MAX;
  //adcReset();
  return sampleBuffer[channel];

  }


/**************************************************
 *  Makes an ADC scan on CH4 = PD4 and returns the value
**************************************************/
// the analog reading from the FSR resistor divider
uint32_t GetADCvalue(void) {

	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;
	scanInit.reference = adcRef1V25;
	scanInit.input     = ADC_SCANCTRL_INPUTMASK_CH0;
	ADC_InitScan(ADC0, &scanInit);
	ADC_Start(ADC0, adcStartScan);

	while (ADC0->STATUS & ADC_STATUS_SCANACT) ;
	//Get ADC result (FSR adc result)
	return ADC_DataScanGet(ADC0);

}

uint32_t GetADCvalue_Force(unsigned channel) {

	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;
	//ORIGINAL:
	//scanInit.reference = adcRefVDD;
	// With 1.25 V reference: in ADC init scan default the ref is 1.25 but just to be sure..
	scanInit.reference =  adcRef1V25;
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
	case ADC_FORCE3: input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH7;
	//sample = ADC_DataScanGet(ADC0);
		break;
	case ADC_FORCE4: input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH0;
	//sample = ADC_DataScanGet(ADC0);
		break;
	default:
		input_channel_mask = ADC_SCANCTRL_INPUTMASK_DEFAULT; break;
	}

	scanInit.input     = input_channel_mask;
	scanInit.resolution = ADC_SCANCTRL_RES_OVS;
	ADC_InitScan(ADC0, &scanInit);
	ADC_Start(ADC0, adcStartScan);
	while (ADC0->STATUS & ADC_STATUS_SCANACT) ;

	//ADC_Calibration(ADC0,adcRefVDD);
	return ADC_DataScanGet(ADC0);

}



uint32_t GetADCvalue_Force4(unsigned channel) {


	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;
	scanInit.reference =  adcRef1V25;

	uint32_t input_channel_mask;
	switch(channel){

	case ADC_FORCE4: input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH0;
	//sample = ADC_DataScanGet(ADC0);
		break;
	case ADC_BATTERY: input_channel_mask = ADC_SCANCTRL_INPUTMASK_CH1;
	break;
	default:
		input_channel_mask = ADC_SCANCTRL_INPUTMASK_DEFAULT; break;

	}

	scanInit.input     = input_channel_mask;
	scanInit.resolution = ADC_SCANCTRL_RES_OVS;
	ADC_InitScan(ADC0, &scanInit);
	ADC_Start(ADC0, adcStartScan);
	while (ADC0->STATUS & ADC_STATUS_SCANACT) ;

	//ADC_Calibration(ADC0,adcRefVDD);
	return ADC_DataScanGet(ADC0);

}











// the analog reading converted to voltage - not used jet
/*
double ADC_to_Voltage_forBattery(uint32_t ADCvalue){
	//Calculate with the voltage divider resistors
	return (ADCvalue / ADC_MAX_VALUE * VMCU * (RESISTOR1+RESISTOR2)/RESISTOR2);
}*/

double ADC_to_Voltage(uint32_t ADCvalue){
	//Calculate with the voltage divider resistors
	return (ADCvalue / ADC_MAX_VALUE * VREF );
}


double Voltage_to_force(double volt){
	volt *= 1000.0;			 // now its in mV
	double fsrResistance;	 //ohm
	double fsrConductance;
	double fsrForce;		 // Newton
	double vmcu = 3300.0;	 // now its in mV,
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

static uint32_t GetADCvalueVDD(){

	ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
	singleInit.reference =  adcRef1V25; //Internal 1.25V reference
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

	double vdd = ( (double)avg / ADC_MAX_VALUE)*1.5*3;

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
